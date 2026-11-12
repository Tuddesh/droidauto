"""JSON request runner for offline planning and DROID reset execution.

Use --request for one JSON document, or send one request per stdin line to keep
manager state across observe/reset/verify operations. Object poses can come from
camera markers or the caller. The detected buffer pose is cached.
--dummy supplies example coordinates; --detect-objects uses the actual cube.
--execute enables reset motion.
"""

import argparse
from contextlib import ExitStack, redirect_stdout
import json
from pathlib import Path
import sys
import traceback

from auto_reset import ResetManager
from auto_reset.calibration import BufferCalibration
from auto_reset.hardware import DroidRobot, ZedWristCamera
from auto_reset.io import EventLog, StationConfig, new_log_path


def handle_request(manager, config, request, *, execute=False):
    """Apply a supplied observation or operate on the existing manager."""
    operation = request.get("operation", "plan")
    if operation not in ("observe", "plan", "reset", "verify", "state"):
        raise ValueError(f"Unknown operation: {operation}")
    if operation == "reset" and not execute:
        raise ValueError("Reset execution requires --execute")
    source = ("supplied" if "observations" in request else "camera"
              if operation in ("observe", "reset") or request.get("detect_objects") else "existing observations")
    manager._record("request_received", operation=operation, object_pose_source=source,
                    detect_buffer=bool(request.get("detect_buffer", False)))
    if operation == "verify":
        poses = {key: config.read_pose(value) for key, value in request["observations"].items()}
        manager.verify(poses, timestamp=request.get("timestamp_s"))
    else:
        if "observations" in request:
            if request.get("detect_objects"):
                raise ValueError("Choose detect_objects or supplied observations")
            manager.observe({key: config.read_pose(value) for key, value in request["observations"].items()},
                            timestamp=request.get("timestamp_s"))
        elif operation != "reset" and (operation == "observe" or request.get("detect_objects")):
            manager.observe(detect_objects=True, detect_buffer=request.get("detect_buffer", False))
        if operation in ("plan", "reset"):
            target = config.make_target(request["targets"], manager.buffer, manager.objects)
            if operation == "plan":
                current = request.get("current_tool_pose")
                actions = manager.plan(target, current_tool_pose=None if current is None else config.read_pose(current),
                                       solve_ik=request.get("solve_ik", False),
                                       rescan_objects=request.get("rescan_objects"))
                return {"actions": actions, "state": dict(manager.state)}
            print("Feedback reset: rescan and reassess after each manipulation", file=sys.stderr, flush=True)
            manager.reset(target, rescan_objects=request.get("rescan_objects"),
                          use_current_observations="observations" in request,
                          detect_buffer=request.get("detect_buffer", False))
    response = {"state": dict(manager.state)}
    if operation == "observe":
        response["observations"] = manager.observe()
    return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default_config = Path("configs/station.local.json")
    parser.add_argument("--config", type=Path, default=default_config if default_config.exists()
                        else Path("configs/station.example.json"))
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument("--request", type=Path, help="Run one request; otherwise read JSON lines from stdin")
    inputs.add_argument("--dummy", action="store_true",
                        help="Supply example object coordinates; buffer is detected or loaded from calibration")
    inputs.add_argument("--detect-objects", action="store_true",
                        help="Detect the actual cube and use the example buffer target; --execute enables movement")
    parser.add_argument("--execute", action="store_true", help="Connect DROID and allow reset requests")
    parser.add_argument("--scan-buffer", action="store_true", help="Capture the stationary buffer again instead of loading its saved pose")
    parser.add_argument("--review-buffer", action="store_true", help="Show frozen buffer/object scans and wait for accept/retake/cancel")
    parser.add_argument("--no-review", action="store_true", help="Skip interactive capture review for automated JSON callers")
    parser.add_argument("--initialize-controllers", action="store_true", help="Explicitly relaunch DROID controllers at startup")
    args = parser.parse_args()
    if args.review_buffer and args.no_review:
        parser.error("Choose --review-buffer or --no-review")
    config = StationConfig(args.config)
    config.camera["review_capture"] = not args.no_review and (args.review_buffer or sys.stdin.isatty())
    if args.dummy or args.detect_objects:
        args.request = config.resolve("reset.example.json")
    if args.dummy:
        print(f"Using dummy object coordinates from {args.request}; buffer pose comes from detection", file=sys.stderr, flush=True)
    # Parse one-shot input before opening hardware or initializing controllers.
    request = json.loads(args.request.read_text()) if args.request else None
    if args.dummy or args.detect_objects:
        request["operation"] = "reset" if args.execute else "plan"
    if args.detect_objects:
        request.pop("observations", None)
        request["detect_objects"] = True
        request["solve_ik"] = True
    with ExitStack() as stack:
        # Even a plan needs a tool-state connection when capturing the buffer.
        # Connecting does not initialize controllers or send a movement command.
        robot = stack.enter_context(DroidRobot(config.data["robot_endpoint"], ik_config=config.data.get("ik")))
        log = EventLog(new_log_path(config.resolve(config.data["log_directory"])), echo=False)
        stack.callback(log.close)
        options = config.data.get("reset", {})
        calibration_path = config.resolve(config.data["calibration_file"])
        calibration = (BufferCalibration.load(calibration_path)
                       if calibration_path.exists() and not args.scan_buffer else None)
        camera = None
        needs_camera = (
            args.scan_buffer or request is None or request.get("detect_objects", False)
            or request.get("operation") == "reset"
            or (request.get("operation") == "observe" and "observations" not in request)
            or (calibration is None and request.get("operation", "plan") in ("observe", "plan", "reset"))
        )
        if config.camera and needs_camera:
            with redirect_stdout(sys.stderr):
                camera = stack.enter_context(ZedWristCamera(
                    str(config.camera["serial"]), config.camera["view"],
                    auto_exposure_roi=config.camera.get("auto_exposure_roi"),
                ))
        manager = ResetManager(buffer=config.make_buffer(), robot=robot,
                               camera=camera, station_config=config, calibration=calibration,
                               planner=config.make_planner(), objects=config.make_objects(), log=log,
                               position_tolerance_m=options.get("position_tolerance_m", 0.01),
                               orientation_tolerance_rad=options.get("orientation_tolerance_rad", 0.1),
                               motion_timeout_s=options.get("motion_timeout_s", 60.0))
        if args.initialize_controllers:
            with redirect_stdout(sys.stderr):
                robot.initialize()
        if args.scan_buffer:
            with redirect_stdout(sys.stderr):
                manager.observe(detect_buffer=True)
        records = [json.dumps(request)] if args.request else sys.stdin
        for record in records:
            if not record.strip():
                continue
            try:
                with redirect_stdout(sys.stderr):
                    response = handle_request(manager, config, json.loads(record), execute=args.execute)
                response["log_path"] = str(log.path)
                print(json.dumps(response, allow_nan=False), flush=True)
            except KeyboardInterrupt:
                manager.cancel()
                print(json.dumps({"error": "Interrupted", "state": dict(manager.state),
                                  "log_path": str(log.path)}), flush=True)
                return 130
            except Exception as exc:
                log.emit("request_failed", reason=str(exc), traceback=traceback.format_exc())
                print(json.dumps({"error": str(exc), "state": dict(manager.state),
                                  "log_path": str(log.path)}), flush=True)
                if args.request:
                    return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
