"""JSON request runner for offline planning and DROID reset execution.

Use --request for one JSON document, or send one request per stdin line to keep
manager state across observe/reset/verify operations. Object poses can come from
camera markers or the caller. The detected buffer pose is cached.
--dummy supplies example coordinates; --detect-objects uses the actual cube.
--execute enables reset motion.
--collect-one detects a table cube and chooses a free buffer cell automatically.
Execution homes before capturing the scene.
"""

import argparse
from contextlib import ExitStack, redirect_stdout
import json
from pathlib import Path
import sys
import traceback

import cv2
import numpy as np

from auto_reset import ResetManager
from auto_reset.calibration import BufferCalibration
from auto_reset.hardware import DroidRobot, ZedWristCamera
from auto_reset.io import EventLog, StationConfig, new_log_path


def handle_request(manager, config, request, *, execute=False):
    """Apply a supplied observation or operate on the existing manager."""
    operation = request.get("operation", "plan")
    if operation not in ("observe", "plan", "reset", "collect", "verify", "state"):
        raise ValueError(f"Unknown operation: {operation}")
    if operation == "reset" and not execute:
        raise ValueError("Reset execution requires --execute")
    if ("scene" in request and request["scene"].get("planar_scene") != config.data.get("planar_scene")):
        raise ValueError("Saved scene uses a different height/localization model; teach the target again")
    source = ("supplied" if "observations" in request else "camera"
              if operation in ("observe", "reset", "collect") or request.get("detect_objects") else "existing observations")
    manager._record("request_received", operation=operation, object_pose_source=source,
                    detect_buffer=bool(request.get("detect_buffer", False)))
    if operation == "collect":
        planar = config.data.get("planar_scene")
        if planar is None:
            raise ValueError("Collecting a cube requires configured table and buffer support heights")
        if not manager.objects:
            raise ValueError("Configure a cube template before collecting markers")
        options = config.data.get("reset", {})
        if execute:
            home = np.asarray(options.get("home_joints_rad"), dtype=float)
            if home.shape != (7,) or not np.isfinite(home).all():
                raise ValueError("Set reset.home_joints_rad to seven finite home joint angles")
            print("Moving to DROID home before observing the scene...", file=sys.stderr, flush=True)
            manager._record("collect_home_started", joints=home.tolist())
            manager.robot.command_joints(home.tolist(), blocking=True)
            manager.robot.wait_until_stopped(timeout_s=manager.motion_timeout_s)
            manager._record("collect_home_finished")
        # The existing catalog factory supplies the same geometry for unbound IDs.
        original = set(manager.objects)
        if config.data.get("cube_dictionary_template") is None:
            config.data["cube_dictionary_template"] = next(iter(manager.objects))
        for key, obj in config.make_objects().items():
            if key not in manager.objects:
                manager.register_object(obj)
        manager.observe(detect_objects=True, detect_buffer=request.get("detect_buffer", False),
                        capture_count=options.get("scan_capture_count", 5),
                        retry_missing=False)
        for key in tuple(manager.objects):
            if key not in original and manager.objects[key].pose is None:
                manager.remove_object(key)
        visible = [obj for obj in manager.objects.values() if obj.pose is not None]
        if not visible:
            raise ValueError("No ArUco cube detected; inspect the saved scan images")
        grid = manager.buffer
        size = grid.cell_size_mm / 1000
        low = -np.asarray(grid.origin) * size
        high = (np.array([grid.columns, grid.rows]) - grid.origin) * size
        buffer_base = np.linalg.inv(grid.pose)
        footprints, sources = {}, []
        for obj in visible:
            half = np.asarray(obj.dimensions_m[:2]) / 2
            corners = np.array([[-half[0], -half[1], 0], [half[0], -half[1], 0],
                                [half[0], half[1], 0], [-half[0], half[1], 0]])
            pose = buffer_base @ obj.pose
            footprint = (corners @ pose[:3, :3].T + pose[:3, 3])[:, :2].astype(np.float32)
            footprints[obj.object_id] = footprint
            if not (np.all(footprint >= low) and np.all(footprint <= high)):
                sources.append(obj)
        if not sources:
            print("All visible cubes are already inside the buffer.", file=sys.stderr, flush=True)
            return {"operation": "collect", "success": True, "status": "already_in_buffer",
                    "visible_objects": [obj.object_id for obj in visible]}
        source = min(sources, key=lambda obj: (obj.marker_dictionary, obj.marker_id, obj.object_id))
        free = []
        for column in range(grid.columns):
            for row in range(grid.rows):
                if grid.item_at(column, row) is not None:
                    continue
                center = grid.cell_center_m(column, row)[:2]
                polygon = (center + np.array([[-size/2, -size/2], [size/2, -size/2],
                                             [size/2, size/2], [-size/2, size/2]])).astype(np.float32)
                if any(cv2.intersectConvexConvex(polygon, footprint)[0] > 0
                       for key, footprint in footprints.items() if key != source.object_id):
                    continue
                free.append((column, row))
        free.sort(key=lambda cell: np.linalg.norm(grid.cell_center_m(*cell)[:2] - (low+high)/2))
        target = None
        observation_pose = manager.robot.read_state().tool_pose.copy()
        for column, row in free:
            candidate = config.make_target({source.object_id: {"cell": [column, row],
                                            "support_height_m": 0.0}}, grid, manager.objects)
            try:
                manager.plan(candidate, solve_ik=True, rescan_objects=False,
                             observation_tool_pose=observation_pose)
            except RuntimeError as exc:
                if "no feasible IK solution" not in str(exc):
                    raise
                manager._record("collect_cell_unreachable", column=column, row=row, reason=str(exc))
                continue
            target = candidate
            break
        if target is None:
            raise ValueError("No unoccupied, reachable buffer cell is available for the detected cube")
        print(f"Collect marker {source.marker_id} ({source.object_id}) into buffer cell ({column}, {row}).",
              file=sys.stderr, flush=True)
        manager._record("collect_target_selected", object_id=source.object_id, marker_id=source.marker_id,
                        column=column, row=row, target_buffer_pose=target.pose_for(source.object_id).tolist())
        response = {"operation": "collect", "object_id": source.object_id,
                    "marker_id": source.marker_id, "cell": [column, row]}
        if not execute:
            response["actions"] = manager.plan(target, solve_ik=True,
                                               rescan_objects=request.get("rescan_objects"),
                                               observation_tool_pose=observation_pose)
            return response
        manager.reset(target, use_current_observations=True, rescan_objects=request.get("rescan_objects"))
        response.update(success=manager.state["success"],
                        placement_errors=manager.state["object_placement_errors"],
                        failure_reason=manager.state["failure_reason"])
        return response
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
    inputs.add_argument("--collect-one", action="store_true",
                        help="Detect a table cube and select a free buffer cell; --execute homes first and performs the reset")
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
    if args.collect_one:
        request = {"operation": "collect", "detect_objects": True}
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
            or request.get("operation") in ("reset", "collect")
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
            if request is not None and request.get("operation") == "collect":
                request["detect_buffer"] = True
            else:
                with redirect_stdout(sys.stderr):
                    manager.observe(detect_buffer=True)
        records = [json.dumps(request)] if request is not None else sys.stdin
        for record in records:
            if not record.strip():
                continue
            try:
                with redirect_stdout(sys.stderr):
                    response = handle_request(manager, config, json.loads(record), execute=args.execute)
                response["log_path"] = str(log.path)
                print(json.dumps(response, allow_nan=False), flush=True)
                if args.collect_one and response.get("success") is False:
                    return 1
            except KeyboardInterrupt:
                manager.cancel()
                print(json.dumps({"error": "Interrupted", "state": dict(manager.state),
                                  "log_path": str(log.path)}), flush=True)
                return 130
            except Exception as exc:
                log.emit("request_failed", reason=str(exc), traceback=traceback.format_exc())
                print(json.dumps({"error": str(exc), "state": dict(manager.state),
                                  "log_path": str(log.path)}), flush=True)
                if request is not None:
                    return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
