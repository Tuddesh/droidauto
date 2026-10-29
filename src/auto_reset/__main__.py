"""JSON request runner for offline planning and DROID reset execution.

Use --request for one JSON document, or send one request per stdin line to keep
manager state across observe/reset/verify operations. No image localization or
camera acquisition is performed. Hardware access is enabled only by --execute.
"""

import argparse
from contextlib import ExitStack, redirect_stdout
import json
from pathlib import Path
import sys

from auto_reset import ResetManager
from auto_reset.hardware import DroidRobot
from auto_reset.io import EventLog, StationConfig, new_log_path


def handle_request(manager, config, request, *, execute=False):
    """Apply a supplied observation or operate on the existing manager."""
    operation = request.get("operation", "plan")
    if operation not in ("observe", "plan", "reset", "verify", "state"):
        raise ValueError(f"Unknown operation: {operation}")
    if operation == "reset" and not execute:
        raise ValueError("Reset execution requires --execute")
    if operation == "verify":
        poses = {key: config.read_pose(value) for key, value in request["observations"].items()}
        manager.verify(poses, timestamp=request.get("timestamp_s"))
    else:
        if "observations" in request:
            manager.observe({key: config.read_pose(value) for key, value in request["observations"].items()},
                            timestamp=request.get("timestamp_s"))
        if operation in ("plan", "reset"):
            target = config.make_target(request["targets"], manager.buffer, manager.objects)
            if operation == "plan":
                current = request.get("current_tool_pose")
                actions = manager.plan(target, current_tool_pose=None if current is None else config.read_pose(current))
                return {"actions": actions, "state": dict(manager.state)}
            manager.reset(target)
    return {"state": dict(manager.state)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/station.local.json"))
    parser.add_argument("--request", type=Path, help="Run one request; otherwise read JSON lines from stdin")
    parser.add_argument("--execute", action="store_true", help="Connect DROID and allow reset requests")
    parser.add_argument("--initialize-controllers", action="store_true", help="Explicitly relaunch DROID controllers at startup")
    args = parser.parse_args()
    if args.initialize_controllers and not args.execute:
        parser.error("--initialize-controllers requires --execute")
    config = StationConfig(args.config)
    with ExitStack() as stack:
        robot = stack.enter_context(DroidRobot(config.data["robot_endpoint"])) if args.execute else None
        log = EventLog(new_log_path(config.resolve(config.data["log_directory"])), echo=False)
        stack.callback(log.close)
        options = config.data.get("reset", {})
        manager = ResetManager(buffer=config.make_buffer(), robot=robot,
                               planner=config.make_planner(), objects=config.make_objects(), log=log,
                               position_tolerance_m=options.get("position_tolerance_m", 0.01),
                               orientation_tolerance_rad=options.get("orientation_tolerance_rad", 0.1),
                               motion_timeout_s=options.get("motion_timeout_s", 60.0))
        if args.initialize_controllers:
            with redirect_stdout(sys.stderr):
                robot.initialize()
        records = [args.request.read_text()] if args.request else sys.stdin
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
                print(json.dumps({"error": str(exc), "state": dict(manager.state),
                                  "log_path": str(log.path)}), flush=True)
                if args.request:
                    return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
