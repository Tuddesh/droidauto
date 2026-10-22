"""Lab-PC entry point: calibrate once, inspect targets, then run the hover task."""

import argparse
import json
import sys

from .calibration import BufferCalibration, capture_calibration
from .camera import ZedWristCamera
from .config import StationConfig
from .motion import HoverPlanner
from .robot import DroidRobot
from .task import EventLog, MarkerHoverTask, new_log_path
from .transforms import transform_to_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("calibrate", "preview", "run"):
        command = commands.add_parser(name)
        command.add_argument("--config", required=True, help="Station JSON file")
        if name != "preview":
            command.add_argument("--initialize", action="store_true",
                                 help="Explicitly restart DROID arm and gripper controllers")
        if name == "run":
            command.add_argument("--cycles", type=int, help="Omit to loop until Ctrl+C")
    args = parser.parse_args()
    config = StationConfig(args.config)
    calibration_path = config.resolve(config.data["calibration_file"])
    if args.command == "calibrate":
        if calibration_path.exists() or calibration_path.with_suffix(".png").exists():
            raise FileExistsError(f"Archive the existing calibration before recalibrating: {calibration_path}")
        with DroidRobot(config.data["robot_endpoint"]) as robot:
            if args.initialize:
                robot.initialize()
            with ZedWristCamera(str(config.camera["serial"]), config.camera["view"]) as camera:
                calibration = capture_calibration(config, robot, camera)
                # Validate the table-normal convention before accepting this pose.
                HoverPlanner(config, calibration)
                calibration.save(calibration_path)
        print(f"Saved calibration: {calibration_path}")
        print(json.dumps(calibration.record["quality"], indent=2))
        return

    calibration = BufferCalibration.load(calibration_path)
    planner = HoverPlanner(config, calibration)
    if args.command == "preview":
        print(json.dumps({
            "calibration_file": str(calibration_path),
            "quality": calibration.record["quality"],
            "tcp_height_m": planner.height,
            "dwell_s": config.task["dwell_s"],
            "targets_base_xyz_rpy": [
                {"marker_id": target.marker_id, "pose": transform_to_pose(target.tool_pose)}
                for target in planner.targets()
            ],
        }, indent=2))
        return

    if args.cycles is not None and args.cycles < 1:
        raise ValueError("--cycles must be positive")
    log = EventLog(new_log_path(config.resolve(config.data["log_directory"])))
    try:
        with DroidRobot(config.data["robot_endpoint"]) as robot:
            if args.initialize:
                robot.initialize()
            MarkerHoverTask(config, calibration, robot, log).run(args.cycles)
    finally:
        log.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Cancelled.", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        sys.exit(1)
