#!/usr/bin/env python3
"""Save a stationary stereo burst above the cube, with existing localization.

Use the latest buffer-capture experiment as reference. --move-to-scan uses the
existing planner's measuring pose; otherwise the operator positions the arm.
All source images, frozen fits, camera geometry and cube poses are saved.
"""

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import traceback

import numpy as np

from auto_reset import ResetManager, TargetConfiguration
from auto_reset.calibration import BufferCalibration
from auto_reset.hardware import DroidRobot, ZedWristCamera
from auto_reset.io import EventLog, StationConfig
from scene.transforms import transform_to_pose


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = ROOT / "configs/station.local.json"
    parser.add_argument("--config", type=Path,
                        default=default if default.exists() else ROOT / "configs/station.example.json")
    parser.add_argument("--buffer-calibration", type=Path, help="Default: latest buffer-capture experiment")
    parser.add_argument("--object", dest="object_id", help="Configured cube; otherwise choose the single visible cube")
    parser.add_argument("--pairs", type=int, default=10, help="Stereo pairs to save; default: 10")
    parser.add_argument("--move-to-scan", action="store_true", help="Move to the existing measuring pose after confirmation")
    parser.add_argument("--initialize-controllers", action="store_true", help="Initialize DROID controllers before capture or scan motion")
    args = parser.parse_args()
    if args.pairs < 1:
        parser.error("--pairs must be positive")
    directory = None
    try:
        config = StationConfig(args.config)
        config.camera["view"] = "stereo"
        candidates = sorted((ROOT / "runs/localization_experiments/buffer").glob("*/buffer.json"))
        reference = args.buffer_calibration or (candidates[-1] if candidates else None)
        if reference is None:
            parser.error("Run python scripts/capture_buffer_cube.py first, or provide --buffer-calibration")
        calibration = BufferCalibration.load(reference)
        calibration.check_config(config)
        objects = config.make_objects()
        if args.object_id is not None:
            if args.object_id not in objects:
                parser.error(f"Unknown configured cube: {args.object_id}")
            objects = {args.object_id: objects[args.object_id]}
        print(f"Buffer reference: {reference}\nStationary burst: {args.pairs} stereo pairs.")
        print("Keep the buffer and cube in their manually arranged positions.")
        print("The camera will approach the measuring pose." if args.move_to_scan
              else "Position the camera at the desired measuring pose; the script will capture there.")
        if input("Enter to begin; q to cancel> ").strip():
            return 0
        directory = ROOT / "runs/localization_experiments/closeup" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        config.data["log_directory"] = str(directory)
        config.camera["review_capture"] = True
        with ExitStack() as stack:
            log = EventLog(directory / "events.jsonl", echo=False)
            stack.callback(log.close)
            print(f"Saving experiment to: {directory}", flush=True)
            (directory / "buffer_reference.json").write_text(json.dumps(calibration.record, indent=2) + "\n")
            robot = stack.enter_context(DroidRobot(config.data["robot_endpoint"], ik_config=config.data.get("ik")))
            if args.initialize_controllers:
                robot.initialize()
            camera = stack.enter_context(ZedWristCamera(
                str(config.camera["serial"]), config.camera["view"],
                auto_exposure_roi=config.camera.get("auto_exposure_roi")))
            manager = ResetManager(buffer=config.make_buffer(), robot=robot, camera=camera,
                                   station_config=config, objects=objects, calibration=calibration,
                                   planner=config.make_planner(), log=log,
                                   motion_timeout_s=config.data.get("reset", {}).get("motion_timeout_s", 60.0))
            baseline = None
            moves = []
            if args.move_to_scan:
                manager.observe(detect_objects=True)
                baseline = dict(manager.state)["last_observation"]
                visible = [key for key, obj in objects.items() if obj.pose is not None]
                if len(visible) != 1:
                    raise ValueError(f"Expected one visible cube for the scan approach, found {visible}; use --object")
                key = visible[0]
                target = TargetConfiguration(buffer_id=manager.buffer.object_id)
                target.set_pose(key, np.linalg.inv(manager.buffer.pose) @ objects[key].pose)
                moves = [action for action in manager.plan(target, solve_ik=True, rescan_objects=True)
                         if action["type"] == "move"]
                (directory / "scan_plan.json").write_text(json.dumps(moves, indent=2) + "\n")
                pose = transform_to_pose(moves[-1]["pose"])
                print(f"Measuring flange pose: XYZ {pose[:3]} m, RPY {np.degrees(pose[3:]).round(2).tolist()} deg.")
                if input("Enter to move to this measuring pose; q to cancel> ").strip():
                    return 0
                for action in moves:
                    log.emit("scan_motion_started", **action)
                    robot.command_joints(action["joints"], blocking=True)
                    robot.wait_until_stopped(timeout_s=manager.motion_timeout_s)
                    log.emit("scan_motion_finished", name=action["name"])
                object_ids = (key,)
            else:
                object_ids = None if args.object_id is None else (args.object_id,)
            manager.observe(detect_objects=True, object_ids=object_ids, capture_count=args.pairs)
            record = {"station_config_file": str(config.path), "station_config": config.data,
                      "buffer_reference_file": str(reference), "base_buffer": manager.buffer.pose.tolist(),
                      "requested_pairs": args.pairs, "scan_moves": moves, "baseline": baseline,
                      "scene": manager.observe(), "observation": dict(manager.state)["last_observation"],
                      "buffer_relative_cubes": {key: (np.linalg.inv(manager.buffer.pose) @ obj.pose).tolist()
                                                for key, obj in objects.items() if obj.pose is not None}}
            (directory / "capture.json").write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
            print(f"Saved burst images, fit and poses: {directory}", flush=True)
            return 0 if record["buffer_relative_cubes"] else 1
    except (KeyboardInterrupt, EOFError):
        print("\nCapture cancelled; existing captures are retained.", file=sys.stderr)
        return 130
    except Exception as exc:
        if directory is not None:
            directory.mkdir(parents=True, exist_ok=True)
            (directory / "error.txt").write_text(traceback.format_exc())
        print(f"Capture failed: {exc}" + (f"\nFiles: {directory}" if directory else ""), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
