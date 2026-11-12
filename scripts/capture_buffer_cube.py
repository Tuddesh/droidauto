#!/usr/bin/env python3
"""Capture a manually arranged buffer/cube scene using existing station methods.

Save the exact stereo inputs, frozen fits, buffer calibration and cube poses
under runs/localization_experiments/buffer/. This experiment commands no motion.
"""

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import traceback

import numpy as np

from auto_reset import ResetManager
from auto_reset.hardware import DroidRobot, ZedWristCamera
from auto_reset.io import EventLog, StationConfig
from scene.transforms import transform_to_pose


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = ROOT / "configs/station.local.json"
    parser.add_argument("--config", type=Path,
                        default=default if default.exists() else ROOT / "configs/station.example.json")
    parser.add_argument("--object", dest="object_id", help="Limit capture to one configured cube")
    args = parser.parse_args()
    directory = None
    try:
        config = StationConfig(args.config)
        config.camera["view"] = "stereo"
        objects = config.make_objects()
        if args.object_id is not None:
            if args.object_id not in objects:
                parser.error(f"Unknown configured cube: {args.object_id}")
            objects = {args.object_id: objects[args.object_id]}
        print("Place the cube in the buffer and position the camera so the buffer is visible.")
        if input("Enter to capture; q to cancel> ").strip():
            return 0
        directory = ROOT / "runs/localization_experiments/buffer" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        config.data["log_directory"] = str(directory)
        config.data["calibration_file"] = str(directory / "buffer.json")
        config.camera["review_capture"] = True
        with ExitStack() as stack:
            log = EventLog(directory / "events.jsonl", echo=False)
            stack.callback(log.close)
            print(f"Saving experiment to: {directory}", flush=True)
            robot = stack.enter_context(DroidRobot(config.data["robot_endpoint"]))
            camera = stack.enter_context(ZedWristCamera(
                str(config.camera["serial"]), config.camera["view"],
                auto_exposure_roi=config.camera.get("auto_exposure_roi")))
            manager = ResetManager(buffer=config.make_buffer(), robot=robot, camera=camera,
                                   station_config=config, objects=objects,
                                   planner=config.make_planner(), log=log,
                                   motion_timeout_s=config.data.get("reset", {}).get("motion_timeout_s", 60.0))
            manager.observe(detect_buffer=True, detect_objects=True)
            relative = {key: (np.linalg.inv(manager.buffer.pose) @ obj.pose).tolist()
                        for key, obj in objects.items() if obj.pose is not None}
            record = {"station_config_file": str(config.path), "station_config": config.data,
                      "base_buffer": manager.buffer.pose.tolist(), "scene": manager.observe(),
                      "buffer_relative_cubes": relative,
                      "buffer_calibration": manager.calibration.record,
                      "observation": dict(manager.state)["last_observation"]}
            (directory / "capture.json").write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
            for key, pose in relative.items():
                values = transform_to_pose(pose)
                print(f"{key}: buffer XYZ {np.round(np.array(values[:3]) * 1000, 2).tolist()} mm, "
                      f"yaw {np.degrees(values[5]):.2f} deg", flush=True)
            print(f"Saved images, fits and poses: {directory}", flush=True)
            if not relative:
                print("Buffer saved, but no configured cube was detected. Inspect the saved images.", file=sys.stderr)
                return 1
            return 0
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
