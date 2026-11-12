#!/usr/bin/env python3
"""Teach a named, buffer-relative target using the existing stationary scan tools.

Arrange the scene, capture and review frozen stereo images, then save a reset
request containing targets only. Camera and robot-state acquisition belong to
ResetManager.observe(); this script never commands motion or controllers.
"""

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shlex
import sys
import tempfile
import traceback

import numpy as np

from auto_reset import ResetManager, TargetConfiguration
from auto_reset.calibration import BufferCalibration
from auto_reset.hardware import DroidRobot, ZedWristCamera
from auto_reset.io import EventLog, StationConfig, new_log_path


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", nargs="?", help="Scene name; prompted when omitted")
    default = ROOT / "configs/station.local.json"
    parser.add_argument("--config", type=Path,
                        default=default if default.exists() else ROOT / "configs/station.example.json")
    parser.add_argument("--scenes-dir", type=Path, default=ROOT / "runs/scenes")
    parser.add_argument("--objects", nargs="+", help="Configured object IDs to teach; default: all")
    parser.add_argument("--scan-buffer", action="store_true", help="Recapture the buffer instead of reusing calibration")
    args = parser.parse_args()
    log = None
    temporary = None
    try:
        name = args.name.strip() if args.name else None
        while not name:
            name = input("Scene name (or q to cancel)> ").strip()
            if name.lower() == "q":
                return 0
        if name in (".", "..") or Path(name).name != name:
            parser.error("Scene name must be a single folder name, without a path")
        config = StationConfig(args.config)
        config.camera["review_capture"] = True
        objects = config.make_objects()
        if args.objects:
            unknown = set(args.objects) - objects.keys()
            if unknown:
                parser.error(f"Unknown configured objects: {', '.join(sorted(unknown))}")
            objects = {key: objects[key] for key in dict.fromkeys(args.objects)}
        if not objects:
            parser.error("No objects are configured for teaching")
        destination = args.scenes_dir.resolve() / name / "reset.json"
        calibration = None
        path = config.resolve(config.data["calibration_file"])
        if path.exists() and not args.scan_buffer:
            try:
                calibration = BufferCalibration.load(path)
                calibration.check_config(config)
            except (OSError, ValueError, KeyError) as exc:
                print(f"Saved buffer calibration cannot be reused: {exc}\nThe buffer will be captured again.", flush=True)
                calibration = None
        print(f"Teaching scene: {name}\nObjects: {', '.join(objects)}\nTarget: {destination}", flush=True)
        print("Targets follow the buffer's coordinate frame. Arrange the scene and position the camera.", flush=True)
        if calibration is not None:
            print("Buffer: using saved calibration; use --scan-buffer if the buffer moved.", flush=True)
        with ExitStack() as stack:
            log = EventLog(new_log_path(ROOT / "runs/teach_scene", prefix="teach"), echo=False)
            stack.callback(log.close)
            print(f"Log: {log.path}", flush=True)
            log.emit("teaching_started", name=name, object_ids=list(objects), destination=str(destination))
            robot = stack.enter_context(DroidRobot(config.data["robot_endpoint"]))
            camera = stack.enter_context(ZedWristCamera(
                str(config.camera["serial"]), config.camera["view"],
                auto_exposure_roi=config.camera.get("auto_exposure_roi"),
            ))
            manager = ResetManager(buffer=config.make_buffer(), robot=robot,
                                   planner=config.make_planner(), camera=camera,
                                   station_config=config, calibration=calibration,
                                   objects=objects, log=log)
            while True:
                choice = input("Enter to capture the stationary scene, q to cancel> ").strip().lower()
                if choice in ("q", "quit"):
                    log.emit("teaching_cancelled")
                    return 0
                if choice:
                    continue
                manager.observe(detect_objects=True)
                missing = [key for key, obj in manager.objects.items()
                           if obj.pose is None or obj.pose_frame != "base"]
                if missing:
                    print("Scene not saved. Missing objects:", flush=True)
                    for key in missing:
                        obj = manager.objects[key]
                        print(f"  {key}: {obj.marker_dictionary} / ID {obj.marker_id}", flush=True)
                    print("Adjust visibility and capture again.", flush=True)
                    continue
                target = TargetConfiguration(buffer_id=manager.buffer.object_id)
                buffer_base = np.linalg.inv(manager.buffer.pose)
                for key, obj in manager.objects.items():
                    target.set_pose(key, buffer_base @ obj.pose)
                while True:
                    choice = input(f"Save scene '{name}'? [s=save, r=retake, q=cancel]> ").strip().lower()
                    if choice in ("s", "r", "q"):
                        break
                if choice == "q":
                    log.emit("teaching_cancelled")
                    return 0
                if choice == "r":
                    continue
                if destination.exists():
                    while True:
                        choice = input(f"Scene '{name}' already exists. [r=replace, c=cancel]> ").strip().lower()
                        if choice in ("r", "c"):
                            break
                    if choice == "c":
                        log.emit("teaching_cancelled")
                        return 0
                request = {
                    "operation": "reset",
                    "targets": {key: {"pose": pose.tolist()} for key, pose in target.poses.items()},
                    "scene": {
                        "name": name,
                        "created_at": datetime.now(timezone.utc).isoformat(),
                        "reference_frame": "buffer",
                        "buffer_id": target.buffer_id,
                        "station_config": str(config.path),
                        "buffer_calibration": manager.calibration.record,
                        "capture": dict(manager.state)["last_observation"],
                    },
                }
                payload = json.dumps(request, indent=2, allow_nan=False) + "\n"
                destination.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=destination.parent,
                                                 delete=False) as output:
                    temporary = Path(output.name)
                    output.write(payload)
                os.replace(temporary, destination)
                temporary = None
                log.emit("scene_saved", name=name, target_file=str(destination), object_ids=list(target.poses))
                print(f"Saved: {destination}\nReset to this scene with:", flush=True)
                print(shlex.join([sys.executable, "-m", "auto_reset", "--config", str(config.path),
                                  "--request", str(destination), "--execute"]), flush=True)
                return 0
    except (KeyboardInterrupt, EOFError):
        print("\nTeaching cancelled; no target saved.", flush=True)
        return 130
    except Exception as exc:
        cancelled = isinstance(exc, RuntimeError) and str(exc) in (
            "Cube scan cancelled by operator", "Buffer scan cancelled by operator")
        if log is not None:
            # ExitStack has closed the log if acquisition failed; retain the traceback.
            with log.path.open("a", encoding="utf-8") as output:
                output.write(json.dumps({"time_s": datetime.now(timezone.utc).timestamp(),
                                         "event": "teaching_cancelled" if cancelled else "teaching_failed", "reason": str(exc),
                                         "traceback": traceback.format_exc()}) + "\n")
        if cancelled:
            print("Teaching cancelled; no target saved.", flush=True)
            return 0
        print(f"Teaching failed: {exc}" + (f"\nLog: {log.path}" if log is not None else ""),
              file=sys.stderr, flush=True)
        return 1
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
