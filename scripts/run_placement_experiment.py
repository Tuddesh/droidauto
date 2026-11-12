#!/usr/bin/env python3
"""Launch a one-cube placement experiment using the existing station framework.

The experiment stages the visible cube on the table, measures one placement
into a random free cell, and repeats. --execute and a start confirmation are
required. Camera and CUDA solver instances are reused across all trials.
"""

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import traceback

from auto_reset import ResetManager
from auto_reset.calibration import BufferCalibration
from auto_reset.experiments import PlacementExperiment
from auto_reset.hardware import DroidRobot, ZedWristCamera
from auto_reset.io import EventLog, StationConfig


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = ROOT / "configs/station.local.json"
    parser.add_argument("--config", type=Path,
                        default=default if default.exists() else ROOT / "configs/station.example.json")
    parser.add_argument("--settings", type=Path, default=ROOT / "configs/placement_experiment.example.json")
    parser.add_argument("--runs", type=int, help="Number of placements; default from settings: 10")
    parser.add_argument("--seed", type=int, help="Record/reproduce random staging and cell selection")
    parser.add_argument("--object", dest="object_id", help="Configured object ID; otherwise require exactly one visible cube")
    parser.add_argument("--scan-buffer", action="store_true", help="Recapture the buffer at home before the first trial")
    parser.add_argument("--review", action="store_true", help="Review each frozen scan interactively instead of running the batch")
    parser.add_argument("--initialize-controllers", action="store_true", help="Explicitly initialize DROID controllers before moving home")
    parser.add_argument("--execute", action="store_true", help="Allow experiment robot motion")
    args = parser.parse_args()
    if not args.execute:
        parser.error("Robot experiments require --execute")
    directory = None
    try:
        config = StationConfig(args.config)
        settings = json.loads(args.settings.read_text())
        runs = settings["runs"] if args.runs is None else args.runs
        if runs < 1:
            parser.error("--runs must be positive")
        config.camera["review_capture"] = args.review
        print(f"Placement experiment: {runs} trials, one cube, random free buffer cells.")
        print(f"Cube: {args.object_id or 'automatically choose the single visible cube'}")
        print(f"Table X/Y bounds (base metres): {settings['table_bounds_xy_m']}")
        print("The arm will move to DROID-home, randomize the cube on the table, and perform the trials.")
        choice = input("Enter to start, q to cancel> ").strip().lower()
        if choice:
            print("Experiment cancelled.")
            return 0
        directory = ROOT / "runs/placement_experiments" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        with ExitStack() as stack:
            log = EventLog(directory / "robot.jsonl", echo=False)
            stack.callback(log.close)
            print(f"Results: {directory}", flush=True)
            (directory / "settings.json").write_text(json.dumps(settings, indent=2) + "\n")
            calibration = None
            path = config.resolve(config.data["calibration_file"])
            if args.scan_buffer:
                print("Buffer: fresh scan at home before the first trial.", flush=True)
            elif path.exists():
                try:
                    calibration = BufferCalibration.load(path)
                    calibration.check_config(config)
                except (OSError, ValueError, KeyError) as exc:
                    print(f"Buffer calibration will be recaptured at home: {exc}", flush=True)
                    calibration = None
            robot = stack.enter_context(DroidRobot(config.data["robot_endpoint"], ik_config=config.data.get("ik")))
            if args.initialize_controllers:
                robot.initialize()
            camera = stack.enter_context(ZedWristCamera(str(config.camera["serial"]), config.camera["view"],
                                                        auto_exposure_roi=config.camera.get("auto_exposure_roi")))
            options = config.data.get("reset", {})
            manager = ResetManager(buffer=config.make_buffer(), robot=robot, camera=camera,
                                   station_config=config, calibration=calibration, objects=config.make_objects(),
                                   planner=config.make_planner(), log=log,
                                   position_tolerance_m=options.get("position_tolerance_m", 0.01),
                                   orientation_tolerance_rad=options.get("orientation_tolerance_rad", 0.1),
                                   motion_timeout_s=options.get("motion_timeout_s", 60.0))
            experiment = PlacementExperiment(manager=manager, home_joints=settings["home_joints_rad"],
                                             table_bounds_xy_m=settings["table_bounds_xy_m"],
                                             output_directory=directory, runs=runs,
                                             object_id=args.object_id, seed=args.seed)
            summary = experiment.run()
            print(f"Completed {summary['completed_trials']} trials; {summary['placement_successes']} within tolerance.")
            print(f"Mean XY error: {summary['mean_xy_error_mm']} mm\nResults: {directory}")
            return 0
    except (KeyboardInterrupt, EOFError):
        print("\nExperiment cancelled. Partial results are retained.", file=sys.stderr)
        return 130
    except Exception as exc:
        if directory is not None:
            (directory / "error.txt").write_text(traceback.format_exc())
        print(f"Experiment failed: {exc}" + (f"\nResults: {directory}" if directory else ""), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
