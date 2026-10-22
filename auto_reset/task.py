"""Marker-hover task composed from reusable buffer, planner and executor modules."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

from .motion import CartesianExecutor, HoverPlanner
from .transforms import transform_to_pose


class EventLog:
    """Append complete events; preserve failed attempts as well as completed visits."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("x")

    def emit(self, event, **fields):
        record = {"time_s": time.time(), "event": event, **fields}
        self._file.write(json.dumps(record, allow_nan=False) + "\n")
        self._file.flush()
        print(event, json.dumps(fields), flush=True)

    def close(self):
        self._file.close()


class MarkerHoverTask:
    """Use one frozen calibration for all visits; never acquire images in the loop."""

    def __init__(self, config, calibration, robot, log):
        self.config, self.calibration, self.robot, self.log = config, calibration, robot, log
        self.planner = HoverPlanner(config, calibration)
        self.executor = CartesianExecutor(robot, config.motion)

    def run(self, cycles=None):
        if cycles is not None and (isinstance(cycles, bool) or not isinstance(cycles, int) or cycles < 1):
            raise ValueError("cycles must be a positive integer or None for continuous operation")
        calibration_hash = hashlib.sha256(json.dumps(self.calibration.record, sort_keys=True).encode()).hexdigest()
        self.log.emit("task_started", calibration_digest=calibration_hash,
                      calibration=self.calibration.record, configuration=self.config.data,
                      cycles=cycles)
        cycle = 0
        try:
            self.robot.require_stationary()
            targets = self.planner.targets()
            while cycles is None or cycle < cycles:
                for target in targets:
                    self.log.emit("visit_started", cycle=cycle + 1, marker_id=target.marker_id,
                                  target_pose=transform_to_pose(target.tool_pose))
                    state = self.robot.read_state()
                    for waypoint in self.planner.route(state.tool_pose, target.tool_pose):
                        self.executor.move_to(waypoint)
                    self.log.emit("hover_arrived", cycle=cycle + 1, marker_id=target.marker_id)
                    self.executor.hold(target.tool_pose, self.config.task["dwell_s"])
                    self.log.emit("visit_completed", cycle=cycle + 1, marker_id=target.marker_id,
                                  measured_pose=transform_to_pose(self.robot.read_state().tool_pose))
                cycle += 1
                self.log.emit("cycle_completed", cycle=cycle)
            self.log.emit("task_completed", cycles=cycle)
        except KeyboardInterrupt:
            self.log.emit("task_cancelled", completed_cycles=cycle)
            try:
                self.executor.hold_current()
            except Exception as exc:
                self.log.emit("cancel_hold_failed", error=str(exc))
            raise
        except Exception as exc:
            # No automatic recovery/relaunch or retry of uncertain motion.
            self.log.emit("task_failed", completed_cycles=cycle, error=str(exc))
            raise


def new_log_path(directory):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return Path(directory) / f"marker_hover_{stamp}.jsonl"
