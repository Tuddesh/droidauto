"""First-attempt cube placement measurements using the station primitives.

Scene observations fit multiple captures. Each trial uses the established
above-cube scan, replans from its refined pickup pose, transfers once, and
measures from the saved home joints. Failed checks are retained per trial;
other trials continue. No table staging or corrective transfers.
Grid clearance checks do not implement trajectory collision avoidance.
"""

from dataclasses import dataclass, field
import csv
import json
from pathlib import Path
import secrets
import time
import traceback

import cv2
import numpy as np

from auto_reset.io import EventLog
from auto_reset.manager import ResetManager, ResetCancelled
from auto_reset.target_configuration import TargetConfiguration
from scene.transforms import pose_error, transform_to_pose


@dataclass
class PlacementExperiment:
    manager: ResetManager
    home_joints: list[float]
    table_bounds_xy_m: list[float]  # Retained for compatibility with existing settings.
    output_directory: Path
    runs: int = 10
    object_id: str | None = None
    seed: int | None = None
    target_cells: list[list[int]] | None = None

    capture_count: int = 5
    observation_attempts: int = 3
    _obstacles: dict = field(default_factory=dict, init=False, repr=False)
    _reservations: dict = field(default_factory=dict, init=False, repr=False)

    def _free_cells(self, object_id):
        """Keep a complete empty cell between target and every other footprint.

        Test actual rotated footprints, including cubes initially on the grid
        and cubes whose last placement missed its target. The moving cube's
        source is excluded because it will be removed before placing.
        """
        manager = self.manager
        grid = manager.buffer
        size = grid.cell_size_mm / 1000
        local_base = np.linalg.inv(grid.pose)
        obstacles = []
        for key, obj in manager.objects.items():
            if key == object_id:
                continue
            poses = [obj.pose] if obj.pose is not None else [self._obstacles.get(key), self._reservations.get(key)]
            for base_pose in poses:
                if base_pose is None:
                    continue
                pose = local_base @ base_pose
                hx, hy = np.asarray(obj.dimensions_m[:2]) / 2
                corners = np.array([[-hx, -hy, 0], [hx, -hy, 0], [hx, hy, 0], [-hx, hy, 0]])
                obstacles.append((corners @ pose[:3, :3].T + pose[:3, 3])[:, :2].astype(np.float32))
        free = []
        for column in range(grid.columns):
            for row in range(grid.rows):
                if grid.item_at(column, row) is not None:
                    continue
                center = grid.cell_center_m(column, row)[:2]
                # A cell plus a one-cell halo in all eight directions.
                half = 1.5 * size
                halo = (center + np.array([[-half, -half], [half, -half],
                                           [half, half], [-half, half]])).astype(np.float32)
                if any(cv2.intersectConvexConvex(halo, footprint)[0] > 1e-10 for footprint in obstacles):
                    continue
                # Keep the same halo around installed grid fixtures.
                if any(grid.item_at(c, r) is not None
                       for c in range(max(0, column-1), min(grid.columns, column+2))
                       for r in range(max(0, row-1), min(grid.rows, row+2))):
                    continue
                free.append((column, row))
        return free

    def _scan_scene(self, events, trial):
        """Fit bursts and retry incomplete scans, merging only fresh detections."""
        manager = self.manager
        manager._state["phase"] = "observing"
        expected = set(self._obstacles)
        fresh = {}
        for attempt in range(1, self.observation_attempts + 1):
            if manager._cancel_requested.is_set():
                raise ResetCancelled("Experiment cancelled during scene scan")
            print(f"Scene scan {attempt}/{self.observation_attempts}: {self.capture_count} captures", flush=True)
            # A failed scan cannot leave previous-trial poses eligible for pickup.
            for obj in manager.objects.values():
                obj.mark_unobserved()
            try:
                manager.observe(detect_objects=True, retry_missing=False, capture_count=self.capture_count)
            except ResetCancelled:
                raise
            except Exception as exc:
                events.emit("scene_scan_failed", trial=trial, attempt=attempt, reason=str(exc))
                print(f"Scene scan failed: {exc}; retrying/continuing.", flush=True)
                continue
            for key, obj in manager.objects.items():
                if obj.pose is not None:
                    fresh[key] = (obj.pose.copy(), obj.observed_at)
            if fresh and expected <= fresh.keys():
                break
        for obj in manager.objects.values():
            obj.mark_unobserved()
        for key, (pose, stamp) in fresh.items():
            manager.objects[key].update_pose(pose, "base", stamp)
            self._obstacles[key] = pose.copy()
            self._reservations.pop(key, None)
        missing = expected - fresh.keys()
        if missing:
            events.emit("scene_objects_missing", trial=trial, object_ids=sorted(missing))
            print(f"{len(missing)} previously seen cubes missing; skipping those pickups.", flush=True)
        return set(fresh)

    def _recover_home(self, home, events, trial, *, transfer_failed=False):
        """Recover after an individual action failure only with usable feedback."""
        manager = self.manager
        if manager._cancel_requested.is_set():
            raise ResetCancelled("Experiment cancelled before recovery")
        # Never resubmit a possibly pending motion RPC. Establish stationary,
        # fresh feedback before returning home to work on another cube.
        manager.robot.require_stationary()
        if transfer_failed and manager.robot.read_gripper_position() > 0.1:
            raise RuntimeError("Transfer failed with the gripper closed; cannot establish an empty gripper for the next cube")
        self._move_home(home, events, trial=trial, before="recovery")
        manager._state["phase"] = "observing"

    def run(self) -> dict:
        manager = self.manager
        for name in ("runs", "capture_count", "observation_attempts"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        home = np.asarray(self.home_joints, dtype=float)
        if home.shape != (7,) or not np.isfinite(home).all():
            raise ValueError("Home needs seven finite joint angles")
        if manager.robot is None or manager.camera is None or manager.station_config is None:
            raise ValueError("Experiment needs a connected robot, open camera and station configuration")
        if manager.station_config.data.get("planar_scene") is None:
            raise ValueError("Placement experiments require the planar scene model")
        if self.seed is not None and (isinstance(self.seed, bool) or not isinstance(self.seed, int) or self.seed < 0):
            raise ValueError("Random seed must be a nonnegative integer")
        cells = None if self.target_cells is None else [tuple(cell) for cell in self.target_cells]
        if cells is not None and (not cells or any(
                len(cell) != 2 or any(isinstance(v, bool) or not isinstance(v, int) for v in cell) for cell in cells)):
            raise ValueError("Target cells must be a non-empty list of [column, row] integer pairs")
        if self.object_id is not None and self.object_id not in manager.objects:
            raise ValueError(f"Unknown configured cube: {self.object_id}")
        seed = secrets.randbits(64) if self.seed is None else self.seed
        rng = np.random.default_rng(seed)
        directory = Path(self.output_directory)
        directory.mkdir(parents=True, exist_ok=True)
        events = EventLog(directory / "trials.jsonl", echo=False)
        rows, status, failure = [], "running", None
        csv_file = None
        if not manager._run_lock.acquire(blocking=False):
            events.close()
            raise RuntimeError("A reset is already executing")
        try:
            events.emit("experiment_started", runs=self.runs, seed=seed, home_joints=home.tolist(),
                        target_cells=self.target_cells, clearance_cells=1, capture_count=self.capture_count,
                        observation_attempts=self.observation_attempts, mode="direct_placement_with_closeup")
            self._move_home(home, events, trial=0, before="first observation")
            self._obstacles.clear()
            self._reservations.clear()
            visible = self._scan_scene(events, trial=0)
            print(f"Detected {len(visible)} cubes: {', '.join(sorted(visible))}", flush=True)
            if cells is not None:
                sample = manager.objects[self.object_id] if self.object_id else next(iter(manager.objects.values()))
                for column, row in cells:
                    TargetConfiguration(buffer_id=manager.buffer.object_id).set_cell(
                        sample.object_id, manager.buffer, column, row, scene_object=sample, support_height_m=0.0)
            fields = ["trial", "object_id", "marker_id", "column", "row", "target_yaw_deg",
                      "observed", "success", "dx_mm", "dy_mm", "xy_error_mm", "yaw_error_deg",
                      "reset_success", "manipulations", "duration_s", "failure_stage", "failure_reason"]
            csv_file = (directory / "trials.csv").open("x", newline="", encoding="utf-8")
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            csv_file.flush()
            setup = {"runs": self.runs, "seed": seed, "home_joints": home.tolist(),
                     "capture_count": self.capture_count, "observation_attempts": self.observation_attempts,
                     "clearance_cells": 1, "target_cells": self.target_cells,
                     "station_config": manager.station_config.data,
                     "visible_objects": {key: {"marker_id": manager.objects[key].marker_id,
                                                "dictionary": manager.objects[key].marker_dictionary,
                                                "base_pose": manager.objects[key].pose.tolist()}
                                         for key in sorted(visible)}}
            (directory / "setup.json").write_text(json.dumps(setup, indent=2) + "\n")
            last_used = {}
            for trial in range(1, self.runs + 1):
                if manager._cancel_requested.is_set():
                    raise ResetCancelled("Experiment cancelled before placement")
                started = time.monotonic()
                result = dict.fromkeys(fields)
                result.update(trial=trial, target_yaw_deg=0.0, observed=False, success=False,
                              reset_success=False, manipulations=0)
                target, object_id, stage = None, None, "selection"
                fatal = None
                try:
                    if not visible:
                        visible = self._scan_scene(events, trial)
                    candidates = [key for key in sorted(visible) if self.object_id is None or key == self.object_id]
                    rng.shuffle(candidates)
                    candidates.sort(key=lambda key: last_used.get(key, 0))
                    selected = None
                    for key in candidates:
                        cube = manager.objects[key]
                        free = self._free_cells(key)
                        if cells is not None:
                            free = [cells[(trial-1) % len(cells)]] if cells[(trial-1) % len(cells)] in free else []
                        else:
                            rng.shuffle(free)
                        for column, row in free:
                            candidate = TargetConfiguration(buffer_id=manager.buffer.object_id)
                            candidate.set_cell(key, manager.buffer, column, row, scene_object=cube, support_height_m=0.0)
                            desired = candidate.pose_for(key)
                            actual = np.linalg.inv(manager.buffer.pose) @ cube.pose
                            if np.linalg.norm(actual[:2, 3] - desired[:2, 3]) < manager.buffer.cell_size_mm / 2000:
                                continue
                            try:
                                # Existing above-cube scan, using the home observation's camera mount.
                                scan = manager.plan(candidate, solve_ik=True, rescan_objects=True)
                            except ResetCancelled:
                                raise
                            except Exception as exc:
                                events.emit("candidate_failed", trial=trial, object_id=key,
                                            column=column, row=row, reason=str(exc))
                                # Scan reachability does not depend on the destination.
                                break
                            selected = (key, cube, column, row, candidate, scan)
                            break
                        if selected is not None:
                            break
                    if selected is None:
                        raise ValueError("No visible cube with a clear target and reachable scan; skipping trial")
                    object_id, cube, column, row, target, scan = selected
                    last_used[object_id] = trial
                    result.update(object_id=object_id, marker_id=cube.marker_id, column=column, row=row)
                    manager.target_configuration = target
                    manager._state.update(success=None, manipulations=0, scene_before=manager.observe(),
                                          timing={"started_s": time.time()})
                    stage = "pickup_scan"
                    print(f"Trial {trial}/{self.runs}: scan above {object_id}, then place in ({column}, {row}).", flush=True)
                    for action in scan:
                        if action["type"] == "rescan_object":
                            action["capture_count"] = self.capture_count
                    manager._state["phase"] = "executing"
                    events.emit("pickup_scan_started", trial=trial, object_id=object_id, actions=scan)
                    manager._execute(scan)
                    if cube.pose is not None:
                        self._obstacles[object_id] = cube.pose.copy()
                    # Solve the transfer AFTER the close-up has refined the pickup.
                    stage = "transfer_planning"
                    manager._state["phase"] = "planning"
                    actions = manager.plan(target, solve_ik=True, rescan_objects=False)
                    stage = "transfer"
                    self._reservations[object_id] = manager.buffer.pose @ target.pose_for(object_id)
                    manager._state["phase"] = "executing"
                    events.emit("transfer_started", trial=trial, object_id=object_id, actions=actions)
                    manager._execute(actions)
                    result["manipulations"] = 1
                    stage = "return_home"
                    self._move_home(home, events, trial=trial, before="measurement")
                    stage = "measurement"
                    visible = self._scan_scene(events, trial)
                    manager._verify(target)
                    result["reset_success"] = bool(manager.state["success"])
                    if cube.pose is None:
                        raise ValueError("Placed cube not detected after scan retries")
                    result["observed"] = True
                    actual = np.linalg.inv(manager.buffer.pose) @ cube.pose
                    desired = target.pose_for(object_id)
                    delta = actual[:2, 3] - desired[:2, 3]
                    result["dx_mm"], result["dy_mm"] = (delta * 1000).tolist()
                    result["xy_error_mm"] = float(np.linalg.norm(delta) * 1000)
                    phi = transform_to_pose(actual)[5] - transform_to_pose(desired)[5]
                    result["yaw_error_deg"] = float(np.degrees(np.arctan2(np.sin(phi), np.cos(phi))))
                    position_m, orientation_rad = pose_error(cube.pose, manager.buffer.pose @ desired)
                    result["success"] = bool(position_m <= manager.position_tolerance_m
                                             and orientation_rad <= manager.orientation_tolerance_rad)
                    if not result["success"]:
                        result.update(failure_stage="measurement", failure_reason="Placement outside tolerance")
                except ResetCancelled:
                    raise
                except Exception as exc:
                    result.update(failure_stage=stage, failure_reason=str(exc))
                    events.emit("trial_failed", trial=trial, object_id=object_id, stage=stage,
                                reason=str(exc), traceback=traceback.format_exc())
                    print(f"Trial {trial}/{self.runs}: {stage} failed: {exc}; continuing with other cubes.", flush=True)
                    try:
                        if stage in ("pickup_scan", "transfer_planning", "transfer", "return_home"):
                            self._recover_home(home, events, trial, transfer_failed=stage == "transfer")
                        if stage != "measurement":
                            visible = self._scan_scene(events, trial)
                    except ResetCancelled:
                        raise
                    except Exception as recovery_error:
                        fatal = RuntimeError(f"Robot recovery failed after trial {trial}: {recovery_error}")
                finally:
                    result["duration_s"] = time.monotonic() - started
                    writer.writerow(result)
                    csv_file.flush()
                    rows.append(result)
                    events.emit("trial_measured", **result, base_buffer=manager.buffer.pose.tolist(),
                                target_buffer_pose=None if target is None else target.pose_for(object_id).tolist(),
                                scene=manager.observe())
                if fatal is not None:
                    raise fatal
                measured = "no placement measurement" if result["xy_error_mm"] is None else f"XY error {result['xy_error_mm']:.2f} mm"
                print(f"Trial {trial}/{self.runs}: {measured}; success={result['success']}", flush=True)
            status = "completed"
        except (Exception, KeyboardInterrupt) as exc:
            status = "cancelled" if isinstance(exc, (ResetCancelled, KeyboardInterrupt)) else "failed"
            failure = str(exc) or type(exc).__name__
            manager._state.update(phase=status, success=False, failure_reason=failure)
            events.emit("experiment_stopped", status=status, reason=failure, traceback=traceback.format_exc())
            raise
        finally:
            errors = [r["xy_error_mm"] for r in rows if r["xy_error_mm"] is not None]
            summary = {"status": status, "requested_trials": self.runs, "completed_trials": len(rows),
                       "placement_successes": sum(r["success"] for r in rows),
                       "failed_trials": sum(not r["success"] for r in rows), "seed": seed,
                       "mean_xy_error_mm": None if not errors else float(np.mean(errors)),
                       "max_xy_error_mm": None if not errors else float(np.max(errors)),
                       "failure_reason": failure, "stop_reason": None}
            try:
                (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            finally:
                try:
                    if csv_file is not None:
                        csv_file.close()
                    events.close()
                finally:
                    manager._run_lock.release()
        return summary

    def _move_home(self, home: np.ndarray, events: EventLog, **context) -> np.ndarray:
        manager = self.manager
        if manager._cancel_requested.is_set():
            raise ResetCancelled("Experiment cancelled before moving home")
        manager.robot.command_joints(home.tolist(), blocking=True)
        manager.robot.wait_until_stopped(timeout_s=manager.motion_timeout_s)
        reached = manager.robot.read_state()
        events.emit("home_reached", commanded_joints=home.tolist(), joints=reached.joints.tolist(),
                    tool_pose=reached.tool_pose.tolist(), **context)
        return reached.tool_pose.copy()
