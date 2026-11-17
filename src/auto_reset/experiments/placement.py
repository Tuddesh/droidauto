"""Fast first-attempt cube placement measurements using the station primitives.

One home scan discovers the scene. Each trial solves and executes one transfer,
returns to the saved home joints, then scans all cubes once. That measurement
also supplies the next pickup and obstacle map. No staging or corrective moves.
Grid clearance checks do not implement trajectory collision avoidance.
"""

from dataclasses import dataclass
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
            if key == object_id or obj.pose is None:
                continue
            pose = local_base @ obj.pose
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

    def run(self) -> dict:
        manager = self.manager
        if isinstance(self.runs, bool) or not isinstance(self.runs, int) or self.runs < 1:
            raise ValueError("Experiment runs must be a positive integer")
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
        rows, status, failure, stop_reason = [], "running", None, None
        csv_file = None
        # Share the manager's cancellation and execution lock; never clear a pending cancellation.
        if not manager._run_lock.acquire(blocking=False):
            events.close()
            raise RuntimeError("A reset is already executing")
        try:
            events.emit("experiment_started", runs=self.runs, seed=seed, home_joints=home.tolist(),
                        target_cells=self.target_cells, clearance_cells=1, mode="direct_placement")
            self._move_home(home, events, trial=0, before="first observation")
            manager.observe(detect_objects=True, retry_missing=False)
            inventory = {key for key, obj in manager.objects.items() if obj.pose is not None}
            if not inventory or self.object_id is not None and self.object_id not in inventory:
                raise ValueError("No selected cube visible from home")
            print(f"Detected {len(inventory)} cubes: {', '.join(sorted(inventory))}", flush=True)
            if cells is not None:
                for column, row in cells:
                    TargetConfiguration(buffer_id=manager.buffer.object_id).set_cell(
                        next(iter(inventory)), manager.buffer, column, row,
                        scene_object=manager.objects[next(iter(inventory))], support_height_m=0.0)
            fields = ["trial", "object_id", "marker_id", "column", "row", "target_yaw_deg",
                      "observed", "success", "dx_mm", "dy_mm", "xy_error_mm", "yaw_error_deg",
                      "reset_success", "manipulations", "duration_s"]
            csv_file = (directory / "trials.csv").open("x", newline="", encoding="utf-8")
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            csv_file.flush()
            setup = {"runs": self.runs, "seed": seed, "home_joints": home.tolist(),
                     "clearance_cells": 1, "target_cells": self.target_cells,
                     "station_config": manager.station_config.data,
                     "visible_objects": {key: {"marker_id": manager.objects[key].marker_id,
                                                "dictionary": manager.objects[key].marker_dictionary,
                                                "base_pose": manager.objects[key].pose.tolist()}
                                         for key in sorted(inventory)}}
            (directory / "setup.json").write_text(json.dumps(setup, indent=2) + "\n")
            last_used = {}
            for trial in range(1, self.runs + 1):
                if manager._cancel_requested.is_set():
                    raise ResetCancelled("Experiment cancelled before placement")
                # Every scene scan refreshes ALL identities, including obstacles.
                visible = {key for key, obj in manager.objects.items() if obj.pose is not None}
                missing = inventory - visible
                if missing:
                    raise RuntimeError(f"Previously visible cubes missing; stopping before further motion: {sorted(missing)}")
                inventory |= visible
                candidates = [key for key in sorted(visible) if self.object_id is None or key == self.object_id]
                rng.shuffle(candidates)
                candidates.sort(key=lambda key: last_used.get(key, 0))
                selected = None
                for object_id in candidates:
                    cube = manager.objects[object_id]
                    free = self._free_cells(object_id)
                    if cells is not None:
                        free = [cells[(trial-1) % len(cells)]] if cells[(trial-1) % len(cells)] in free else []
                    else:
                        rng.shuffle(free)
                    for column, row in free:
                        target = TargetConfiguration(buffer_id=manager.buffer.object_id)
                        target.set_cell(object_id, manager.buffer, column, row,
                                        scene_object=cube, support_height_m=0.0)
                        desired = target.pose_for(object_id)
                        # Require a real transfer, even when revisiting an already placed cube.
                        actual = np.linalg.inv(manager.buffer.pose) @ cube.pose
                        if np.linalg.norm(actual[:2, 3] - desired[:2, 3]) < manager.buffer.cell_size_mm / 2000:
                            continue
                        try:
                            actions = manager.plan(target, solve_ik=True, rescan_objects=False)
                        except RuntimeError as exc:
                            if "no feasible IK solution" not in str(exc):
                                raise
                            events.emit("cell_unreachable", trial=trial, object_id=object_id, column=column, row=row)
                            continue
                        selected = (object_id, cube, column, row, target, actions)
                        break
                    if selected is not None:
                        break
                if selected is None:
                    stop_reason = "No reachable target with one-cell clearance" if cells is None else "Requested target blocked or unreachable"
                    events.emit("experiment_capacity_reached", trial=trial, reason=stop_reason)
                    print(stop_reason + "; retaining completed measurements.", flush=True)
                    break
                object_id, cube, column, row, target, actions = selected
                print(f"Trial {trial}/{self.runs}: {object_id} -> ({column}, {row}); place once, measure.", flush=True)
                started = time.monotonic()
                # Execute the already solved transfer using the manager's existing
                # command/logging primitive. Avoid the feedback loop's replanning,
                # close-up scan, retries and duplicate verification captures.
                manager.target_configuration = target
                manager._state.update(phase="executing", success=None, manipulations=0,
                                      scene_before=manager.observe(), timing={"started_s": time.time()})
                events.emit("transfer_started", trial=trial, object_id=object_id, actions=actions)
                manager._execute(actions)
                manager._state["manipulations"] = 1
                self._move_home(home, events, trial=trial, before="measurement")
                manager._state["phase"] = "observing"
                manager.observe(detect_objects=True, retry_missing=False)
                manager._verify(target)
                duration = time.monotonic() - started
                observed = cube.pose is not None
                dx = dy = distance = angle = None
                success = False
                if observed:
                    actual = np.linalg.inv(manager.buffer.pose) @ cube.pose
                    delta = actual[:2, 3] - desired[:2, 3]
                    dx, dy = (delta * 1000).tolist()
                    distance = float(np.linalg.norm(delta) * 1000)
                    phi = transform_to_pose(actual)[5] - transform_to_pose(desired)[5]
                    angle = float(np.degrees(np.arctan2(np.sin(phi), np.cos(phi))))
                    position_m, orientation_rad = pose_error(cube.pose, manager.buffer.pose @ desired)
                    success = (position_m <= manager.position_tolerance_m
                               and orientation_rad <= manager.orientation_tolerance_rad)
                result = {"trial": trial, "object_id": object_id, "marker_id": cube.marker_id,
                          "column": column, "row": row, "target_yaw_deg": 0.0,
                          "observed": observed, "success": bool(success),
                          "dx_mm": dx, "dy_mm": dy, "xy_error_mm": distance, "yaw_error_deg": angle,
                          "reset_success": bool(manager.state["success"]),
                          "manipulations": 1, "duration_s": duration}
                writer.writerow(result)
                csv_file.flush()
                rows.append(result)
                last_used[object_id] = trial
                events.emit("trial_measured", **result, base_buffer=manager.buffer.pose.tolist(),
                            target_buffer_pose=desired.tolist(), scene=manager.observe())
                measured = "cube not detected" if distance is None else f"XY error {distance:.2f} mm, yaw error {angle:.2f} deg"
                print(f"Trial {trial}/{self.runs}: {measured}; success={success}", flush=True)
                missing = inventory - {key for key, obj in manager.objects.items() if obj.pose is not None}
                if missing:
                    raise RuntimeError(f"Cubes missing after placement; recorded trial and stopped: {sorted(missing)}")
            status = "completed" if len(rows) == self.runs else "capacity_reached"
        except (Exception, KeyboardInterrupt) as exc:
            status = "cancelled" if isinstance(exc, (ResetCancelled, KeyboardInterrupt)) else "failed"
            failure = str(exc) or type(exc).__name__
            manager._state.update(phase=status, success=False, failure_reason=failure)
            events.emit("experiment_stopped", status=status, reason=failure, traceback=traceback.format_exc())
            raise
        finally:
            errors = [r["xy_error_mm"] for r in rows if r["xy_error_mm"] is not None]
            summary = {"status": status, "requested_trials": self.runs, "completed_trials": len(rows),
                       "placement_successes": sum(r["success"] for r in rows), "seed": seed,
                       "mean_xy_error_mm": None if not errors else float(np.mean(errors)),
                       "max_xy_error_mm": None if not errors else float(np.max(errors)),
                       "failure_reason": failure, "stop_reason": stop_reason}
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
