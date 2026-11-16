"""One-cube first-attempt placement measurements with randomized table staging.

The reset manager owns observations and transfers. This runner selects free
buffer cells (or follows a fixed cell list), checks staging footprints/visibility/IK,
and persists every trial. Before each placement and each measurement it commands
the saved home joints, so stopping errors cannot accumulate into a drifting
observation pose, and every placement is measured from the same home view.
These geometric checks do not implement trajectory collision avoidance.
"""

from dataclasses import dataclass
from copy import deepcopy
import csv
import json
from pathlib import Path
import secrets
import time
import traceback

import cv2
import numpy as np

from auto_reset.io import EventLog
from auto_reset.hardware import DroidRobot
from auto_reset.manager import ResetManager, ResetCancelled
from auto_reset.target_configuration import TargetConfiguration
from scene.transforms import pose_error, pose_to_transform, transform_to_pose


@dataclass
class PlacementExperiment:
    manager: ResetManager
    home_joints: list[float]
    table_bounds_xy_m: list[float]
    output_directory: Path
    runs: int = 10
    object_id: str | None = None
    seed: int | None = None
    target_cells: list[list[int]] | None = None

    def run(self) -> dict:
        """Attempt each random staging and buffer placement once, then measure from home."""
        manager = self.manager
        if isinstance(self.runs, bool) or not isinstance(self.runs, int) or self.runs < 1:
            raise ValueError("Experiment runs must be a positive integer")
        home = np.asarray(self.home_joints, dtype=float)
        bounds = np.asarray(self.table_bounds_xy_m, dtype=float)
        if home.shape != (7,) or not np.isfinite(home).all():
            raise ValueError("Home needs seven finite joint angles")
        if bounds.shape != (4,) or not np.isfinite(bounds).all() or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
            raise ValueError("Table bounds must be [xmin, xmax, ymin, ymax] in base metres")
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
        seed = secrets.randbits(64) if self.seed is None else self.seed
        rng = np.random.default_rng(seed)
        directory = Path(self.output_directory)
        directory.mkdir(parents=True, exist_ok=True)
        events = EventLog(directory / "trials.jsonl", echo=False)
        rows, status, failure = [], "running", None
        csv_file = None
        try:
            events.emit("experiment_started", runs=self.runs, seed=seed, table_bounds_xy_m=bounds.tolist(),
                        home_joints=home.tolist(), planar_scene=manager.station_config.data["planar_scene"],
                        target_cells=self.target_cells)
            home_pose = self._move_home(home, events, trial=0, before="first observation")
            if self.object_id is not None and self.object_id not in manager.objects:
                raise ValueError(f"Unknown configured cube: {self.object_id}")
            manager.observe(detect_objects=True,
                            object_ids=None if self.object_id is None else (self.object_id,))
            visible = [key for key, obj in manager.objects.items() if obj.pose is not None
                       and (self.object_id is None or key == self.object_id)]
            if len(visible) != 1:
                raise ValueError(f"Expected one visible cube, found {visible}; select one with --object")
            object_id = visible[0]
            cube = manager.objects[object_id]
            for column, row in cells or ():
                TargetConfiguration(buffer_id=manager.buffer.object_id).set_cell(
                    object_id, manager.buffer, column, row, scene_object=cube, support_height_m=0.0)
            h = np.asarray(cube.dimensions_m) / 2
            top = np.array([[-h[0], -h[1], h[2]], [h[0], -h[1], h[2]],
                            [h[0], h[1], h[2]], [-h[0], h[1], h[2]]])
            fields = ["trial", "object_id", "marker_id", "column", "row", "target_yaw_deg",
                      "observed", "success", "dx_mm", "dy_mm", "xy_error_mm", "yaw_error_deg",
                      "reset_success", "manipulations", "duration_s"]
            csv_file = (directory / "trials.csv").open("x", newline="", encoding="utf-8")
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            csv_file.flush()
            with (directory / "setup.json").open("x", encoding="utf-8") as output:
                json.dump({"runs": self.runs, "seed": seed, "object_id": object_id,
                           "marker_id": cube.marker_id, "home_joints": home.tolist(),
                           "home_tool_pose": home_pose.tolist(), "table_bounds_xy_m": bounds.tolist(),
                           "cube_dimensions_m": cube.dimensions_m,
                           "object_tool_grasp": cube.grasp_poses()[0].tolist(),
                           "planar_scene": manager.station_config.data["planar_scene"],
                           "position_tolerance_m": manager.position_tolerance_m,
                           "orientation_tolerance_rad": manager.orientation_tolerance_rad,
                           "target_cells": self.target_cells,
                           "station_config": str(manager.station_config.path)}, output, indent=2)
            for trial in range(1, self.runs + 1):
                if manager._cancel_requested.is_set():
                    raise ResetCancelled("Experiment cancelled before staging")
                # The previous trial ended at home, so this observation repeats its measurement without motion.
                manager.observe(detect_objects=True, object_ids=(object_id,))
                cell_target = None
                if cells is not None:
                    column, row = cells[(trial - 1) % len(cells)]
                    cell_target = TargetConfiguration(buffer_id=manager.buffer.object_id)
                    cell_target.set_cell(object_id, manager.buffer, column, row,
                                         scene_object=cube, support_height_m=0.0)
                print(f"Trial {trial}/{self.runs}: finding a home-visible table staging pose"
                      + ("" if cell_target is None else f" from which cell ({column}, {row}) is reachable") + "...",
                      flush=True)
                geometry = manager.state["last_observation"]
                base_left = np.asarray(geometry["base_left_camera"])
                cell = manager.buffer.cell_size_mm / 1000
                low = -np.asarray(manager.buffer.origin) * cell
                high = (np.array([manager.buffer.columns, manager.buffer.rows]) - manager.buffer.origin) * cell
                local_buffer = np.array([[low[0], low[1], 0], [high[0], low[1], 0],
                                         [high[0], high[1], 0], [low[0], high[1], 0]])
                buffer_polygon = (local_buffer @ manager.buffer.pose[:3, :3].T + manager.buffer.pose[:3, 3])[:, :2].astype(np.float32)
                options = manager.station_config.data.get("reset", {})
                # A bad current pickup must fail rather than reject every random destination.
                current = TargetConfiguration(buffer_id=manager.buffer.object_id)
                current.set_pose(object_id, np.linalg.inv(manager.buffer.pose) @ cube.pose)
                manager.plan(current, solve_ik=True, rescan_objects=False,
                             observation_tool_pose=home_pose)
                scan_options = {}
                if options.get("rescan_before_pick", True):
                    scan_options = {"tool_left_camera": np.asarray(geometry["tool_left_camera"]),
                                    "left_right_camera": geometry["eyes"].get("right", {}).get("left_camera_eye"),
                                    "scan_distance_m": options.get("scan_distance_m", 0.25)}
                    manager.plan(current, solve_ik=True, rescan_objects=True,
                                 observation_tool_pose=home_pose)
                attempts = 0
                while True:
                    if manager._cancel_requested.is_set():
                        raise ResetCancelled("Experiment cancelled while sampling staging")
                    attempts += 1
                    x, y = rng.uniform([bounds[0], bounds[2]], [bounds[1], bounds[3]])
                    yaw = float(rng.uniform(-np.pi, np.pi))
                    table_z = manager.station_config.data["planar_scene"]["table_z_m"]
                    base_stage = pose_to_transform([x, y, table_z + h[2], 0, 0, yaw])
                    world = top @ base_stage[:3, :3].T + base_stage[:3, 3]
                    footprint = world[:, :2].astype(np.float32)
                    if (np.any(footprint[:, 0] < bounds[0]) or np.any(footprint[:, 0] > bounds[1])
                            or np.any(footprint[:, 1] < bounds[2]) or np.any(footprint[:, 1] > bounds[3])):
                        continue
                    if cv2.intersectConvexConvex(footprint, buffer_polygon)[0] > 0:
                        continue
                    visible_eyes = []
                    for view, eye in geometry["eyes"].items():
                        camera_base = np.linalg.inv(base_left @ np.asarray(eye["left_camera_eye"]))
                        camera_points = world @ camera_base[:3, :3].T + camera_base[:3, 3]
                        if np.any(camera_points[:, 2] <= 0):
                            continue
                        pixels = cv2.projectPoints(world, cv2.Rodrigues(camera_base[:3, :3])[0],
                                                  camera_base[:3, 3], np.asarray(eye["matrix"]),
                                                  np.asarray(eye["distortion"]))[0].reshape(-1, 2)
                        width, height = eye["image_size_px"]
                        if np.all(pixels >= 0) and np.all(pixels < [width - 1, height - 1]):
                            visible_eyes.append(view)
                    if not visible_eyes:
                        continue
                    stage = TargetConfiguration(buffer_id=manager.buffer.object_id)
                    stage.set_pose(object_id, np.linalg.inv(manager.buffer.pose) @ base_stage)
                    try:
                        manager.plan(stage, solve_ik=True, rescan_objects=False,
                                     observation_tool_pose=home_pose)
                        staged_cube = deepcopy(cube)
                        staged_cube.update_pose(base_stage, "base", time.time())
                        checks = []
                        if scan_options:
                            checks.append(manager.planner.plan_pick_place(staged_cube, base_stage,
                                          current_tool_pose=home_pose, **scan_options))
                        if cell_target is not None:
                            # A staging pose can be resampled; a fixed cell cannot.
                            checks.append(manager.planner.plan_pick_place(
                                staged_cube, manager.buffer.pose @ cell_target.pose_for(object_id),
                                current_tool_pose=home_pose, observation_tool_pose=home_pose))
                        for check in checks:
                            seed_joints = home
                            for action in check:
                                if action["type"] == "move":
                                    pose = DroidRobot._clip_cartesian_target(np.asarray(action["pose"]))
                                    seed_joints = np.asarray(manager.robot.joint_target_for_pose(pose, seed_joints=seed_joints))
                    except RuntimeError as exc:
                        if "no feasible IK solution" not in str(exc):
                            raise
                        continue
                    events.emit("staging_selected", trial=trial, attempts=attempts,
                                base_cube_pose=base_stage.tolist(), visible_eyes=visible_eyes)
                    break
                if manager._cancel_requested.is_set():
                    raise ResetCancelled("Experiment cancelled before staging transfer")
                print(f"Trial {trial}/{self.runs}: random table target base XYZ "
                      f"[{x:.3f}, {y:.3f}, {table_z + h[2]:.3f}] m, yaw {np.degrees(yaw):.1f} deg; one transfer.", flush=True)
                manager.reset(stage, use_current_observations=True, max_manipulations=1)
                events.emit("staging_measured", trial=trial, scene=dict(manager.state)["scene_after"])
                self._move_home(home, events, trial=trial, before="placement")
                print(f"Trial {trial}/{self.runs}: rescanning buffer and cube before placement...", flush=True)
                manager.observe(detect_buffer=True, detect_objects=True, object_ids=(object_id,))
                target, free_cells = cell_target, []
                if cell_target is not None:
                    try:
                        manager.plan(cell_target, solve_ik=True, rescan_objects=False,
                                     observation_tool_pose=home_pose)
                    except RuntimeError as exc:
                        if "no feasible IK solution" not in str(exc):
                            raise
                        events.emit("cell_unreachable", trial=trial, column=column, row=row)
                        raise RuntimeError(f"Fixed cell ({column}, {row}) is unreachable from the "
                                           f"measured staging pose: {exc}") from exc
                else:
                    free_cells = [(column, row) for column in range(manager.buffer.columns)
                                  for row in range(manager.buffer.rows) if manager.buffer.item_at(column, row) is None]
                for index in rng.permutation(len(free_cells)):
                    column, row = free_cells[int(index)]
                    candidate = TargetConfiguration(buffer_id=manager.buffer.object_id)
                    candidate.set_cell(object_id, manager.buffer, column, row,
                                       scene_object=cube, support_height_m=0.0)
                    try:
                        manager.plan(candidate, solve_ik=True, rescan_objects=False,
                                     observation_tool_pose=home_pose)
                    except RuntimeError as exc:
                        if "no feasible IK solution" not in str(exc):
                            raise
                        events.emit("cell_unreachable", trial=trial, column=column, row=row)
                        continue
                    target = candidate
                    break
                if target is None:
                    raise RuntimeError("No free buffer cell has a feasible transfer from the measured staging pose")
                print(f"Trial {trial}/{self.runs}: place once into cell ({column}, {row}), then measure...", flush=True)
                if manager._cancel_requested.is_set():
                    raise ResetCancelled("Experiment cancelled before placement")
                started = time.monotonic()
                manager.reset(target, use_current_observations=True, max_manipulations=1,
                              rescan_buffer_after_action=True)
                duration = time.monotonic() - started
                state = dict(manager.state)
                # The request verified from where its return stopped; measure from the home view instead.
                self._move_home(home, events, trial=trial, before="measurement")
                manager.observe(detect_buffer=True, detect_objects=True, object_ids=(object_id,))
                observed = cube.pose is not None
                dx = dy = distance = angle = None
                success = False
                if observed:
                    actual = np.linalg.inv(manager.buffer.pose) @ cube.pose
                    desired = manager.target_configuration.pose_for(object_id)
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
                          "reset_success": bool(state["success"]),
                          "manipulations": state["manipulations"], "duration_s": duration}
                writer.writerow(result)
                csv_file.flush()
                rows.append(result)
                events.emit("trial_measured", **result, state=state,
                            base_buffer=manager.buffer.pose.tolist(),
                            target_buffer_pose=manager.target_configuration.pose_for(object_id).tolist())
                measured = "cube not detected" if distance is None else f"XY error {distance:.2f} mm, yaw error {angle:.2f} deg"
                print(f"Trial {trial}/{self.runs}: {measured}; success={result['success']}", flush=True)
                if not observed:
                    raise RuntimeError("Cube missing after placement; recorded the failed trial and stopped before further motion")
            status = "completed"
        except (Exception, KeyboardInterrupt) as exc:
            status = "cancelled" if isinstance(exc, (ResetCancelled, KeyboardInterrupt)) else "failed"
            failure = str(exc) or type(exc).__name__
            events.emit("experiment_stopped", status=status, reason=failure, traceback=traceback.format_exc())
            raise
        finally:
            errors = [r["xy_error_mm"] for r in rows if r["xy_error_mm"] is not None]
            summary = {"status": status, "requested_trials": self.runs, "completed_trials": len(rows),
                       "placement_successes": sum(r["success"] for r in rows), "seed": seed,
                       "mean_xy_error_mm": None if not errors else float(np.mean(errors)),
                       "max_xy_error_mm": None if not errors else float(np.max(errors)),
                       "failure_reason": failure}
            (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            if csv_file is not None:
                csv_file.close()
            events.close()
        return summary

    def _move_home(self, home: np.ndarray, events: EventLog, **context) -> np.ndarray:
        """Command the saved home joints and log the reached state; returns the tool pose."""
        manager = self.manager
        if manager._cancel_requested.is_set():
            raise ResetCancelled("Experiment cancelled before moving home")
        manager.robot.command_joints(home.tolist(), blocking=True)
        manager.robot.wait_until_stopped(timeout_s=manager.motion_timeout_s)
        reached = manager.robot.read_state()
        events.emit("home_reached", commanded_joints=home.tolist(), joints=reached.joints.tolist(),
                    tool_pose=reached.tool_pose.tolist(), **context)
        return reached.tool_pose.copy()
