"""Simple endpoint pick-and-place planning from supplied object poses.

The planner composes camera/tool mounts and object grasps into scan or transfer
waypoints. A scan ends at an observation action; pickup is replanned afterwards
from the refined pose. Poses are in robot-base coordinates. It does not run IK,
interpolate trajectories or perform collision avoidance; the transit corridor
must be clear and DROID executes each endpoint.
"""

from __future__ import annotations

from collections.abc import Mapping
import math

import numpy as np

from scene import Cube, SceneObject
from scene.transforms import transform
from .target_configuration import TargetConfiguration

ResetPlan = list[dict[str, object]]


class ResetPlanner:
    """Build ordered endpoint actions; ResetManager owns their execution."""

    def __init__(self, *, transit_height_m: float, lift_clearance_m: float = 0.08):
        if not math.isfinite(transit_height_m):
            raise ValueError("Transit height must be finite (absolute robot-base Z)")
        if not math.isfinite(lift_clearance_m) or lift_clearance_m <= 0:
            raise ValueError("Lift clearance must be positive")
        self.transit_height_m = float(transit_height_m)
        self.lift_clearance_m = float(lift_clearance_m)

    def plan(self, objects: Mapping[str, SceneObject], target_configuration: TargetConfiguration,
             *, buffer_pose: np.ndarray, current_tool_pose: np.ndarray,
             tool_left_camera: np.ndarray | None = None,
             left_right_camera: np.ndarray | None = None,
             scan_distance_m: float = 0.25) -> ResetPlan:
        """Plan transfers, or the first close-up scan when a camera mount is supplied."""
        buffer_pose, current = transform(buffer_pose), transform(current_tool_pose)
        actions = []
        for object_id in target_configuration.poses:
            if object_id not in objects:
                raise KeyError(f"Unknown target object: {object_id}")
            target = buffer_pose @ target_configuration.pose_for(object_id)
            transfer = self.plan_pick_place(
                objects[object_id], target, current_tool_pose=current,
                tool_left_camera=tool_left_camera, left_right_camera=left_right_camera,
                scan_distance_m=scan_distance_m,
            )
            actions.extend(transfer)
            if tool_left_camera is not None:
                # Later waypoints require the observation at this scan pose.
                break
            current = transform(transfer[-1]["pose"])
        return actions

    def plan_pick_place(self, scene_object: SceneObject, target_pose: np.ndarray, *,
                        current_tool_pose: np.ndarray, grasp_index: int = 0,
                        tool_left_camera: np.ndarray | None = None,
                        left_right_camera: np.ndarray | None = None,
                        scan_distance_m: float = 0.25) -> ResetPlan:
        """Plan a calibrated close-up scan, or a transfer using T_object_tool."""
        if scene_object.pose is None or scene_object.pose_frame != "base":
            raise ValueError(f"{scene_object.object_id} needs a supplied pose in the base frame")
        current = transform(current_tool_pose)
        if tool_left_camera is not None:
            if not isinstance(scene_object, Cube):
                raise ValueError("Close-up scanning requires a marker-topped cube")
            if not math.isfinite(scan_distance_m) or scan_distance_m <= 0:
                raise ValueError("Scan distance must be positive")
            mount = transform(tool_left_camera)
            marker = transform(scene_object.pose) @ scene_object.marker_mount_transform()
            # Preserve the current camera heading; optical +Z points down in base.
            camera_now = current @ mount
            yaw = math.atan2(camera_now[1, 0], camera_now[0, 0])
            c, s = math.cos(yaw), math.sin(yaw)
            camera_scan = np.eye(4)
            camera_scan[:3, :3] = [[c, s, 0], [s, -c, 0], [0, 0, -1]]
            midpoint = (np.zeros(3) if left_right_camera is None
                        else transform(left_right_camera)[:3, 3] / 2)
            marker_in_camera = midpoint + [0, 0, scan_distance_m]
            camera_scan[:3, 3] = marker[:3, 3] - camera_scan[:3, :3] @ marker_in_camera
            scan = transform(camera_scan @ np.linalg.inv(mount))
            height = max(self.transit_height_m, float(current[2, 3]), float(scan[2, 3]))
            lift, above_scan = current.copy(), scan.copy()
            lift[2, 3] = above_scan[2, 3] = height
            actions = [
                {"type": "move", "name": name, "object_id": scene_object.object_id,
                 "pose": pose.tolist()}
                for name, pose in (("initial_lift", lift), ("above_scan", above_scan), ("scan", scan))
            ]
            actions.insert(1, {"type": "open_gripper", "name": "open_gripper",
                               "object_id": scene_object.object_id})
            return actions + [{"type": "rescan_object", "name": "rescan_object",
                               "object_id": scene_object.object_id, "scan_distance_m": scan_distance_m}]
        grasp = scene_object.grasp_poses()[grasp_index]
        source = transform(scene_object.pose) @ grasp
        destination = transform(target_pose) @ grasp
        height = max(self.transit_height_m, float(current[2, 3]),
                     float(source[2, 3]) + self.lift_clearance_m,
                     float(destination[2, 3]) + self.lift_clearance_m)

        def above(pose):
            result = pose.copy()
            result[2, 3] = height
            return result

        def move(name, pose):
            return {"type": "move", "name": name, "object_id": scene_object.object_id,
                    "pose": transform(pose).tolist()}

        def gripper(kind):
            return {"type": kind, "name": kind, "object_id": scene_object.object_id}

        return [move("initial_lift", above(current)), gripper("open_gripper"),
                move("above_pick", above(source)), move("pick", source),
                gripper("close_gripper"), move("lift", above(source)),
                move("above_place", above(destination)), move("place", destination),
                gripper("open_gripper"), move("retreat", above(destination))]
