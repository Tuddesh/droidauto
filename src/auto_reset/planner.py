"""Simple endpoint pick-and-place planning from supplied object poses.

The planner composes object/tool grasps and produces lift, transit, pick, place
and gripper actions. Poses are in robot-base coordinates. It does not run IK,
interpolate trajectories or perform collision avoidance; the transit corridor
must be clear and DROID executes each endpoint.
"""

from __future__ import annotations

from collections.abc import Mapping
import math

import numpy as np

from scene import SceneObject
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
             *, buffer_pose: np.ndarray, current_tool_pose: np.ndarray) -> ResetPlan:
        """Resolve buffer targets to base and concatenate transfers in target order."""
        buffer_pose, current = transform(buffer_pose), transform(current_tool_pose)
        actions = []
        for object_id in target_configuration.poses:
            if object_id not in objects:
                raise KeyError(f"Unknown target object: {object_id}")
            target = buffer_pose @ target_configuration.pose_for(object_id)
            transfer = self.plan_pick_place(objects[object_id], target, current_tool_pose=current)
            actions.extend(transfer)
            current = transform(transfer[-1]["pose"])
        return actions

    def plan_pick_place(self, scene_object: SceneObject, target_pose: np.ndarray, *,
                        current_tool_pose: np.ndarray, grasp_index: int = 0) -> ResetPlan:
        """Use one configured T_object_tool grasp at pickup and placement."""
        if scene_object.pose is None or scene_object.pose_frame != "base":
            raise ValueError(f"{scene_object.object_id} needs a supplied pose in the base frame")
        grasp = scene_object.grasp_poses()[grasp_index]
        source = transform(scene_object.pose) @ grasp
        destination = transform(target_pose) @ grasp
        current = transform(current_tool_pose)
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
