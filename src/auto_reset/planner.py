"""Reset-planning boundary owned by ResetManager.

A planner will consume observed objects and a desired configuration and produce
ordered robot/gripper actions. No trajectory generation, grasp selection,
pick-and-place policy or existing reset-method integration is implemented here.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from scene import SceneObject
from .target_configuration import TargetConfiguration

ResetPlan = Sequence[Mapping[str, object]]


class ResetPlanner:
    """Future reset-method implementation; the manager owns this collaborator."""

    def plan(self, objects: Mapping[str, SceneObject], target_configuration: TargetConfiguration) -> ResetPlan:
        """Plan restoration of the observed scene; not implemented yet."""
        raise NotImplementedError("Scene reset planning is not implemented")

    def plan_pick_place(self, scene_object: SceneObject, target_pose: np.ndarray) -> ResetPlan:
        """Plan one object's transfer; not implemented yet."""
        raise NotImplementedError("Pick-and-place planning is not implemented")
