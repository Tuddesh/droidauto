"""Reset orchestration, supplied observations and manager-owned outcome state.

No localization runs here. Callers supply base-frame poses, request a reset and
later submit final measurements for verification. Execution delegates each
endpoint/gripper command to DROID; no positional-error gating or interpolation
is inserted. Cancellation takes effect between blocking commands.
"""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass, field
import math
from threading import Event, Lock
import time
from types import MappingProxyType
from typing import TYPE_CHECKING

import numpy as np

from scene import SceneObject
from scene.transforms import pose_error, transform
from .planner import ResetPlan, ResetPlanner
from .target_configuration import TargetConfiguration

if TYPE_CHECKING:
    from buffer import Buffer
    from .hardware import DroidRobot
    from .io import EventLog


class ResetCancelled(RuntimeError):
    """Execution stopped at a command boundary after cancellation was requested."""


@dataclass(kw_only=True, eq=False)
class ResetManager:
    """Own reset execution and results; observations are supplied explicitly."""

    buffer: Buffer
    robot: DroidRobot | None
    planner: ResetPlanner
    log: EventLog | None = None
    objects: dict[str, SceneObject] = field(default_factory=dict)
    target_configuration: TargetConfiguration | None = None
    position_tolerance_m: float = 0.01
    orientation_tolerance_rad: float = 0.1
    motion_timeout_s: float = 60.0
    _state: dict[str, object] = field(default_factory=lambda: {
        "phase": "idle", "current_action": None, "success": None,
        "scene_before": None, "scene_after": None,
        "object_placement_errors": {}, "failure_reason": None, "timing": {},
    }, init=False, repr=False)
    _cancel_requested: Event = field(default_factory=Event, init=False, repr=False)
    _run_lock: Lock = field(default_factory=Lock, init=False, repr=False)

    def __post_init__(self):
        for value in (self.position_tolerance_m, self.orientation_tolerance_rad, self.motion_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Verification tolerances and motion timeout must be positive")
        initial = self.objects
        self.objects = {}
        for scene_object in initial.values():
            self.register_object(scene_object)

    @property
    def state(self) -> Mapping[str, object]:
        """Return a detached snapshot, so callers cannot mutate internal results."""
        return MappingProxyType(deepcopy(self._state))

    def register_object(self, scene_object: SceneObject) -> None:
        """Register a persistent object identity; reject duplicate identities."""
        if self._run_lock.locked():
            raise RuntimeError("Cannot change objects during a reset")
        if not isinstance(scene_object, SceneObject):
            raise TypeError("Expected a SceneObject")
        if scene_object.object_id == self.buffer.object_id or scene_object.object_id in self.objects:
            raise ValueError(f"Duplicate physical object identity: {scene_object.object_id}")
        self.objects[scene_object.object_id] = scene_object

    def remove_object(self, object_id: str) -> None:
        """Remove a registered object when no reset is executing."""
        if self._run_lock.locked():
            raise RuntimeError("Cannot change objects during a reset")
        del self.objects[object_id]

    def observe(self, observations: Mapping[str, np.ndarray] | None = None, *,
                timestamp: float | None = None) -> dict:
        """Accept T_base_object measurements or return the current scene snapshot.

        In an explicit observation batch, omitted task objects become unobserved.
        The fixed buffer pose is retained unless a replacement is supplied.
        There are no camera, localizer or robot-state calls in this method.
        """
        if observations is not None:
            if self._state["phase"] in ("planning", "executing"):
                raise RuntimeError("Cannot replace observations during reset execution")
            timestamp = time.time() if timestamp is None else float(timestamp)
            if not math.isfinite(timestamp):
                raise ValueError("Observation timestamp must be finite")
            entities = {self.buffer.object_id: self.buffer, **self.objects}
            unknown = set(observations) - entities.keys()
            if unknown:
                raise KeyError(f"Unknown observed object(s): {sorted(unknown)}")
            checked = {key: transform(value) for key, value in observations.items()}
            for key, scene_object in self.objects.items():
                if key not in checked:
                    scene_object.mark_unobserved()
            for key, pose in checked.items():
                entities[key].update_pose(pose, "base", timestamp)
            self._record("observations_supplied", object_ids=list(checked), timestamp_s=timestamp)
        return {key: {"pose": None if obj.pose is None else obj.pose.tolist(),
                      "pose_frame": obj.pose_frame, "observed_at": obj.observed_at}
                for key, obj in {self.buffer.object_id: self.buffer, **self.objects}.items()}

    def plan(self, target_configuration: TargetConfiguration, *,
             current_tool_pose: np.ndarray | None = None) -> ResetPlan:
        """Build a complete plan before execution; supplied tool pose allows offline use."""
        if target_configuration.buffer_id != self.buffer.object_id:
            raise ValueError("Target configuration belongs to a different buffer")
        if not target_configuration.poses:
            raise ValueError("A reset request needs at least one target")
        if self.buffer.pose is None or self.buffer.pose_frame != "base":
            raise ValueError("Buffer needs a supplied base-frame pose")
        if current_tool_pose is None:
            if self.robot is None:
                raise ValueError("Offline planning needs current_tool_pose")
            current_tool_pose = self.robot.read_state().tool_pose
        return self.planner.plan(self.objects, target_configuration,
                                 buffer_pose=self.buffer.pose, current_tool_pose=current_tool_pose)

    def reset(self, target_configuration: TargetConfiguration) -> None:
        """Plan/execute once; retain unknown success until final poses are verified."""
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("A reset is already executing")
        try:
            if self.robot is None:
                raise ValueError("Reset execution needs a connected robot backend")
            self._cancel_requested.clear()
            if self._state["phase"] == "awaiting_verification":
                self._record("verification_not_supplied", previous_scene=self._state["scene_before"])
            self.target_configuration = deepcopy(target_configuration)
            self._state.update(phase="planning", current_action=None, success=None,
                               scene_before=self.observe(), scene_after=None,
                               object_placement_errors={}, failure_reason=None,
                               timing={"started_s": time.time()})
            try:
                self._record("reset_started", scene=self._state["scene_before"],
                             targets={key: value.tolist() for key, value in self.target_configuration.poses.items()})
                actions = self.plan(self.target_configuration)
                self._record("plan_created", actions=actions)
                self._state["phase"] = "executing"
                # Once execution begins, the supplied pickup poses cannot describe
                # the resulting scene. Never substitute commanded targets for measurements.
                for object_id in self.target_configuration.poses:
                    self.objects[object_id].mark_unobserved()
                self._execute(actions)
                self._state.update(phase="awaiting_verification", current_action=None)
                self._record("awaiting_verification")
            except (Exception, KeyboardInterrupt) as exc:
                self._state.update(phase="cancelled" if isinstance(exc, (ResetCancelled, KeyboardInterrupt)) else "failed",
                                   success=False, failure_reason=str(exc) or type(exc).__name__)
                self._record("reset_failed", phase=self._state["phase"], reason=self._state["failure_reason"],
                             current_action=self._state["current_action"])
                raise
            finally:
                self._state["timing"]["execution_finished_s"] = time.time()
        finally:
            self._run_lock.release()

    def cancel(self) -> None:
        """Request a stop between commands; cannot interrupt a blocking DROID RPC."""
        self._cancel_requested.set()

    def _execute(self, actions: ResetPlan) -> None:
        """Send endpoints once and wait for joint motion to settle before advancing."""
        for index, action in enumerate(actions):
            if self._cancel_requested.is_set():
                raise ResetCancelled("Cancellation requested; no further commands sent")
            self._state["current_action"] = {"index": index, **action}
            self._record("action_started", **self._state["current_action"])
            kind = action["type"]
            if kind == "move":
                self.robot.command_pose(transform(action["pose"]), blocking=True)
                self.robot.wait_until_stopped(timeout_s=self.motion_timeout_s)
            elif kind == "open_gripper":
                self.robot.open_gripper()
            elif kind == "close_gripper":
                self.robot.close_gripper()
            else:
                raise ValueError(f"Unknown reset action: {kind}")
            self._record("action_finished", index=index, name=action["name"])
        if self._cancel_requested.is_set():
            raise ResetCancelled("Cancellation requested after final command")

    def verify(self, observations: Mapping[str, np.ndarray], *, timestamp: float | None = None) -> None:
        """Submit measured final poses and compute the reset outcome."""
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("Cannot verify while a reset is executing")
        try:
            if self._state["phase"] != "awaiting_verification":
                raise RuntimeError("No executed reset is awaiting verification")
            self.observe(observations, timestamp=timestamp)
            self._verify(self.target_configuration)
        finally:
            self._run_lock.release()

    def _verify(self, target_configuration: TargetConfiguration) -> None:
        """Compare actual final object poses with buffer-relative targets."""
        errors = {}
        for object_id in target_configuration.poses:
            obj = self.objects[object_id]
            if obj.pose is None:
                errors[object_id] = {"observed": False, "within_tolerance": False}
                continue
            desired = self.buffer.pose @ target_configuration.pose_for(object_id)
            distance, angle = pose_error(obj.pose, desired)
            errors[object_id] = {"observed": True, "position_m": distance, "orientation_rad": angle,
                                 "within_tolerance": distance <= self.position_tolerance_m
                                 and angle <= self.orientation_tolerance_rad}
        success = all(error["within_tolerance"] for error in errors.values())
        self._state.update(phase="completed", success=success, current_action=None,
                           scene_after=self.observe(), object_placement_errors=errors,
                           failure_reason=None if success else "Final placement missing or outside tolerance")
        self._state["timing"]["verified_s"] = time.time()
        self._record("reset_verified", success=success, errors=errors, scene=self._state["scene_after"])

    def _record(self, event: str, **details) -> None:
        """Append a reset event when an EventLog was supplied."""
        if self.log is not None:
            self.log.emit(event, **details)
