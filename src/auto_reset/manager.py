"""Reset orchestration, camera/supplied observations and internal outcome state.

Object poses come from camera marker observations or the caller. Existing
Buffer localization supplies the fixed buffer pose, cached until a rescan.
Callers later submit final measurements for verification.
Execution delegates each
endpoint/gripper command to DROID; no positional-error gating or interpolation
is inserted. Cancellation takes effect between blocking commands.
"""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
from threading import Event, Lock
import time
from types import MappingProxyType
from typing import TYPE_CHECKING

import cv2
import numpy as np

from scene import SceneObject
from scene.transforms import pose_error, pose_to_transform, transform
from .planner import ResetPlan, ResetPlanner
from .target_configuration import TargetConfiguration
from .hardware.droid import DroidRobot

if TYPE_CHECKING:
    from buffer import Buffer
    from .hardware import ZedWristCamera
    from .io import EventLog, StationConfig
    from .calibration import BufferCalibration


class ResetCancelled(RuntimeError):
    """Execution stopped at a command boundary after cancellation was requested."""


@dataclass(kw_only=True, eq=False)
class ResetManager:
    """Own resets using supplied object poses and a detected fixed buffer pose."""

    buffer: Buffer
    robot: DroidRobot | None
    planner: ResetPlanner
    camera: ZedWristCamera | None = None
    station_config: StationConfig | None = None
    calibration: BufferCalibration | None = None
    log: EventLog | None = None
    objects: dict[str, SceneObject] = field(default_factory=dict)
    target_configuration: TargetConfiguration | None = None
    position_tolerance_m: float = 0.01
    orientation_tolerance_rad: float = 0.1
    motion_timeout_s: float = 60.0
    aruco_detector: cv2.aruco.ArucoDetector = field(init=False, repr=False)
    _state: dict[str, object] = field(default_factory=lambda: {
        "phase": "idle", "current_action": None, "success": None,
        "scene_before": None, "scene_after": None,
        "object_placement_errors": {}, "failure_reason": None, "timing": {},
    }, init=False, repr=False)
    _cancel_requested: Event = field(default_factory=Event, init=False, repr=False)
    _run_lock: Lock = field(default_factory=Lock, init=False, repr=False)

    def __post_init__(self):
        # Available to observe() for object detection as well as buffer scans.
        self.aruco_detector = self.buffer.aruco_detector
        for value in (self.position_tolerance_m, self.orientation_tolerance_rad, self.motion_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Verification tolerances and motion timeout must be positive")
        initial = self.objects
        self.objects = {}
        for scene_object in initial.values():
            self.register_object(scene_object)
        if self.calibration is not None:
            if self.station_config is not None:
                self.calibration.check_config(self.station_config)
            self.buffer.update_pose(self.calibration.base_buffer, "base",
                                    self.calibration.record["created_at_s"])

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
                timestamp: float | None = None, detect_buffer: bool = False,
                detect_objects: bool = False) -> dict:
        """Accept supplied poses or capture cube markers with the shared detector.

        In an explicit observation batch, omitted task objects become unobserved.
        The buffer is detected through the existing calibration capture utility;
        its actual pose fitting stays in Buffer.localize()/localize_stereo().
        Calling without arguments returns the current snapshot without capture.
        detect_objects=True captures current cube poses; missing cubes become
        unobserved. Scan diagnostics are retained in state['last_observation'].
        """
        if detect_objects and observations is not None:
            raise ValueError("Choose camera object detection or supplied object poses")
        if detect_objects:
            if self._state["phase"] in ("planning", "executing"):
                raise RuntimeError("Cannot replace observations during reset execution")
            if self.camera is None or self.robot is None or self.station_config is None:
                raise ValueError("Object detection needs the station camera, robot connection and configuration")
            for obj in self.objects.values():
                obj.mark_unobserved()
            self._state.pop("last_observation", None)
        checked = {}
        if observations is not None:
            if self._state["phase"] in ("planning", "executing"):
                raise RuntimeError("Cannot replace observations during reset execution")
            timestamp = time.time() if timestamp is None else float(timestamp)
            if not math.isfinite(timestamp):
                raise ValueError("Observation timestamp must be finite")
            if self.buffer.object_id in observations:
                raise ValueError("Supply object poses only; the buffer pose comes from camera detection")
            unknown = set(observations) - self.objects.keys()
            if unknown:
                raise KeyError(f"Unknown observed object(s): {sorted(unknown)}")
            checked = {key: transform(value) for key, value in observations.items()}
        if detect_buffer or ((observations is not None or detect_objects) and self.buffer.pose is None):
            if self._state["phase"] == "executing":
                raise RuntimeError("Cannot rescan the buffer during execution")
            if self.camera is None or self.robot is None or self.station_config is None:
                raise ValueError("Buffer detection needs the station camera, robot connection and configuration")
            from .calibration.buffer import capture_calibration

            self.buffer.mark_unobserved()
            self.calibration = None
            self._record("buffer_detection_started")
            # Automatic base-frame composition needs live feedback around the
            # capture; a frozen pre-stop pose must not become a new calibration.
            self.robot.read_state()
            with self.camera as camera:
                calibration = capture_calibration(self.station_config, self.robot, camera,
                                                  review=self.station_config.camera.get("review_capture", False))
            self.robot.read_state()
            path = self.station_config.resolve(self.station_config.data["calibration_file"])
            calibration.save(path)
            self.buffer.update_pose(calibration.base_buffer, "base", calibration.record["created_at_s"])
            self.calibration = calibration
            self._record("buffer_detected", pose=calibration.base_buffer.tolist(),
                         quality=calibration.record["quality"], calibration_file=str(path))
            print(f"Buffer detected; calibration and source image saved to {path}", flush=True)
        if detect_objects:
            from .calibration.buffer import load_hand_eye
            from .localization import localize_cubes

            config = self.station_config
            self._record("object_detection_started")
            try:
                with self.camera as camera:
                    camera.set_auto_exposure_roi(config.camera.get("auto_exposure_roi"))
                    while True:
                        self.robot.read_state()
                        frames = camera.capture_pair() if camera.view == "stereo" else [camera.capture()]
                        if not frames or any(frame.serial != str(config.camera["serial"]) for frame in frames):
                            raise ValueError("Object scan does not match the configured camera")
                        base_tool = self.robot.read_state().tool_pose
                        if "tool_left_camera_pose" in config.camera:
                            tool_left = pose_to_transform(config.camera["tool_left_camera_pose"])
                        elif self.calibration is not None and "tool_left_camera" in self.calibration.record:
                            tool_left = transform(self.calibration.record["tool_left_camera"])
                        else:
                            try:
                                tool_left = load_hand_eye(config.resolve(config.camera["hand_eye_file"]),
                                                          frames[0].serial, "left")
                            except (ValueError, OSError, KeyError):
                                right = next((frame for frame in frames if frame.view == "right"), None)
                                if right is None or right.left_camera_eye is None:
                                    raise
                                tool_right = load_hand_eye(config.resolve(config.camera["hand_eye_file"]),
                                                           right.serial, "right")
                                tool_left = tool_right @ np.linalg.inv(transform(right.left_camera_eye))
                        directory = (config.resolve(config.data["log_directory"]) / "object_scans" /
                                     datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ"))
                        observations, report = localize_cubes(
                            frames, self.aruco_detector, config.aruco_dictionary_names,
                            self.objects, base_tool @ tool_left, directory,
                            review=config.camera.get("review_capture", False),
                        )
                        self._state["last_observation"] = report
                        self._record("objects_detected", **report)
                        if observations is not None:
                            timestamp = time.time()
                            checked = {key: transform(value) for key, value in observations.items()}
                            break
            except (Exception, KeyboardInterrupt) as exc:
                self._record("object_detection_failed", reason=str(exc) or type(exc).__name__)
                raise
        if observations is not None:
            for key, scene_object in self.objects.items():
                if key not in checked:
                    scene_object.mark_unobserved()
            for key, pose in checked.items():
                self.objects[key].update_pose(pose, "base", timestamp)
            self._record("observations_captured" if detect_objects else "observations_supplied",
                         object_ids=list(checked), timestamp_s=timestamp)
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
        if self.buffer.pose is None:
            self.observe(detect_buffer=True)
        if self.buffer.pose is None or self.buffer.pose_frame != "base":
            raise ValueError("Buffer detection did not establish its base-frame pose")
        if current_tool_pose is None:
            if self.robot is None:
                raise ValueError("Offline planning needs current_tool_pose")
            current_tool_pose = self.robot.read_state().tool_pose
        actions = self.planner.plan(self.objects, target_configuration,
                                    buffer_pose=self.buffer.pose, current_tool_pose=current_tool_pose)
        for action in actions:
            if action["type"] == "move":
                requested = transform(action["pose"])
                clipped = DroidRobot._clip_cartesian_target(requested)
                if not np.array_equal(requested, clipped):
                    self._record("target_clipped", name=action["name"], object_id=action["object_id"],
                                 requested_pose=requested.tolist(), applied_pose=clipped.tolist())
                action["pose"] = clipped.tolist()
        return actions

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
