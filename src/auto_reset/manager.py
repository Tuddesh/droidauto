"""Reset orchestration, camera/supplied observations and internal outcome state.

Object poses come from camera marker observations or the caller. Existing
Buffer localization supplies the fixed buffer pose, cached until a rescan.
Reset execution rescans after each manipulation and chooses its next action
from measured scene errors until all requested objects reach their targets.
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
import traceback
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
                detect_objects: bool = False, object_ids: tuple[str, ...] | None = None,
                capture_count: int = 1) -> dict:
        """Accept supplied poses or capture cube markers with the shared detector.

        In an explicit observation batch, omitted task objects become unobserved.
        The buffer is detected through the existing calibration capture utility;
        its actual pose fitting stays in Buffer.localize()/localize_stereo().
        Calling without arguments returns the current snapshot without capture.
        detect_objects=True captures current cube poses; missing cubes become
        unobserved. Scan diagnostics are retained in state['last_observation'].
        The caller owns one open camera session for the manager's lifetime.
        Automatic scans wait for sustained low joint velocities. A close-up
        can fit multiple stereo pairs from the same stationary camera pose.
        """
        if detect_objects and observations is not None:
            raise ValueError("Choose camera object detection or supplied object poses")
        if object_ids is not None and not detect_objects:
            raise ValueError("object_ids selects objects for camera detection only")
        if isinstance(capture_count, bool) or not isinstance(capture_count, int) or capture_count < 1:
            raise ValueError("capture_count must be a positive integer")
        options = {} if self.station_config is None else self.station_config.data.get("reset", {})
        scan_settle_s = float(options.get("scan_settle_s", 2.5))
        scan_velocity = float(options.get("scan_velocity_threshold_rad_s", 0.005))
        selected = self.objects if object_ids is None else {key: self.objects[key] for key in object_ids}
        if detect_objects:
            if self._state["phase"] in ("planning", "executing"):
                raise RuntimeError("Cannot replace observations during reset execution")
            if self.camera is None or self.robot is None or self.station_config is None:
                raise ValueError("Object detection needs the station camera, robot connection and configuration")
            for obj in selected.values():
                obj.mark_unobserved()
            self._state.pop("last_observation", None)
        checked = {}
        if observations is not None:
            if self._state["phase"] in ("planning", "executing"):
                raise RuntimeError("Cannot replace observations during reset execution")
            self._state.pop("last_observation", None)
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
            self.robot.wait_until_stopped(timeout_s=self.motion_timeout_s,
                                          settle_s=scan_settle_s, velocity_threshold_rad_s=scan_velocity)
            self.camera.set_auto_exposure_roi(self.station_config.camera.get("auto_exposure_roi"))
            calibration = capture_calibration(self.station_config, self.robot, self.camera,
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
                camera = self.camera
                camera.set_auto_exposure_roi(config.camera.get("auto_exposure_roi"))
                while True:
                    if self._cancel_requested.is_set():
                        raise ResetCancelled("Cancellation requested before object capture")
                    self.robot.wait_until_stopped(timeout_s=self.motion_timeout_s,
                                                  settle_s=scan_settle_s, velocity_threshold_rad_s=scan_velocity)
                    frames = []
                    for _ in range(capture_count):
                        if self._cancel_requested.is_set():
                            raise ResetCancelled("Cancellation requested during object capture")
                        before = self.robot.read_state()
                        if np.max(np.abs(before.joint_velocities)) > scan_velocity:
                            break
                        pair = camera.capture_pair() if camera.view == "stereo" else [camera.capture()]
                        if (len(pair) != (2 if camera.view == "stereo" else 1)
                                or any(frame.serial != str(config.camera["serial"]) for frame in pair)):
                            raise ValueError("Object scan does not match the configured camera")
                        after = self.robot.read_state()
                        if np.max(np.abs(after.joint_velocities)) > scan_velocity:
                            break
                        frames.extend(pair)
                    if len(frames) != capture_count * (2 if camera.view == "stereo" else 1):
                        self._record("object_capture_restarted", reason="Arm moving during capture")
                        continue
                    base_tool = after.tool_pose
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
                        selected, base_tool @ tool_left, directory,
                        review=config.camera.get("review_capture", False),
                        planar_scene=config.data.get("planar_scene"), buffer=self.buffer,
                    )
                    report["tool_left_camera"] = tool_left.tolist()
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
            planar = None if self.station_config is None else self.station_config.data.get("planar_scene")
            if planar is not None and not detect_objects:
                buffer_base = np.linalg.inv(self.buffer.pose)
                local = {key: {"pose": buffer_base @ pose} for key, pose in checked.items()}
                normalized = self.station_config.make_target(local, self.buffer, self.objects)
                checked = {key: self.buffer.pose @ pose for key, pose in normalized.poses.items()}
            for key, scene_object in selected.items():
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
             current_tool_pose: np.ndarray | None = None, solve_ik: bool = False,
             rescan_objects: bool | None = None,
             observation_tool_pose: np.ndarray | None = None) -> ResetPlan:
        """Plan the next scan or transfer stage; supplied tool pose allows offline use."""
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
        report = self._state.get("last_observation")
        options = {} if self.station_config is None else self.station_config.data.get("reset", {})
        if rescan_objects is None:
            rescan_objects = report is not None and options.get("rescan_before_pick", True)
        scan_options = {}
        if rescan_objects:
            if report is None:
                raise ValueError("Close-up scanning needs an initial camera observation")
            mount = report.get("tool_left_camera")
            if mount is None:
                raise ValueError("Initial camera observation does not contain its camera mounting transform")
            right = report["eyes"].get("right")
            if right is not None and right.get("left_camera_eye") is None:
                raise ValueError("Stereo scanning needs the measured right-eye transform")
            scan_options = {"tool_left_camera": transform(mount),
                            "left_right_camera": None if right is None else right["left_camera_eye"],
                            "scan_distance_m": options.get("scan_distance_m", 0.25)}
        actions = self.planner.plan(self.objects, target_configuration,
                                    buffer_pose=self.buffer.pose, current_tool_pose=current_tool_pose,
                                    observation_tool_pose=observation_tool_pose, **scan_options)
        if solve_ik and self.robot is None:
            raise ValueError("Joint planning needs a robot backend with local cuRobo IK")
        seed = self.robot.read_state().joints if solve_ik else None
        for action in actions:
            if action["type"] == "move":
                requested = transform(action["pose"])
                clipped = DroidRobot._clip_cartesian_target(requested)
                if not np.array_equal(requested, clipped):
                    self._record("target_clipped", name=action["name"], object_id=action["object_id"],
                                 requested_pose=requested.tolist(), applied_pose=clipped.tolist())
                action["pose"] = clipped.tolist()
                if solve_ik:
                    self._record("ik_started", name=action["name"], object_id=action["object_id"],
                                 target_pose=clipped.tolist(), seed_joints=seed.tolist())
                    joints = self.robot.joint_target_for_pose(clipped, seed_joints=seed)
                    action["joints"] = joints
                    seed = np.asarray(joints, dtype=float)
                    self._record("ik_solved", name=action["name"], object_id=action["object_id"],
                                 **self.robot.last_ik)
        return actions

    def reset(self, target_configuration: TargetConfiguration, *, rescan_objects: bool | None = None,
              use_current_observations: bool = False, detect_buffer: bool = False,
              max_manipulations: int | None = None, rescan_buffer_after_action: bool = False) -> None:
        """Observe/compare/execute one manipulation repeatedly until the target is measured.

        Every completed manipulation returns to the starting observation pose.
        The next full scene observation determines the next action, including
        retries of misplaced objects. By default there is no manipulation limit;
        experiments can cap attempts and return the fresh measured result.
        Caller-supplied observations may seed only the first iteration; later
        iterations always capture fresh images with the existing station camera.
        """
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("A reset is already executing")
        try:
            self._cancel_requested.clear()
            self.target_configuration = deepcopy(target_configuration)
            self._state.update(phase="observing", current_action=None, success=None,
                               scene_before=None, scene_after=None, iterations=0, manipulations=0,
                               object_placement_errors={}, failure_reason=None,
                               timing={"started_s": time.time()})
            try:
                if self.robot is None:
                    raise ValueError("Reset execution needs a connected robot backend")
                if max_manipulations is not None and (isinstance(max_manipulations, bool)
                        or not isinstance(max_manipulations, int) or max_manipulations < 1):
                    raise ValueError("max_manipulations must be a positive integer or None")
                if self.camera is None or self.station_config is None:
                    raise ValueError("Feedback reset requires an open station camera and configuration for rescanning")
                if self.target_configuration.buffer_id != self.buffer.object_id:
                    raise ValueError("Target configuration belongs to a different buffer")
                if not self.target_configuration.poses:
                    raise ValueError("A reset request needs at least one target")
                unknown = set(self.target_configuration.poses) - self.objects.keys()
                if unknown:
                    raise KeyError(f"Unknown target object(s): {sorted(unknown)}")
                if self.station_config.data.get("planar_scene") is not None:
                    self.target_configuration = self.station_config.make_target(
                        {key: {"pose": pose} for key, pose in self.target_configuration.poses.items()},
                        self.buffer, self.objects,
                    )
                observation_tool_pose = self.robot.read_state().tool_pose.copy()
                options = self.station_config.data.get("reset", {})
                if rescan_objects is None:
                    rescan_objects = options.get("rescan_before_pick", True)
                self._record("reset_started", observation_tool_pose=observation_tool_pose.tolist(),
                             max_manipulations=max_manipulations,
                             targets={key: value.tolist() for key, value in self.target_configuration.poses.items()})
                while True:
                    if self._cancel_requested.is_set():
                        raise ResetCancelled("Cancellation requested before scene observation")
                    self._state.update(phase="observing", current_action=None)
                    if use_current_observations and self._state["iterations"] == 0:
                        if self.buffer.pose is None or detect_buffer:
                            self.observe(detect_buffer=True)
                        self._record("initial_observations_used", scene=self.observe())
                    else:
                        self.observe(detect_objects=True,
                                     detect_buffer=(detect_buffer and self._state["iterations"] == 0
                                                    or rescan_buffer_after_action and self._state["manipulations"] > 0),
                                     object_ids=tuple(self.target_configuration.poses))
                    self._state["iterations"] += 1
                    if self._state["scene_before"] is None:
                        self._state["scene_before"] = self.observe()
                    self._state["phase"] = "assessing"
                    self._verify(self.target_configuration, finalize=False)
                    if self._cancel_requested.is_set():
                        raise ResetCancelled("Cancellation requested after scene observation")
                    errors = self._state["object_placement_errors"]
                    if (all(error["within_tolerance"] for error in errors.values())
                            or max_manipulations is not None and self._state["manipulations"] >= max_manipulations):
                        self._verify(self.target_configuration)
                        break
                    pending = [key for key in self.target_configuration.poses
                               if errors[key]["observed"] and not errors[key]["within_tolerance"]]
                    if not pending:
                        self._record("scene_observation_incomplete", iteration=self._state["iterations"],
                                     missing_objects=[key for key, error in errors.items() if not error["observed"]])
                        continue
                    object_id = pending[0]
                    target = TargetConfiguration(buffer_id=self.target_configuration.buffer_id,
                                                 poses={object_id: self.target_configuration.poses[object_id]})
                    self._record("reset_action_selected", iteration=self._state["iterations"],
                                 object_id=object_id, errors=errors[object_id])
                    self._state["phase"] = "planning"
                    # Supplied initial coordinates have no captured camera geometry
                    # yet. Later camera observations enable the close-up stage.
                    scan_pickup = rescan_objects and self._state.get("last_observation") is not None
                    actions = self.plan(target, solve_ik=True, rescan_objects=scan_pickup,
                                        observation_tool_pose=observation_tool_pose)
                    self._record("plan_created", stage="scan" if scan_pickup else "transfer", actions=actions)
                    if scan_pickup:
                        self._state["phase"] = "executing"
                        self._execute(actions)
                        self._state["phase"] = "planning"
                        actions = self.plan(target, solve_ik=True, rescan_objects=False,
                                            observation_tool_pose=observation_tool_pose)
                        self._record("plan_created", stage="transfer", actions=actions)
                    self._state["phase"] = "executing"
                    # A manipulation may disturb other objects. The next decision
                    # must use a new scan of all task objects, never these old poses.
                    for key in self.target_configuration.poses:
                        self.objects[key].mark_unobserved()
                    self._execute(actions)
                    self._state["manipulations"] += 1
                    self._record("reset_action_finished", object_id=object_id,
                                 manipulation=self._state["manipulations"])
            except (Exception, KeyboardInterrupt) as exc:
                self._state.update(phase="cancelled" if isinstance(exc, (ResetCancelled, KeyboardInterrupt)) else "failed",
                                   success=False, failure_reason=str(exc) or type(exc).__name__)
                self._record("reset_failed", phase=self._state["phase"], reason=self._state["failure_reason"],
                             current_action=self._state["current_action"], traceback=traceback.format_exc())
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
                self.robot.command_joints(action["joints"], blocking=True)
                self.robot.wait_until_stopped(timeout_s=self.motion_timeout_s)
            elif kind == "open_gripper":
                self.robot.open_gripper()
            elif kind == "close_gripper":
                self.robot.close_gripper()
            elif kind == "rescan_object":
                object_id = action["object_id"]
                obj = self.objects[object_id]
                coarse_pose = None if obj.pose is None else obj.pose.copy()
                coarse_timestamp = obj.observed_at
                self._state["phase"] = "observing"
                try:
                    options = {} if self.station_config is None else self.station_config.data.get("reset", {})
                    self.observe(detect_objects=True, object_ids=(object_id,),
                                 capture_count=options.get("scan_capture_count", 5))
                finally:
                    self._state["phase"] = "executing"
                if obj.pose is None:
                    if coarse_pose is None:
                        raise ValueError(f"No close-up or coarse pose available for {object_id}")
                    obj.update_pose(coarse_pose, "base", coarse_timestamp)
                    print(f"Warning: close-up could not localize {object_id}; using its coarse pose", flush=True)
                    self._record("object_rescan_fallback", object_id=object_id, pose=coarse_pose.tolist(),
                                 reason=self._state.get("last_observation", {}).get("objects", {}).get(object_id))
                else:
                    self._record("object_pose_refined", object_id=object_id, pose=obj.pose.tolist())
            else:
                raise ValueError(f"Unknown reset action: {kind}")
            self._record("action_finished", index=index, name=action["name"])
        if self._cancel_requested.is_set():
            raise ResetCancelled("Cancellation requested after final command")

    def verify(self, observations: Mapping[str, np.ndarray], *, timestamp: float | None = None) -> None:
        """Reassess an executed reset from independently supplied measurements."""
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("Cannot verify while a reset is executing")
        try:
            if self._state["phase"] not in ("awaiting_verification", "completed"):
                raise RuntimeError("No executed reset is available for verification")
            self.observe(observations, timestamp=timestamp)
            self._verify(self.target_configuration)
        finally:
            self._run_lock.release()

    def _verify(self, target_configuration: TargetConfiguration, *, finalize: bool = True) -> None:
        """Compare measured poses for a loop decision or a final reset outcome."""
        errors = {}
        for object_id in target_configuration.poses:
            obj = self.objects[object_id]
            if obj.pose is None or obj.pose_frame != "base":
                errors[object_id] = {"observed": False, "within_tolerance": False}
                continue
            desired = self.buffer.pose @ target_configuration.pose_for(object_id)
            distance, angle = pose_error(obj.pose, desired)
            errors[object_id] = {"observed": True, "position_m": distance, "orientation_rad": angle,
                                 "within_tolerance": distance <= self.position_tolerance_m
                                 and angle <= self.orientation_tolerance_rad}
        success = all(error["within_tolerance"] for error in errors.values())
        if not finalize:
            self._state.update(scene_after=self.observe(), object_placement_errors=errors)
            self._record("scene_compared", iteration=self._state.get("iterations"),
                         target_reached=success, errors=errors, scene=self._state["scene_after"])
            return
        self._state.update(phase="completed", success=success, current_action=None,
                           scene_after=self.observe(), object_placement_errors=errors,
                           failure_reason=None if success else "Final placement missing or outside tolerance")
        self._state["timing"]["verified_s"] = time.time()
        self._record("reset_verified", success=success, errors=errors, scene=self._state["scene_after"])

    def _record(self, event: str, **details) -> None:
        """Append a reset event when an EventLog was supplied."""
        if self.log is not None:
            self.log.emit(event, **details)
