"""Existing DROID hardware backend, independent of experiment entry points.

Connections and controller initialization are explicit. Endpoint IK runs locally
on an NVIDIA GPU with cuRobo; DROID receives joint targets only. Gripper commands
and motion-settling waits use the existing server API.
"""

from dataclasses import dataclass
import logging
import time

import numpy as np

from scene.transforms import pose_to_transform, transform


LOG = logging.getLogger(__name__)
MIN_GRIPPING_POINT_BASE_Z_M = -0.025
FLANGE_TO_GRIPPING_POINT_M = 0.172
GRIPPER_SETTLE_S = 2.5


@dataclass
class RobotState:
    tool_pose: np.ndarray
    joints: np.ndarray
    joint_velocities: np.ndarray
    timestamp_s: float
    command_successful: bool


class DroidRobot:
    def __init__(self, endpoint="tcp://192.168.88.12:4242", client=None, max_state_age_s=0.5,
                 *, ik_config=None):
        self.endpoint = endpoint
        self._client = client
        self._owns_client = client is None
        self.max_state_age_s = max_state_age_s
        self._last_stamp = None
        self.ik_config = ik_config
        self._ik_solver = None
        self.last_ik = None

    def __enter__(self):
        if self._client is None:
            import zerorpc
            self._client = zerorpc.Client(heartbeat=20, timeout=60)
            self._client.connect(self.endpoint)
        return self

    def __exit__(self, *_):
        if self._owns_client and self._client is not None:
            self._client.close()
            self._client = None

    def initialize(self):
        """Explicitly launch controllers and attach only when both interfaces work."""
        self._relaunch_controllers()
        self._last_stamp = None

    def _relaunch_controllers(self, timeout_s: float = 30.0, poll_interval_s: float = 0.5) -> None:
        """Initialize using existing DROID RPCs, with one bounded gripper recovery.

        Polymetis can swallow the gripper metadata RPC error and return a
        half-built GripperInterface. Retry transient attachment failures. If
        gripper metadata remains unavailable for five seconds, relaunch the
        existing controllers once within this initialization call. No server
        files or installed drivers are modified. Unrelated errors propagate.
        """
        import zerorpc

        if not np.isfinite([timeout_s, poll_interval_s]).all() or min(timeout_s, poll_interval_s) <= 0:
            raise ValueError("Startup timeout and polling interval must be positive")
        print("Starting arm and gripper controllers...", flush=True)
        self._client.launch_controller()
        print("Attaching arm and gripper interfaces...", flush=True)
        deadline = time.monotonic() + timeout_s
        last_error = None
        waiting_reported = False
        attached = False
        previous_stamp = None
        metadata_pending = False
        metadata_missing_since = None
        recovery_used = False
        while True:
            try:
                if not attached:
                    self._client.launch_robot()
                    attached = True
                # The gripper metadata attribute alone does not establish a
                # functioning state connection. Exercise the complete read path.
                state, timestamps = self._client.get_robot_state()
                if (np.asarray(state["joint_positions"]).shape != (7,)
                        or not np.isfinite(state["joint_positions"]).all()
                        or not np.isfinite(float(state["gripper_position"]))):
                    raise RuntimeError("DROID initialized but returned invalid arm/gripper feedback")
                stamp = (float(timestamps["robot_timestamp_seconds"])
                         + float(timestamps["robot_timestamp_nanos"]) / 1e9)
                if not np.isfinite(stamp):
                    raise RuntimeError("DROID initialized but returned an invalid feedback timestamp")
                if previous_stamp is not None and stamp > previous_stamp:
                    print("Arm and gripper interfaces ready; robot feedback is advancing.", flush=True)
                    return
                previous_stamp = stamp
                last_error = None
                metadata_missing_since = None
            except zerorpc.exceptions.RemoteError as exc:
                message = f"{exc.msg}\n{exc.traceback}"
                missing_metadata = (exc.name == "AttributeError" and (
                    ("GripperInterface" in message and "metadata" in message)
                    or "_max_gripper_width" in message))
                grpc_starting = (exc.name == "_InactiveRpcError" and any(text in message for text in (
                    "Robot context not valid", "StatusCode.UNAVAILABLE", "StatusCode.CANCELLED")))
                # The existing gripper server returns None until driver metadata
                # is registered; gRPC reports this as a serialization failure.
                metadata_pending = (exc.name == "_InactiveRpcError" and "StatusCode.INTERNAL" in message
                                    and any(text in message.lower() for text in (
                                        "failed to serialize response", "exception serializing response")))
                if not (missing_metadata or grpc_starting or metadata_pending):
                    raise
                last_error = exc
                attached = False
                previous_stamp = None
                now = time.monotonic()
                if missing_metadata or metadata_pending:
                    if metadata_missing_since is None:
                        metadata_missing_since = now
                    if not recovery_used and now - metadata_missing_since >= 5.0 and now < deadline:
                        LOG.warning("Gripper initialization stayed incomplete; recovering DROID controllers once.")
                        recovery_used = True
                        self._client.launch_controller()
                        metadata_missing_since = None
                else:
                    metadata_missing_since = None
                if not waiting_reported:
                    print(f"Interfaces not ready ({exc.name}); waiting within this startup...", flush=True)
                    waiting_reported = True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if last_error is None:
                    detail = "Robot feedback remained frozen after interface attachment."
                elif last_error.name == "AttributeError" or metadata_pending:
                    detail = "Gripper metadata stayed unavailable."
                    if recovery_used:
                        detail += " One controller recovery was already attempted."
                else:
                    detail = str(last_error.msg)
                raise TimeoutError(f"DROID startup did not become ready within {timeout_s:g} s. {detail}") from last_error
            time.sleep(min(poll_interval_s, remaining))

    def read_state(self) -> RobotState:
        # Adjacent orchestration calls can fall within one server update. Allow
        # a brief wait for a new sample; never reuse a frozen sample for motion.
        for attempt in range(4):
            state, stamp = self._client.get_robot_state()
            timestamp = float(stamp["robot_timestamp_seconds"]) + float(stamp["robot_timestamp_nanos"]) / 1e9
            if self._last_stamp is None or timestamp > self._last_stamp:
                break
            if attempt < 3:
                time.sleep(0.01)
        if not np.isfinite(timestamp) or abs(time.time() - timestamp) > self.max_state_age_s:
            raise RuntimeError("Robot feedback is stale or PC/NUC clocks differ")
        if self._last_stamp is not None and timestamp <= self._last_stamp:
            raise RuntimeError("Robot feedback timestamp did not advance")
        self._last_stamp = timestamp
        joints = np.asarray(state["joint_positions"], dtype=float)
        velocities = np.asarray(state["joint_velocities"], dtype=float)
        if joints.shape != (7,) or velocities.shape != (7,) or not np.isfinite([joints, velocities]).all():
            raise RuntimeError("Expected seven finite measured joint positions and velocities")
        return RobotState(pose_to_transform(state["cartesian_position"]), joints,
                          velocities, timestamp, bool(state["prev_command_successful"]))

    def require_stationary(self) -> RobotState:
        first = self.read_state()
        time.sleep(0.15)
        second = self.read_state()
        if (np.max(np.abs(second.joint_velocities)) > 0.05
                or np.max(np.abs(second.joints - first.joints)) > 0.005):
            raise RuntimeError("Arm is moving; stationary feedback is required")
        return second

    def read_calibration_pose(self) -> tuple[np.ndarray, dict]:
        """Read the tool pose for an operator-triggered stationary capture.

        No age, clock synchronization, advancing-timestamp or velocity gate:
        the operator establishes stationarity when requesting calibration.
        """
        state, timestamps = self._client.get_robot_state()
        return pose_to_transform(state["cartesian_position"]), timestamps

    @staticmethod
    def _clip_cartesian_target(target) -> np.ndarray:
        """Raise a flange target to keep the gripping point at base Z >= -25 mm.

        The gripping point is 172 mm along flange +Z. Only base-frame Z is
        changed; XY and orientation are preserved. This constrains endpoints,
        not DROID's joint-space trajectory or arbitrary direct joint commands.
        """
        target = transform(target)
        offset_z = float(target[2, 2]) * FLANGE_TO_GRIPPING_POINT_M
        minimum_flange_z = MIN_GRIPPING_POINT_BASE_Z_M - offset_z
        if target[2, 3] < minimum_flange_z:
            requested_point_z = float(target[2, 3]) + offset_z
            target[2, 3] = minimum_flange_z
            LOG.warning(
                "Clipped Cartesian target: gripping-point base Z %.4f m -> %.4f m "
                "(flange Z raised to %.4f m; XY and orientation preserved)",
                requested_point_z, MIN_GRIPPING_POINT_BASE_Z_M, minimum_flange_z,
            )
        return target

    def joint_target_for_pose(self, target, *, seed_joints=None):
        """Solve flange IK locally on CUDA and return joints in DROID order.

        The cached cuRobo solver uses the Panda arm, without the stock Panda Hand
        collision geometry. Joint bounds come from station configuration and
        are intersected with the model limits before optimization. Supplying a
        seed allows a whole endpoint sequence to be solved without robot calls.
        No remote IK, Cartesian command or CPU fallback is used.
        """
        target = self._clip_cartesian_target(target)
        self.last_ik = None
        if self.ik_config is None:
            raise ValueError("Local cuRobo IK requires the station's ik configuration; see station.example.json")
        try:
            import torch
            from curobo.config_io import load_yaml
            from curobo.content import get_robot_configs_path
            from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
            from curobo.types import DeviceCfg, GoalToolPose, JointState, Pose
            from curobo._src.types.robot import RobotCfg
        except ImportError as exc:
            raise RuntimeError("Install DroidAuto's CUDA dependencies from environment.yml; "
                               "local IK requires CUDA PyTorch and cuRobo V2") from exc
        from scipy.spatial.transform import Rotation

        options = self.ik_config
        device = torch.device(options["device"])
        if device.type != "cuda" or not torch.cuda.is_available():
            raise RuntimeError("cuRobo IK requires an available NVIDIA CUDA GPU; CPU fallback is disabled")
        if self._ik_solver is None:
            from copy import deepcopy
            names = [f"panda_joint{i}" for i in range(1, 8)]
            lower = np.asarray(options["joint_lower_rad"], dtype=float)
            upper = np.asarray(options["joint_upper_rad"], dtype=float)
            margin = float(options["joint_margin_rad"])
            if (lower.shape != (7,) or upper.shape != (7,) or
                    not np.isfinite([lower, upper]).all() or not np.isfinite(margin) or margin < 0):
                raise ValueError("IK needs seven finite lower/upper joint bounds and a nonnegative margin")
            if np.any(lower + margin >= upper - margin):
                raise ValueError("IK joint margin leaves an empty joint range")
            if options["tool_frame"] != "panda_link8":
                raise ValueError("DROID reports the panda_link8 flange; IK must target that same frame")
            seeds = options["num_seeds"]
            if isinstance(seeds, bool) or not isinstance(seeds, int) or seeds < 1:
                raise ValueError("ik.num_seeds must be a positive integer")
            for key in ("position_tolerance_m", "orientation_tolerance_rad"):
                if not np.isfinite(options[key]) or options[key] <= 0:
                    raise ValueError(f"ik.{key} must be finite and positive")
            robot_data = deepcopy(load_yaml(str(get_robot_configs_path() / options["robot_config"])))["robot_cfg"]
            kin = robot_data["kinematics"]
            kin["tool_frames"] = [options["tool_frame"]]
            # The actual tool is a Robotiq. Keep the supplied Panda arm spheres,
            # not fictitious Panda fingers/hand or an attached object.
            links = [f"panda_link{i}" for i in range(8)]
            kin["collision_link_names"] = links
            kin["mesh_link_names"] = links
            kin["extra_links"] = {}
            kin["extra_collision_spheres"] = {}
            kin["lock_joints"] = {}
            kin["grasp_contact_link_names"] = []
            for key in ("collision_spheres", "self_collision_buffer"):
                kin[key] = {name: value for name, value in kin[key].items() if name in links}
            kin["self_collision_ignore"] = {
                name: [other for other in others if other in links]
                for name, others in kin["self_collision_ignore"].items() if name in links
            }
            cspace = kin["cspace"]
            indices = [cspace["joint_names"].index(name) for name in names]
            for key in ("default_joint_position", "null_space_weight", "cspace_distance_weight"):
                cspace[key] = [cspace[key][index] for index in indices]
            cspace["joint_names"] = names
            model = RobotCfg.create(robot_data, device_cfg=DeviceCfg(device=device, dtype=torch.float32))
            bounds = model.kinematics.get_joint_limits()
            if bounds.joint_names != names:
                raise RuntimeError(f"Unexpected cuRobo joint order: {bounds.joint_names}")
            configured = torch.tensor(np.stack((lower + margin, upper - margin)), device=device, dtype=torch.float32)
            bounds.position[0].copy_(torch.maximum(bounds.position[0], configured[0]))
            bounds.position[1].copy_(torch.minimum(bounds.position[1], configured[1]))
            if torch.any(bounds.position[0] >= bounds.position[1]).item():
                raise ValueError("Configured IK bounds do not overlap the robot model")
            config = InverseKinematicsCfg.create(
                robot=model, device_cfg=DeviceCfg(device=device, dtype=torch.float32),
                num_seeds=seeds, self_collision_check=True,
                position_tolerance=options["position_tolerance_m"],
                orientation_tolerance=options["orientation_tolerance_rad"],
                use_cuda_graph=True,
            )
            self._ik_solver = InverseKinematics(config)
            self._ik_bounds = bounds.position.clone()
            self._ik_joint_names = names
            print(f"cuRobo IK on {torch.cuda.get_device_name(device)} ({device}); flange panda_link8", flush=True)
        seed = np.asarray(self.read_state().joints if seed_joints is None else seed_joints, dtype=float)
        if seed.shape != (7,) or not np.isfinite(seed).all():
            raise ValueError("IK seed must contain seven finite joint angles")
        current = torch.tensor(seed[None, :], device=device, dtype=torch.float32)
        xyzw = Rotation.from_matrix(target[:3, :3]).as_quat()
        goal = Pose(position=torch.tensor(target[None, :3, 3], device=device, dtype=torch.float32),
                    quaternion=torch.tensor(xyzw[[3, 0, 1, 2]][None, :], device=device, dtype=torch.float32))
        result = self._ik_solver.solve_pose(
            GoalToolPose.from_poses({options["tool_frame"]: goal}, num_goalset=1),
            current_state=JointState.from_position(current, joint_names=self._ik_joint_names),
            return_seeds=options["num_seeds"],
        )
        output = result.js_solution
        order = [output.joint_names.index(name) for name in self._ik_joint_names]
        candidates = output.position[..., order].reshape(-1, 7)
        if not candidates.is_cuda or not result.success.is_cuda:
            raise RuntimeError("cuRobo returned CPU results; CUDA-only IK is required")
        valid = (result.success.reshape(-1) & torch.isfinite(candidates).all(dim=1) &
                 (candidates >= self._ik_bounds[0]).all(dim=1) &
                 (candidates <= self._ik_bounds[1]).all(dim=1))
        if not valid.any().item():
            raise RuntimeError("cuRobo found no feasible IK solution within the configured joint bounds; no command sent")
        distance = ((candidates - current) ** 2).sum(dim=1).masked_fill(~valid, float("inf"))
        selected = int(distance.argmin().item())
        solution = candidates[selected]
        joints = solution.detach().cpu().tolist()
        self.last_ik = {
            "backend": "curobo", "device": str(device), "target_pose": target.tolist(),
            "joint_names": self._ik_joint_names, "joints": joints,
            "position_error_m": float(result.position_error.reshape(-1)[selected].item()),
            "orientation_error_rad": float(result.rotation_error.reshape(-1)[selected].item()),
            "min_joint_margin_rad": float(torch.minimum(solution - self._ik_bounds[0],
                                                        self._ik_bounds[1] - solution).min().item()),
        }
        return joints

    def command_pose(self, target, blocking=True):
        """Solve an endpoint on the local CUDA GPU, then send one joint move.

        Blocking and nonblocking calls both use local cuRobo IK. No motion RPC
        is retried: an exception leaves its outcome uncertain.
        """
        target = self._clip_cartesian_target(target)
        joints = self.joint_target_for_pose(target)
        return self.command_joints(joints, blocking=blocking)

    def command_joints(self, joints, blocking=True):
        """Send a taught seven-joint endpoint through DROID's joint API."""
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError("Expected seven finite joint angles in radians")
        return self._client.update_joints(joints.tolist(), False, blocking)

    def open_gripper(self) -> None:
        """Command fully open, then allow 2.5 seconds for physical movement."""
        result = self._client.update_gripper(0.0, False, True)
        time.sleep(GRIPPER_SETTLE_S)
        return result

    def close_gripper(self) -> None:
        """Command fully closed, then allow 2.5 seconds for physical movement."""
        result = self._client.update_gripper(1.0, False, True)
        time.sleep(GRIPPER_SETTLE_S)
        return result

    def read_gripper_position(self) -> float:
        """Read normalized gripper feedback without arm-motion/state-age gates."""
        state, _ = self._client.get_robot_state()
        position = float(state["gripper_position"])
        if not np.isfinite(position):
            raise RuntimeError("DROID returned a nonfinite gripper position")
        return position

    def wait_until_stopped(self, *, timeout_s: float = 60.0,
                           velocity_threshold_rad_s: float = 0.02,
                           settle_s: float = 0.2) -> RobotState:
        """Wait for sustained low measured joint velocities, without endpoint gating.

        DROID owns trajectory execution. This wait only establishes when to
        send the next endpoint; it does not compare measured/commanded poses.
        """
        if (not np.isfinite([timeout_s, velocity_threshold_rad_s, settle_s]).all()
                or min(timeout_s, velocity_threshold_rad_s, settle_s) <= 0):
            raise ValueError("Motion wait parameters must be positive")
        deadline, quiet_since = time.monotonic() + timeout_s, None
        while time.monotonic() < deadline:
            state = self.read_state()
            if np.max(np.abs(state.joint_velocities)) <= velocity_threshold_rad_s:
                quiet_since = time.monotonic() if quiet_since is None else quiet_since
                if time.monotonic() - quiet_since >= settle_s:
                    if not state.command_successful:
                        raise RuntimeError("DROID reports the previous command was unsuccessful")
                    return state
            else:
                quiet_since = None
            time.sleep(0.05)
        raise TimeoutError(f"Robot did not stop moving within {timeout_s:g} seconds")
