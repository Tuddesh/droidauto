"""Existing DROID hardware backend, independent of experiment entry points.

Connections and controller initialization are explicit. The current endpoint
IK/joint adapter is retained. Gripper commands and motion-settling waits use
the existing server API. No reset orchestration or GUI belongs in this module.
"""

from dataclasses import dataclass
import time

import numpy as np

from scene.transforms import pose_to_transform, transform, transform_to_pose


@dataclass
class RobotState:
    tool_pose: np.ndarray
    joints: np.ndarray
    joint_velocities: np.ndarray
    timestamp_s: float
    command_successful: bool


class DroidRobot:
    def __init__(self, endpoint="tcp://192.168.88.12:4242", client=None, max_state_age_s=0.5):
        self.endpoint = endpoint
        self._client = client
        self._owns_client = client is None
        self.max_state_age_s = max_state_age_s
        self._last_stamp = None

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
        """Explicitly restart controllers using the existing project startup helper."""
        self._relaunch_controllers()
        self._last_stamp = None

    def _relaunch_controllers(self, attempts: int = 5) -> None:
        """Preserve the existing explicit full-pair startup retry sequence."""
        import zerorpc

        if attempts < 1:
            raise ValueError("At least one relaunch attempt is required")
        for attempt in range(1, attempts + 1):
            try:
                print(f"Startup {attempt}/{attempts}: launch_controller()...", flush=True)
                self._client.launch_controller()
                print(f"Startup {attempt}/{attempts}: launch_robot()...", flush=True)
                self._client.launch_robot()
                return
            except zerorpc.exceptions.RemoteError as exc:
                print(f"relaunch attempt {attempt}/{attempts} failed: {exc.name}", flush=True)
                if attempt == attempts:
                    raise RuntimeError(f"Robot server relaunch failed after {attempts} attempts: {exc}") from exc
                time.sleep(0.1)

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

    def joint_target_for_pose(self, target):
        """Ask DROID's working endpoint IK helper for seven plain joint values.

        The installed blocking update_pose passes the (joints, success) IK
        tuple straight into torch.Tensor. The existing add_noise_to_joints RPC
        correctly unpacks that tuple. Despite its name, it applies the supplied
        deterministic Cartesian offset and solves endpoint IK; it adds no random
        noise and sends no motion. Use the same base-frame pose difference as
        DROID's pose_diff/add_poses (not a tool-frame SE(3) relative transform).
        """
        target = transform(target)
        state = self.read_state()
        delta = np.eye(4)
        delta[:3, 3] = target[:3, 3] - state.tool_pose[:3, 3]
        delta[:3, :3] = target[:3, :3] @ state.tool_pose[:3, :3].T
        joints = np.asarray(self._client.add_noise_to_joints(
            state.joints.tolist(), transform_to_pose(delta)), dtype=float)
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise RuntimeError("DROID endpoint IK must return seven finite joint angles")
        return joints.tolist()

    def command_pose(self, target, blocking=True):
        """Solve an endpoint on DROID, then send one joint move.

        DROID performs IK and model-limit handling on the server. No motion RPC
        is retried: an exception leaves its outcome uncertain.
        """
        if not blocking:
            return self._client.update_pose(transform_to_pose(target), False, False)
        joints = self.joint_target_for_pose(target)
        return self.command_joints(joints, blocking=True)

    def command_joints(self, joints, blocking=True):
        """Send a taught seven-joint endpoint through DROID's joint API."""
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError("Expected seven finite joint angles in radians")
        return self._client.update_joints(joints.tolist(), False, blocking)

    def open_gripper(self) -> None:
        """Send one blocking normalized-position command: 0 is fully open."""
        return self._client.update_gripper(0.0, False, True)

    def close_gripper(self) -> None:
        """Send one blocking normalized-position command: 1 is fully closed."""
        return self._client.update_gripper(1.0, False, True)

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
