"""Small DROID adapter with fresh measured feedback; no controller startup on import."""

from dataclasses import dataclass
import time

import numpy as np

from .transforms import pose_to_transform, transform_to_pose


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
        from scripts.droid_rpc import relaunch
        relaunch(self._client)
        self._last_stamp = None

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

    def command_pose(self, target):
        """Stream an absolute Cartesian target; leave the gripper untouched.

        DROID performs IK and model-limit handling on the server. No motion RPC
        is retried: an exception leaves its outcome uncertain.
        """
        return self._client.update_pose(transform_to_pose(target), False, False)
