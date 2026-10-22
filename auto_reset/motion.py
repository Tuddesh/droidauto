"""Buffer-relative waypoints and a feedback-checked Cartesian executor.

This is a geometric planner for the cleared marker-hover workspace. It does
not model obstacles, self-collisions or full-arm swept volumes.
"""

from dataclasses import dataclass
import math
import time

import numpy as np

from .transforms import interpolate, pose_error, transform


class MotionCancelled(Exception):
    """The operator requested a normal task stop."""


@dataclass
class HoverTarget:
    marker_id: int
    tool_pose: np.ndarray


class HoverPlanner:
    """Hover height is TCP distance along the buffer normal, not camera distance."""

    def __init__(self, config, calibration):
        calibration.check_config(config)
        self.grid = calibration.make_buffer(config)
        self.base_buffer = calibration.base_buffer
        self.rotation = calibration.scan_tool_pose[:3, :3]
        self.height = config.task["hover_height_m"]
        self.transit_height = config.task["transit_height_m"]
        self.order = config.task["marker_order"]
        # The single-image solution must describe an upward-facing table buffer.
        if self.base_buffer[2, 2] < math.cos(math.radians(30)):
            raise ValueError("Buffer normal is not table-up (within 30 degrees); inspect calibration")

    def targets(self) -> list[HoverTarget]:
        markers = self.grid.marker_cells()
        result = []
        for marker_id in self.order:
            center = self.grid.cell_center_m(*markers[marker_id])
            center[2] = self.height
            target = np.eye(4)
            target[:3, :3] = self.rotation
            target[:3, 3] = (self.base_buffer @ np.r_[center, 1])[:3]
            result.append(HoverTarget(marker_id, target))
        return result

    def route(self, start, target) -> list[np.ndarray]:
        """Rise, traverse above the buffer, descend; preserve scan orientation.

        The approach raises along buffer +Z, not an obstacle-aware free-space
        path. If already higher than the transit plane, keep that height.
        """
        start, target = transform(start), transform(target)
        inverse = np.linalg.inv(self.base_buffer)
        start_buffer = (inverse @ np.r_[start[:3, 3], 1])[:3]
        target_buffer = (inverse @ np.r_[target[:3, 3], 1])[:3]
        height = max(self.transit_height, start_buffer[2], target_buffer[2])
        above_start, above_target = start.copy(), target.copy()
        start_buffer[2] = target_buffer[2] = height
        above_start[:3, 3] = (self.base_buffer @ np.r_[start_buffer, 1])[:3]
        above_target[:3, 3] = (self.base_buffer @ np.r_[target_buffer, 1])[:3]
        # Change orientation at the raised starting position, before traversing.
        aligned = above_start.copy()
        aligned[:3, :3] = target[:3, :3]
        return [above_start, aligned, above_target, target]


class CartesianExecutor:
    """Stream small absolute poses through DROID's IK, then verify measured arrival."""

    def __init__(self, robot, settings, cancel_requested=None):
        self.robot, self.settings = robot, settings
        self.period = 1 / settings["rate_hz"]
        self.cancel_requested = cancel_requested or (lambda: False)

    def _check_cancel(self):
        if self.cancel_requested():
            raise MotionCancelled("Stopped by operator")

    def _within(self, actual, target) -> bool:
        distance, angle = pose_error(actual, target)
        return (distance <= self.settings["position_tolerance_m"]
                and angle <= self.settings["rotation_tolerance_rad"])

    def _check_tracking(self, state, target):
        distance, angle = pose_error(state.tool_pose, target)
        if (not state.command_successful
                or distance > self.settings["max_tracking_error_m"]
                or angle > self.settings["max_tracking_error_rad"]):
            raise RuntimeError(f"DROID motion failed or tracking error exceeded bounds ({distance:.4f} m, {angle:.4f} rad)")

    def move_to(self, target):
        self._check_cancel()
        start = self.robot.read_state().tool_pose
        target = transform(target)
        distance, angle = pose_error(start, target)
        if self._within(start, target):
            return
        duration = max(distance / self.settings["speed_m_s"],
                       angle / self.settings["rotation_speed_rad_s"], self.period)
        # Smoothstep starts/ends at zero reference velocity. Its peak derivative
        # is 1.5, so extend duration to respect the configured reference speed.
        count = math.ceil(1.5 * duration / self.period)
        previous = start
        for step in range(1, count + 1):
            self._check_cancel()
            tick = time.monotonic()
            state = self.robot.read_state()
            self._check_tracking(state, previous)
            fraction = step / count
            fraction = fraction * fraction * (3 - 2 * fraction)
            previous = interpolate(start, target, fraction)
            self.robot.command_pose(previous)
            time.sleep(max(0, self.period - (time.monotonic() - tick)))
        self._settle(target)

    def _settle(self, target):
        deadline = time.monotonic() + self.settings["settle_timeout_s"]
        while time.monotonic() < deadline:
            self._check_cancel()
            state = self.robot.read_state()
            self._check_tracking(state, target)
            if self._within(state.tool_pose, target) and np.max(np.abs(state.joint_velocities)) <= 0.05:
                return
            self.robot.command_pose(target)
            time.sleep(self.period)
        raise TimeoutError("Robot did not settle at the requested Cartesian target")

    def hold(self, target, duration_s):
        """Count dwell only while measured at the target; deviations fail the task."""
        started = time.monotonic()
        while True:
            self._check_cancel()
            tick = time.monotonic()
            state = self.robot.read_state()
            self._check_tracking(state, target)
            if (not self._within(state.tool_pose, target)
                    or np.max(np.abs(state.joint_velocities)) > 0.05):
                raise RuntimeError("Robot left the verified hover pose during dwell")
            if time.monotonic() - started >= duration_s:
                return
            self.robot.command_pose(target)
            time.sleep(max(0, self.period - (time.monotonic() - tick)))

    def hold_current(self):
        """Best-effort hold after operator cancellation; not an emergency stop."""
        self.robot.command_pose(self.robot.read_state().tool_pose)
