import io
import math
import time
import unittest
from contextlib import redirect_stdout
from unittest.mock import Mock, patch

import zerorpc

from scripts.joint_offset_move import run_move


class JointOffsetMoveTests(unittest.TestCase):
    def make_robot(self):
        robot = Mock(spec=["launch_controller", "launch_robot", "get_robot_state", "update_joints"])
        state = {
            "joint_positions": [0.1, -0.8, 0.2, -2.0, 0.3, 1.5, 0.4],
            "joint_velocities": [0.0] * 7,
            "prev_command_successful": True,
        }

        def get_robot_state():
            now = time.time()
            return state, {
                "robot_timestamp_seconds": int(now),
                "robot_timestamp_nanos": int((now % 1) * 1e9),
            }

        def update_joints(target, velocity, blocking):
            state["joint_positions"] = list(target)

        robot.get_robot_state.side_effect = get_robot_state
        robot.update_joints.side_effect = update_joints
        return robot, state

    def test_initializes_and_adds_offset_to_measurement_after_confirmation(self):
        robot, state = self.make_robot()
        original = state["joint_positions"].copy()

        def confirm(_):
            state["joint_positions"][0] += 0.02
            return ""

        with patch("builtins.input", side_effect=confirm), redirect_stdout(io.StringIO()):
            run_move(robot)

        robot.launch_controller.assert_called_once_with()
        robot.launch_robot.assert_called_once_with()
        robot.update_joints.assert_called_once()
        target, velocity, blocking = robot.update_joints.call_args.args
        for index, value in enumerate(original):
            self.assertAlmostEqual(target[index], value + math.pi / 8 + (0.02 if index == 0 else 0))
        self.assertEqual((velocity, blocking), (False, True))

    def test_cancelling_confirmation_sends_no_motion(self):
        robot, _ = self.make_robot()
        with patch("builtins.input", side_effect=KeyboardInterrupt), redirect_stdout(io.StringIO()):
            with self.assertRaises(KeyboardInterrupt):
                run_move(robot)
        robot.update_joints.assert_not_called()

    def test_first_startup_failure_retries_both_launch_calls_and_moves_once(self):
        robot, _ = self.make_robot()
        robot.launch_robot.side_effect = [
            zerorpc.exceptions.RemoteError("_InactiveRpcError", "Robot context not valid", None),
            None,
        ]
        with patch("builtins.input", return_value=""), redirect_stdout(io.StringIO()) as output:
            run_move(robot)
        self.assertEqual(robot.launch_controller.call_count, 2)
        self.assertEqual(robot.launch_robot.call_count, 2)
        robot.update_joints.assert_called_once()
        self.assertIn("relaunch attempt 1/5 failed: _InactiveRpcError", output.getvalue())

    def test_five_startup_failures_stop_before_any_state_read_or_move(self):
        robot, _ = self.make_robot()
        failure = zerorpc.exceptions.RemoteError("_InactiveRpcError", "Robot context not valid", None)
        robot.launch_robot.side_effect = failure
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, "after 5 attempts") as caught:
            run_move(robot)
        self.assertEqual(robot.launch_controller.call_count, 5)
        self.assertEqual(robot.launch_robot.call_count, 5)
        self.assertIs(caught.exception.__cause__, failure)
        robot.get_robot_state.assert_not_called()
        robot.update_joints.assert_not_called()

    def test_motion_failure_is_not_retried(self):
        robot, _ = self.make_robot()
        robot.update_joints.side_effect = zerorpc.exceptions.RemoteError("RpcError", "Move failed", None)
        with patch("builtins.input", return_value=""), redirect_stdout(io.StringIO()):
            with self.assertRaises(zerorpc.exceptions.RemoteError):
                run_move(robot)
        robot.launch_controller.assert_called_once()
        robot.launch_robot.assert_called_once()
        robot.update_joints.assert_called_once()

    def test_advancing_but_old_feedback_sends_no_motion(self):
        robot, _ = self.make_robot()
        get_state = robot.get_robot_state.side_effect

        def old_state():
            state, stamp = get_state()
            stamp["robot_timestamp_seconds"] -= 1
            return state, stamp

        robot.get_robot_state.side_effect = old_state
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, "stale"):
            run_move(robot)
        robot.update_joints.assert_not_called()

    def test_frozen_feedback_sends_no_motion(self):
        robot, state = self.make_robot()
        robot.get_robot_state.side_effect = None
        robot.get_robot_state.return_value = (
            state, {"robot_timestamp_seconds": int(time.time()), "robot_timestamp_nanos": 0}
        )
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(RuntimeError, "not advancing"):
            run_move(robot)
        robot.update_joints.assert_not_called()

    def test_server_ignoring_target_is_reported_as_failure(self):
        robot, _ = self.make_robot()
        robot.update_joints.side_effect = None
        with patch("builtins.input", return_value=""), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "did not confirm"):
                run_move(robot)


if __name__ == "__main__":
    unittest.main()
