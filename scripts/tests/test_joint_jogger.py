import math
import time
import unittest
from unittest.mock import Mock

import zerorpc

from scripts.joint_jogger import (
    PANDA_LIMITS,
    command_reached_target,
    execute_job,
    plan_jog,
    read_fresh_state,
    send_gripper_target,
    send_joint_target,
)


def state(positions=None, velocities=None):
    return {
        "joint_positions": positions or [0, 0, 0, -1.5, 0, 1.5, 0],
        "joint_velocities": velocities or [0] * 7,
        "gripper_position": 0.0,
        "prev_command_successful": True,
    }


class FakeRobot:
    def __init__(self):
        self.current = state()
        self.calls = []

    def launch_controller(self):
        self.calls.append("launch_controller")

    def launch_robot(self):
        self.calls.append("launch_robot")

    def get_robot_state(self):
        self.calls.append("get_robot_state")
        now = time.time()
        seconds = int(now)
        return self.current, {
            "robot_timestamp_seconds": seconds,
            "robot_timestamp_nanos": int((now - seconds) * 1e9),
        }

    def update_joints(self, target, velocity, blocking):
        self.calls.append(("update_joints", velocity, blocking))
        self.current["joint_positions"] = list(target)

    def update_gripper(self, position, velocity, blocking):
        self.calls.append(("update_gripper", velocity, blocking))
        self.current["gripper_position"] = position


class JointJoggerTests(unittest.TestCase):
    def test_valid_step_changes_only_selected_joint(self):
        target = plan_jog(state(), 2, 0.5)
        self.assertAlmostEqual(target[2], math.radians(0.5))
        self.assertEqual(target[:2] + target[3:], state()["joint_positions"][:2] + state()["joint_positions"][3:])

    def test_rejects_oversize_and_nonfinite_steps(self):
        for joint, degrees in ((0, 1.01), (1, math.nan), (2, math.inf), (3, 0), (7, 0.5)):
            with self.subTest(joint=joint, degrees=degrees), self.assertRaises(ValueError):
                plan_jog(state(), joint, degrees)

    def test_rejects_target_near_joint_limit(self):
        positions = state()["joint_positions"]
        positions[0] = PANDA_LIMITS[0][1] - math.radians(5.1)
        with self.assertRaisesRegex(ValueError, "within 5 degrees"):
            plan_jog(state(positions=positions), 0, 0.5)

    def test_rejects_motion_and_invalid_state(self):
        with self.assertRaisesRegex(ValueError, "Arm is moving"):
            plan_jog(state(velocities=[0.1] + [0] * 6), 0, 0.5)
        with self.assertRaisesRegex(ValueError, "seven joint"):
            plan_jog(state(positions=[0] * 6), 0, 0.5)

    def test_outcome_checks_every_joint_and_server_status(self):
        target = state()["joint_positions"]
        response = state()
        response["prev_command_successful"] = True
        self.assertTrue(command_reached_target(response, target))
        response["joint_positions"][6] += math.radians(2)
        self.assertFalse(command_reached_target(response, target))
        response["joint_positions"][6] = 0
        response["prev_command_successful"] = False
        self.assertFalse(command_reached_target(response, target))

    def test_rpc_sends_plain_joint_list_as_blocking_position(self):
        class FakeRobot:
            def update_joints(self, *args):
                self.args = args

        robot = FakeRobot()
        target = state()["joint_positions"]
        send_joint_target(robot, target)
        self.assertEqual(robot.args, (target, False, True))

    def test_gripper_buttons_send_blocking_position_commands(self):
        class FakeRobot:
            def update_gripper(self, *args):
                self.args = args

        robot = FakeRobot()
        send_gripper_target(robot, 0.0)
        self.assertEqual(robot.args, (0.0, False, True))
        send_gripper_target(robot, 1.0)
        self.assertEqual(robot.args, (1.0, False, True))
        with self.assertRaises(ValueError):
            send_gripper_target(robot, 0.5)

    def test_refresh_with_no_action_values(self):
        robot = FakeRobot()
        reported, message, joint = execute_job(robot, "refresh", ())
        self.assertIs(reported, robot.current)
        self.assertEqual(message, "Robot state ready.")
        self.assertIsNone(joint)
        self.assertNotIn("update_joints", robot.calls)

    def test_relaunch_retries_full_startup_before_returning_ready_state(self):
        robot = FakeRobot()
        robot.launch_robot = Mock(side_effect=[
            zerorpc.exceptions.RemoteError("_InactiveRpcError", "Robot context not valid", None),
            None,
        ])
        reported, message, joint = execute_job(robot, "relaunch", ())
        self.assertEqual(robot.calls.count("launch_controller"), 2)
        self.assertEqual(robot.launch_robot.call_count, 2)
        self.assertIs(reported, robot.current)
        self.assertEqual(message, "Controller relaunched; live robot state ready.")
        self.assertIsNone(joint)
        self.assertFalse(any(isinstance(call, tuple) for call in robot.calls))

    def test_stale_or_frozen_state_disables_motion(self):
        class FrozenRobot(FakeRobot):
            def __init__(self, age):
                super().__init__()
                now = time.time() - age
                seconds = int(now)
                self.stamp = {
                    "robot_timestamp_seconds": seconds,
                    "robot_timestamp_nanos": int((now - seconds) * 1e9),
                }

            def get_robot_state(self):
                return self.current, self.stamp

        for age, error in ((60, "stale"), (0, "did not advance")):
            with self.subTest(age=age):
                robot = FrozenRobot(age)
                with self.assertRaisesRegex(ValueError, error):
                    execute_job(robot, "jog", (0, 0.5))
                self.assertNotIn(("update_joints", False, True), robot.calls)
                self.assertNotIn(("update_gripper", False, True), robot.calls)

    def test_missing_timestamp_disables_motion(self):
        robot = FakeRobot()
        robot.get_robot_state = lambda: (robot.current, {})
        with self.assertRaisesRegex(ValueError, "no valid timestamp"):
            read_fresh_state(robot)

    def test_jog_and_gripper_gui_actions(self):
        robot = FakeRobot()
        _, message, joint = execute_job(robot, "jog", (2, 0.5))
        self.assertEqual((message, joint), ("J3 moved +0.5°.", 2))
        self.assertAlmostEqual(robot.current["joint_positions"][2], math.radians(0.5))
        self.assertIn(("update_joints", False, True), robot.calls)

        _, message, joint = execute_job(robot, "gripper", (1.0,))
        self.assertEqual((message, joint), ("Gripper closed.", None))
        self.assertIn(("update_gripper", False, True), robot.calls)
        _, message, _ = execute_job(robot, "gripper", (0.0,))
        self.assertEqual(message, "Gripper opened.")


if __name__ == "__main__":
    unittest.main()
