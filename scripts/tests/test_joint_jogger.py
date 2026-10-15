import math
import unittest

from scripts.joint_jogger import (
    PANDA_LIMITS,
    command_reached_target,
    parse_jog,
    plan_jog,
)


def state(positions=None, velocities=None):
    return {
        "joint_positions": positions or [0, 0, 0, -1.5, 0, 1.5, 0],
        "joint_velocities": velocities or [0] * 7,
    }


class JointJoggerTests(unittest.TestCase):
    def test_valid_step_changes_only_selected_joint(self):
        joint, degrees = parse_jog("3 +0.5")
        target = plan_jog(state(), joint, degrees)
        self.assertEqual(joint, 2)
        self.assertAlmostEqual(target[2], math.radians(0.5))
        self.assertEqual(target[:2] + target[3:], state()["joint_positions"][:2] + state()["joint_positions"][3:])

    def test_rejects_oversize_and_nonfinite_steps(self):
        for command in ("1 +1.01", "2 nan", "3 inf", "4 0", "8 +0.5"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                parse_jog(command)

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


if __name__ == "__main__":
    unittest.main()
