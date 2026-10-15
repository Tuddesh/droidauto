#!/usr/bin/env python3
"""Interactive, single-step Panda joint jogger for the installed DROID server."""

from __future__ import annotations

import argparse
import math
import sys

# Panda position limits in radians, from Franka's FCI control parameters:
# https://support.franka.de/docs/control_parameters.html
PANDA_LIMITS = (
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)
MAX_STEP_DEG = 1.0
LIMIT_MARGIN_RAD = math.radians(5.0)
STILL_VELOCITY_RAD_S = 0.05
POSITION_CHANGE_RAD = math.radians(0.5)
TARGET_TOLERANCE_RAD = math.radians(1.0)


def parse_jog(text: str) -> tuple[int, float]:
    """Parse a one-based joint number and signed step in degrees."""
    parts = text.split()
    if len(parts) != 2:
        raise ValueError("Enter a joint and signed degrees, for example: 3 +0.5")
    try:
        joint = int(parts[0])
        degrees = float(parts[1])
    except ValueError as exc:
        raise ValueError("Joint must be 1–7 and step must be a number in degrees") from exc
    if joint not in range(1, 8):
        raise ValueError("Joint must be 1–7")
    if not math.isfinite(degrees) or not 0 < abs(degrees) <= MAX_STEP_DEG:
        raise ValueError(f"Step must be nonzero and at most {MAX_STEP_DEG:g} degree")
    return joint - 1, degrees


def checked_positions(state: dict) -> list[float]:
    positions = list(state["joint_positions"])
    velocities = list(state["joint_velocities"])
    if len(positions) != 7 or len(velocities) != 7:
        raise ValueError("Robot state must contain seven joint positions and velocities")
    if not all(math.isfinite(value) for value in positions + velocities):
        raise ValueError("Robot state contains a non-finite joint value")
    if any(abs(value) > STILL_VELOCITY_RAD_S for value in velocities):
        raise ValueError("Arm is moving; wait until all joints are still")
    for index, (value, (lower, upper)) in enumerate(zip(positions, PANDA_LIMITS), 1):
        if not lower <= value <= upper:
            raise ValueError(f"Joint {index} is outside the configured Panda limits")
    return positions


def plan_jog(state: dict, joint: int, degrees: float) -> list[float]:
    positions = checked_positions(state)
    target = positions.copy()
    target[joint] += math.radians(degrees)
    lower, upper = PANDA_LIMITS[joint]
    if not lower + LIMIT_MARGIN_RAD <= target[joint] <= upper - LIMIT_MARGIN_RAD:
        raise ValueError(f"Target for joint {joint + 1} is within 5 degrees of a joint limit")
    return target


def command_reached_target(state: dict, target: list[float]) -> bool:
    actual = list(state["joint_positions"])
    return (
        len(actual) == 7
        and bool(state["prev_command_successful"])
        and all(math.isfinite(value) for value in actual)
        and max(abs(value - desired) for value, desired in zip(actual, target))
        <= TARGET_TOLERANCE_RAD
    )


def send_joint_target(robot, target: list[float]) -> None:
    """Call the installed FrankaRobot RPC with a blocking position target."""
    robot.update_joints(target, False, True)


def show_state(state: dict) -> None:
    positions = checked_positions(state)
    print("Joints (degrees): " + "  ".join(
        f"J{index}={math.degrees(value):+.2f}"
        for index, value in enumerate(positions, 1)
    ))
    print(f"Gripper: {state['gripper_position']:.3f}; "
          f"previous command successful: {state['prev_command_successful']}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nuc-ip", default="192.168.88.12")
    parser.add_argument(
        "--initialize",
        action="store_true",
        help="explicitly start/restart the NUC controller and robot interface",
    )
    args = parser.parse_args()

    # The NUC runs DROID. This workstation needs only the ZeroRPC transport.
    import zerorpc

    print(f"Connecting to DROID at tcp://{args.nuc_ip}:4242")
    robot = zerorpc.Client(heartbeat=20)
    try:
        robot.connect(f"tcp://{args.nuc_ip}:4242")
        if args.initialize:
            print("Initializing the NUC controller and robot interface...")
            robot.launch_controller()
            robot.launch_robot()
        state, _ = robot.get_robot_state()
        show_state(state)
        print("Enter 'joint signed-degrees' (example: 3 +0.5), 'state', or 'q'.")
        print("Each jog is capped at 1 degree and requires typing 'move' to confirm.")

        while True:
            try:
                command = input("jog> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0
            if command.lower() in {"q", "quit", "exit"}:
                return 0
            if command.lower() == "state":
                state, _ = robot.get_robot_state()
                show_state(state)
                continue

            try:
                joint, degrees = parse_jog(command)
                state, _ = robot.get_robot_state()
                target = plan_jog(state, joint, degrees)
            except (ValueError, KeyError) as exc:
                print(f"No move: {exc}")
                continue

            print(f"J{joint + 1}: {math.degrees(state['joint_positions'][joint]):+.2f}° "
                  f"-> {math.degrees(target[joint]):+.2f}°")
            try:
                confirmation = input("Type 'move' to execute: ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nNo move.")
                return 0
            if confirmation != "move":
                print("No move.")
                continue

            # Recheck after the prompt so an intervening motion cancels the jog.
            latest, _ = robot.get_robot_state()
            try:
                latest_positions = checked_positions(latest)
            except (ValueError, KeyError) as exc:
                print(f"No move: {exc}")
                continue
            if max(abs(a - b) for a, b in zip(latest_positions, state["joint_positions"])) > POSITION_CHANGE_RAD:
                print("No move: joint positions changed during confirmation.")
                continue

            send_joint_target(robot, target)
            after, _ = robot.get_robot_state()
            show_state(after)
            if not command_reached_target(after, target):
                print("Command outcome is uncertain or target was not reached; stopping.", file=sys.stderr)
                return 1
    finally:
        robot.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Jogger stopped: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
