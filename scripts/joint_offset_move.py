#!/usr/bin/env python3
"""Initialize DROID, then move every joint by +pi/8 radians after confirmation."""

from __future__ import annotations

import argparse
import math
import time

import zerorpc

if __package__:
    from .droid_rpc import MAX_STATE_AGE_S, RPC_TIMEOUT_S, relaunch
else:
    from droid_rpc import MAX_STATE_AGE_S, RPC_TIMEOUT_S, relaunch


def timestamp_seconds(stamp: dict) -> float:
    return float(stamp["robot_timestamp_seconds"]) + float(stamp["robot_timestamp_nanos"]) * 1e-9


def read_live_state(robot) -> dict:
    """Require advancing, recent feedback and a stationary arm before proceeding."""
    _, first_stamp = robot.get_robot_state()
    time.sleep(0.15)
    state, second_stamp = robot.get_robot_state()
    first_time = timestamp_seconds(first_stamp)
    second_time = timestamp_seconds(second_stamp)
    if not math.isfinite(first_time) or not math.isfinite(second_time) or second_time <= first_time:
        raise RuntimeError("Robot timestamp is not advancing")
    if abs(time.time() - second_time) > MAX_STATE_AGE_S:
        raise RuntimeError("Robot feedback is stale or the NUC and PC clocks differ")
    positions = list(state["joint_positions"])
    velocities = list(state["joint_velocities"])
    if len(positions) != 7 or len(velocities) != 7:
        raise RuntimeError("Expected seven measured joint positions and velocities")
    if not all(math.isfinite(value) for value in positions + velocities):
        raise RuntimeError("Robot feedback contains non-finite joint values")
    if any(abs(value) > 0.05 for value in velocities):
        raise RuntimeError("Arm is moving; wait for it to stop before running this script")
    return state


def show_positions(label: str, positions: list[float]) -> None:
    print(f"{label} (rad): {[round(value, 6) for value in positions]}", flush=True)
    print(f"{label} (deg): {[round(math.degrees(value), 2) for value in positions]}", flush=True)


def run_move(robot) -> None:
    print("Initializing DROID arm and gripper controllers...", flush=True)
    relaunch(robot)

    state = read_live_state(robot)
    positions = list(state["joint_positions"])
    show_positions("Measured joints", positions)
    show_positions("Target preview", [value + math.pi / 8 for value in positions])
    input("This moves ALL SEVEN joints by +22.5 degrees. Press Enter to move; Ctrl+C to cancel: ")

    # Apply the requested offset to a new measurement, rather than to feedback
    # that may have become old while the operator was reading the preview.
    state = read_live_state(robot)
    target = [value + math.pi / 8 for value in state["joint_positions"]]
    show_positions("Sending target", target)
    robot.update_joints(target, False, True)  # position mode, blocking

    after = read_live_state(robot)
    show_positions("Measured after move", list(after["joint_positions"]))
    error = max(abs(actual - desired) for actual, desired in zip(after["joint_positions"], target))
    print(f"Maximum target error: {math.degrees(error):.3f} degrees", flush=True)
    if not after["prev_command_successful"] or error > math.radians(1):
        raise RuntimeError("The server did not confirm the requested target was reached")
    print("Move complete.", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nuc-ip", default="192.168.88.12")
    args = parser.parse_args()
    robot = zerorpc.Client(timeout=RPC_TIMEOUT_S, heartbeat=20)
    robot.connect(f"tcp://{args.nuc_ip}:4242")
    try:
        run_move(robot)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
