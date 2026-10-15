#!/usr/bin/env python3
"""Small GUI for jogging the DROID Panda arm and operating its gripper."""

from __future__ import annotations

import argparse
import math
import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

# Panda joint limits in radians, from Franka's FCI control parameters:
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
TARGET_TOLERANCE_RAD = math.radians(1.0)
GRIPPER_TOLERANCE = 0.15


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
    if joint not in range(7) or not math.isfinite(degrees) or not 0 < abs(degrees) <= MAX_STEP_DEG:
        raise ValueError("Select a joint and a nonzero step of at most 1 degree")
    target = checked_positions(state).copy()
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


def send_gripper_target(robot, position: float) -> None:
    """Position is normalized: 0 opens, 1 closes."""
    if position not in (0.0, 1.0):
        raise ValueError("Gripper target must be fully open or fully closed")
    robot.update_gripper(position, False, True)


def execute_job(robot, action: str, values: tuple) -> tuple[dict, str, int | None]:
    """Perform one GUI action and return the state to display."""
    if action == "initialize":
        robot.launch_controller()
        robot.launch_robot()
    elif action == "jog":
        joint, degrees = values
        before, _ = robot.get_robot_state()
        target = plan_jog(before, joint, degrees)
        send_joint_target(robot, target)
        after, _ = robot.get_robot_state()
        if not command_reached_target(after, target):
            raise RuntimeError("Joint command result is uncertain; read state before trying again")
    elif action == "gripper":
        position = values[0]
        before, _ = robot.get_robot_state()
        checked_positions(before)
        send_gripper_target(robot, position)
        after, _ = robot.get_robot_state()
        if abs(after["gripper_position"] - position) > GRIPPER_TOLERANCE:
            raise RuntimeError("Gripper target was not reached; read state before trying again")
    elif action != "refresh":
        raise ValueError(f"Unknown GUI action: {action}")

    state, _ = robot.get_robot_state()
    checked_positions(state)
    if action == "jog":
        message = f"J{values[0] + 1} moved {values[1]:+.1f}°."
        jogged_joint = values[0]
    elif action == "gripper":
        message = "Gripper opened." if values[0] == 0.0 else "Gripper closed."
        jogged_joint = None
    elif action == "initialize":
        message = "Controller initialized; robot state ready."
        jogged_joint = None
    else:
        message = "Robot state ready."
        jogged_joint = None
    return state, message, jogged_joint


class JointJoggerGUI:
    def __init__(self, root: tk.Tk, nuc_ip: str):
        self.root = root
        self.nuc_ip = nuc_ip
        self.jobs: queue.Queue[tuple] = queue.Queue()
        self.events: queue.Queue[tuple] = queue.Queue()
        self.ready = False
        self.busy = False

        root.title("DroidAuto joint jogger")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.close)
        frame = ttk.Frame(root, padding=16)
        frame.grid()

        ttk.Label(frame, text=f"DROID NUC: {nuc_ip}:4242").grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(frame, text="Drag a slider to choose a step; click Jog to move one joint.").grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(4, 12)
        )

        self.position_labels: list[ttk.Label] = []
        self.sliders: list[tk.Scale] = []
        self.jog_buttons: list[ttk.Button] = []
        for joint in range(7):
            row = joint + 2
            label = ttk.Label(frame, text=f"J{joint + 1}: --°", width=14)
            label.grid(row=row, column=0, sticky="w")
            self.position_labels.append(label)

            slider = tk.Scale(
                frame, from_=-MAX_STEP_DEG, to=MAX_STEP_DEG,
                orient=tk.HORIZONTAL, resolution=0.1, length=310,
                label="step in degrees", showvalue=True,
            )
            slider.set(0)
            slider.grid(row=row, column=1, columnspan=2, padx=8)
            self.sliders.append(slider)

            button = ttk.Button(frame, text=f"Jog J{joint + 1}", command=lambda j=joint: self.jog(j))
            button.grid(row=row, column=3, padx=(8, 0))
            self.jog_buttons.append(button)

        controls = ttk.Frame(frame)
        controls.grid(row=9, column=0, columnspan=4, sticky="ew", pady=(14, 6))
        self.refresh_button = ttk.Button(controls, text="Read state", command=lambda: self.submit("refresh"))
        self.refresh_button.pack(side="left", padx=(0, 8))
        self.init_button = ttk.Button(controls, text="Initialize controller", command=self.initialize)
        self.init_button.pack(side="left", padx=(0, 8))
        self.open_button = ttk.Button(controls, text="Open gripper", command=lambda: self.submit("gripper", 0.0))
        self.open_button.pack(side="left", padx=(0, 8))
        self.close_button = ttk.Button(controls, text="Close gripper", command=lambda: self.submit("gripper", 1.0))
        self.close_button.pack(side="left")

        self.gripper_label = ttk.Label(frame, text="Gripper: --")
        self.gripper_label.grid(row=10, column=0, columnspan=4, sticky="w")
        self.status = ttk.Label(frame, text="Connecting...", wraplength=600)
        self.status.grid(row=11, column=0, columnspan=4, sticky="w", pady=(8, 0))

        self.set_controls()
        threading.Thread(target=self.worker, daemon=True).start()
        self.submit("refresh")
        root.after(100, self.drain_events)

    def set_controls(self) -> None:
        for button in (self.refresh_button, self.init_button):
            button.configure(state="disabled" if self.busy else "normal")
        for button in (*self.jog_buttons, self.open_button, self.close_button):
            button.configure(state="normal" if self.ready and not self.busy else "disabled")

    def submit(self, action: str, *values) -> None:
        if self.busy:
            return
        self.busy = True
        self.status.configure(text={
            "refresh": "Reading robot state...",
            "initialize": "Initializing controller and robot interface...",
            "jog": "Sending joint command...",
            "gripper": "Sending gripper command...",
        }[action])
        self.set_controls()
        self.jobs.put((action, *values))

    def jog(self, joint: int) -> None:
        degrees = float(self.sliders[joint].get())
        if degrees == 0:
            self.status.configure(text="Set a nonzero step before clicking Jog.")
            return
        self.submit("jog", joint, degrees)

    def initialize(self) -> None:
        if messagebox.askyesno(
            "Initialize DROID controller",
            "This will start or restart the NUC robot controller. Continue?",
            parent=self.root,
        ):
            self.submit("initialize")

    def worker(self) -> None:
        robot = None
        try:
            import zerorpc

            robot = zerorpc.Client(heartbeat=20)
            robot.connect(f"tcp://{self.nuc_ip}:4242")
            while True:
                action, *values = self.jobs.get()
                if action == "quit":
                    return
                try:
                    state, message, jogged_joint = execute_job(robot, action, tuple(values))
                    self.events.put(("ok", state, message, jogged_joint))
                except Exception as exc:
                    self.events.put(("error", str(exc)))
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            if robot is not None:
                robot.close()

    def drain_events(self) -> None:
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            self.busy = False
            if event[0] == "ok":
                _, state, message, jogged_joint = event
                self.ready = True
                for joint, value in enumerate(state["joint_positions"]):
                    self.position_labels[joint].configure(text=f"J{joint + 1}: {math.degrees(value):+.2f}°")
                self.gripper_label.configure(text=f"Gripper: {state['gripper_position']:.2f} (0 open, 1 closed)")
                if jogged_joint is not None:
                    self.sliders[jogged_joint].set(0)
                self.status.configure(text=message)
            else:
                self.ready = False
                self.status.configure(text=f"Stopped: {event[1]}")
            self.set_controls()
        self.root.after(100, self.drain_events)

    def close(self) -> None:
        if self.busy:
            messagebox.showinfo("DroidAuto", "Wait for the current command to finish.", parent=self.root)
            return
        self.jobs.put(("quit",))
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nuc-ip", default="192.168.88.12")
    args = parser.parse_args()
    root = tk.Tk()
    JointJoggerGUI(root, args.nuc_ip)
    root.mainloop()


if __name__ == "__main__":
    main()
