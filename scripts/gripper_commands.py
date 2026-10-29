#!/usr/bin/env python3
"""Interactive, line-based manual test of the existing DROID gripper backend.

Open/close are sent only in response to an explicit menu choice. There is no
automatic cycling, screen clearing, camera or arm movement command. Controller
relaunch is optional and explicitly affects both arm and gripper controllers.
Full errors and command history are saved under ignored runs/gripper_commands/.
"""

import argparse
from datetime import datetime, timezone
import json
import logging
from pathlib import Path

from auto_reset.hardware import DroidRobot


ROOT = Path(__file__).resolve().parents[1]
MENU = """
  1 / o   Open gripper
  2 / c   Close gripper
  3 / s   Read reported gripper position (0 = open, 1 = closed)
  h       Show this menu
  0 / q   Quit
"""


def report_error(exc, log, log_path):
    """Keep terminal errors brief; retain the complete traceback in the log."""
    log.exception("Operation failed")
    name = getattr(exc, "name", type(exc).__name__)
    detail = "" if hasattr(exc, "name") else f": {exc}"
    print(f"Failed — {name}{detail}\nFull error saved to: {log_path}", flush=True)


def run_menu(robot, log, log_path):
    """Wait for Enter after each choice; run one command and return to the prompt."""
    print(MENU, flush=True)
    while True:
        try:
            choice = input("Gripper [1=open, 2=close, 3=state, q=quit]> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.", flush=True)
            return 0
        if choice in ("q", "quit", "0", "exit"):
            print("Exiting.", flush=True)
            return 0
        if not choice:
            continue
        if choice in ("h", "help", "?"):
            print(MENU, flush=True)
            continue
        actions = {"1": "open", "o": "open", "open": "open",
                   "2": "close", "c": "close", "close": "close",
                   "3": "state", "s": "state", "state": "state"}
        action = actions.get(choice)
        if action is None:
            print("Choose 1 (open), 2 (close), 3 (state), h (help) or q (quit).", flush=True)
            continue
        try:
            log.info("Requested: %s", action)
            if action == "state":
                print("Reading gripper feedback...", flush=True)
                position = robot.read_gripper_position()
                print(f"Reported position: {position:.3f}  (0 = open, 1 = closed)", flush=True)
                log.info("Reported gripper position: %.6f", position)
            else:
                print(f"Sending {action.upper()} — waiting for DROID (RPC timeout: 60 s)...", flush=True)
                if action == "open":
                    robot.open_gripper()
                else:
                    robot.close_gripper()
                print(f"{action.upper()} command returned. Press 3 to read reported position.", flush=True)
                log.info("%s command returned", action)
        except KeyboardInterrupt:
            log.exception("Interrupted during an RPC; outcome unknown")
            print("\nInterrupted during a request; its outcome is unknown. Exiting.", flush=True)
            return 130
        except Exception as exc:
            report_error(exc, log, log_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = ROOT / "configs/station.local.json"
    parser.add_argument("--config", type=Path,
                        default=default if default.exists() else ROOT / "configs/station.example.json")
    parser.add_argument("--endpoint", help="Override the configured DROID tcp://HOST:PORT endpoint")
    parser.add_argument("--initialize-controllers", action="store_true",
                        help="Relaunch BOTH arm and gripper controllers once before the menu")
    args = parser.parse_args()
    try:
        endpoint = args.endpoint or json.loads(args.config.read_text())["robot_endpoint"]
    except (OSError, ValueError, KeyError) as exc:
        parser.error(f"Cannot read DROID endpoint: {exc}")
    directory = ROOT / "runs/gripper_commands"
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    log_path = directory / f"gripper_{stamp}.log"
    log = logging.getLogger("droidauto.gripper_commands")
    log.setLevel(logging.INFO)
    log.propagate = False
    handler = logging.FileHandler(log_path, mode="x")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    log.addHandler(handler)
    print(f"DroidAuto gripper test\nTarget: {endpoint}\nLog: {log_path}", flush=True)
    log.info("Target: %s", endpoint)
    try:
        with DroidRobot(endpoint) as robot:
            if args.initialize_controllers:
                print("Initializing BOTH arm and gripper controllers...", flush=True)
                log.info("Controller initialization requested")
                robot.initialize()
            return run_menu(robot, log, log_path)
    except KeyboardInterrupt:
        log.exception("Startup interrupted")
        print("\nStartup interrupted. Exiting.", flush=True)
        return 130
    except Exception as exc:
        report_error(exc, log, log_path)
        return 1
    finally:
        log.removeHandler(handler)
        handler.close()


if __name__ == "__main__":
    raise SystemExit(main())
