"""DROID connection settings and Patrik's controller startup retry sequence."""

import time

import zerorpc

RPC_TIMEOUT_S = 60
MAX_STATE_AGE_S = 0.5


def relaunch(robot, attempts: int = 5) -> None:
    """Retry the full startup pair, as DROID's ServerInterface(launch=True) does."""
    if attempts < 1:
        raise ValueError("At least one relaunch attempt is required")
    for attempt in range(1, attempts + 1):
        try:
            robot.launch_controller()
            robot.launch_robot()
            return
        except zerorpc.exceptions.RemoteError as exc:
            print(f"relaunch attempt {attempt}/{attempts} failed: {exc.name}", flush=True)
            if attempt == attempts:
                raise RuntimeError(f"Robot server relaunch failed after {attempts} attempts: {exc}") from exc
            time.sleep(0.1)
