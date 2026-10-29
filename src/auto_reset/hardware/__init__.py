"""Existing concrete DROID and ZED adapters, separate from reset orchestration.

Importing these classes does not connect hardware. Driver abstractions will be
introduced if an actual second backend requires them.
"""

from .droid import DroidRobot, RobotState
from .zed import CameraFrame, ZedWristCamera

__all__ = ["DroidRobot", "RobotState", "ZedWristCamera", "CameraFrame"]
