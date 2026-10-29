"""Physical scene models shared by buffers, fixtures and manipulation objects.

This package contains physical data, supplied observations and grasp geometry. Importing it
does not open cameras, connect to robots or run a reset.
"""

from .physical_object import PhysicalObject
from .scene_object import Cube, SceneObject

__all__ = ["PhysicalObject", "SceneObject", "Cube"]
