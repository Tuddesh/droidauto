"""Gridfinity buffers and fixtures built on the shared scene.PhysicalObject.

The public buffer classes retain their layout and localization APIs; reset
planning and task-object manipulation belong to auto_reset's prepared core.
"""

from .buffer import Buffer, LocalizationError
from .buffer_item import BufferItem

__all__ = ["Buffer", "BufferItem", "LocalizationError"]
