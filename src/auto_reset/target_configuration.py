"""Desired object placements, stored separately from measured physical poses.

Targets are T_buffer_object matrices keyed by the same persistent object_id
used by SceneObject. The data container and intended API are prepared here;
target editing and grid-cell-to-object pose resolution remain unimplemented.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from buffer import Buffer


@dataclass(kw_only=True)
class TargetConfiguration:
    """A requested arrangement in one named buffer coordinate frame."""

    buffer_id: str
    poses: dict[str, np.ndarray] = field(default_factory=dict, repr=False)

    def set_pose(self, object_id: str, pose: np.ndarray) -> None:
        """Set T_buffer_object; not implemented yet."""
        raise NotImplementedError("Target pose editing is not implemented")

    def set_cell(self, object_id: str, buffer: Buffer, column: int, row: int, yaw: float = 0) -> None:
        """Resolve an object's placement in a cell using its dimensions and yaw.

        Object dimensions must be supplied through the manager's object
        registry when this resolver is implemented; a cell center alone does
        not determine an object's resting origin or placement height.
        """
        raise NotImplementedError("Buffer-cell target resolution is not implemented")

    def remove(self, object_id: str) -> None:
        """Remove a desired placement; not implemented yet."""
        raise NotImplementedError("Target removal is not implemented")

    def pose_for(self, object_id: str) -> np.ndarray:
        """Return the requested T_buffer_object; not implemented yet."""
        raise NotImplementedError("Target pose lookup is not implemented")
