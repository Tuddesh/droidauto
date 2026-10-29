"""Requested object placements in buffer coordinates, separate from observations.

Cell placement uses object dimensions and an explicitly supplied support-plane
height relative to the buffer marker plane. This module performs no localization.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING

import numpy as np

from scene import SceneObject
from scene.transforms import pose_to_transform, transform

if TYPE_CHECKING:
    from buffer import Buffer


@dataclass(kw_only=True)
class TargetConfiguration:
    """A requested arrangement in one named buffer coordinate frame."""

    buffer_id: str
    poses: dict[str, np.ndarray] = field(default_factory=dict, repr=False)

    def __post_init__(self):
        self.poses = {key: transform(value) for key, value in self.poses.items()}

    def set_pose(self, object_id: str, pose: np.ndarray) -> None:
        """Store a copy of T_buffer_object."""
        if not isinstance(object_id, str) or not object_id:
            raise ValueError("Target needs an object_id")
        self.poses[object_id] = transform(pose)

    def set_cell(self, object_id: str, buffer: Buffer, column: int, row: int,
                 yaw: float = 0.0, *, scene_object: SceneObject,
                 support_height_m: float) -> None:
        """Place the geometric center above a free cell's supplied support plane.

        support_height_m is the measured cell floor Z relative to the marker
        plane; these planes are not assumed to coincide. Rotated object extents
        must fit inside one cell. Multi-cell placement is not supported.
        """
        if buffer.object_id != self.buffer_id or scene_object.object_id != object_id:
            raise ValueError("Target, buffer and object identities must match")
        if any(isinstance(v, bool) or not isinstance(v, int) for v in (column, row)):
            raise ValueError("Cell indices must be integers")
        if not math.isfinite(yaw) or not math.isfinite(support_height_m):
            raise ValueError("Yaw and support height must be finite")
        if buffer.item_at(column, row) is not None:
            raise ValueError("Target cell contains a buffer fixture")
        dimensions = np.asarray(scene_object.dimensions_m, dtype=float)
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise ValueError("Cell placement requires three positive object dimensions")
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        extent = np.array([[c, s], [s, c]]) @ dimensions[:2]
        if np.any(extent > buffer.cell_size_mm / 1000 + 1e-9):
            raise ValueError("Rotated object footprint does not fit in one cell")
        center = buffer.cell_center_m(column, row)
        center[2] = support_height_m + dimensions[2] / 2
        self.set_pose(object_id, pose_to_transform([*center, 0.0, 0.0, yaw]))

    def remove(self, object_id: str) -> None:
        """Remove one requested placement; unknown IDs raise KeyError."""
        del self.poses[object_id]

    def pose_for(self, object_id: str) -> np.ndarray:
        """Return a copy of the requested T_buffer_object."""
        return self.poses[object_id].copy()
