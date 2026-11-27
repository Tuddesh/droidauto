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
from buffer.buffer_item import BufferItem

if TYPE_CHECKING:
    from buffer import Buffer


@dataclass(kw_only=True)
class TargetConfiguration:
    """A requested arrangement in one named buffer coordinate frame."""

    buffer_id: str
    poses: dict[str, np.ndarray] = field(default_factory=dict, repr=False)
    cells: dict[str, set[tuple[int, int]]] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self):
        self.poses = {key: transform(value) for key, value in self.poses.items()}

    def set_pose(self, object_id: str, pose: np.ndarray) -> None:
        """Store a copy of T_buffer_object."""
        if not isinstance(object_id, str) or not object_id:
            raise ValueError("Target needs an object_id")
        self.poses[object_id] = transform(pose)
        self.cells.pop(object_id, None)

    def set_cell(self, object_id: str, buffer: Buffer, column: int, row: int,
                 yaw: float = 0.0, *, scene_object: SceneObject,
                 support_height_m: float, footprint: BufferItem | None = None) -> None:
        """Center an object above a free cell or an explicit rectangular reservation.

        support_height_m is the measured cell floor Z relative to the marker
        plane. The object's support_z_m determines its origin height. Default
        placements retain the one-cell dimension check. An explicit BufferItem
        footprint reserves cells independently of physical overhang.
        """
        if buffer.object_id != self.buffer_id or scene_object.object_id != object_id:
            raise ValueError("Target, buffer and object identities must match")
        if any(isinstance(v, bool) or not isinstance(v, int) for v in (column, row)):
            raise ValueError("Cell indices must be integers")
        if not math.isfinite(yaw) or not math.isfinite(support_height_m):
            raise ValueError("Yaw and support height must be finite")
        dimensions = scene_object.dimensions_m
        if dimensions is not None:
            dimensions = np.asarray(dimensions, dtype=float)
            if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
                raise ValueError("Object dimensions must be three positive finite lengths")
            if footprint is None:
                c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
                extent = np.array([[c, s], [s, c]]) @ dimensions[:2]
                if np.any(extent > buffer.cell_size_mm / 1000 + 1e-9):
                    raise ValueError("Rotated object footprint does not fit in one cell")
        if footprint is None:
            footprint = BufferItem(object_id=object_id)
        if not isinstance(footprint, BufferItem):
            raise TypeError("footprint must be a BufferItem")
        cells = {(x, y) for y in range(row, row + footprint.height_cells)
                 for x in range(column, column + footprint.width_cells)}
        for x, y in cells:
            if buffer.item_at(x, y) is not None:
                raise ValueError("Target region contains a buffer fixture")
        if any(cells & occupied for key, occupied in self.cells.items() if key != object_id):
            raise ValueError("Target regions overlap")
        support_z = scene_object.support_z_m
        if support_z is None:
            support_z = 0.0 if dimensions is None else -dimensions[2] / 2
        if not math.isfinite(support_z):
            raise ValueError("Object support_z_m must be finite")
        center = buffer.cell_center_m(column, row)
        center[:2] += (np.array([footprint.width_cells, footprint.height_cells]) - 1) * buffer.cell_size_mm / 2000
        center[2] = support_height_m - support_z
        self.set_pose(object_id, pose_to_transform([*center, 0.0, 0.0, yaw]))
        self.cells[object_id] = cells

    def remove(self, object_id: str) -> None:
        """Remove one requested placement; unknown IDs raise KeyError."""
        del self.poses[object_id]
        self.cells.pop(object_id, None)

    def pose_for(self, object_id: str) -> np.ndarray:
        """Return a copy of the requested T_buffer_object."""
        return self.poses[object_id].copy()
