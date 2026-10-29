"""A buffer fixture with a planar ArUco marker and shared physical identity.

Marker dimensions describe the printed code, independently of the fixture's
physical dimensions and cell footprint inherited through BufferItem.
"""

import math

from ..buffer_item import BufferItem


class ArucoTile(BufferItem):
    """A marker fixed at the center of a cell, parallel to the buffer plane.

    ``marker_size_mm`` is the width of the actual black marker square,
    including its black border but excluding the white quiet zone.
    """

    def __init__(
        self,
        marker_id: int,
        marker_size_mm: float = 32.0,
        dictionary: str = "DICT_4X4_50",
        *,
        object_id: str | None = None,
    ) -> None:
        super().__init__(object_id=object_id)
        if isinstance(marker_id, bool) or not isinstance(marker_id, int) or marker_id < 0:
            raise ValueError("marker_id must be a non-negative integer")
        if not math.isfinite(marker_size_mm) or marker_size_mm <= 0:
            raise ValueError("marker_size_mm must be finite and positive")
        self.marker_id = marker_id
        self.marker_size_mm = float(marker_size_mm)
        self.dictionary = dictionary
