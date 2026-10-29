"""Gridfinity footprints sharing the common PhysicalObject identity and pose.

BufferItem adds cell dimensions to physical data inherited from scene. The
legacy item_id property aliases object_id so existing layout callers retain
one identity; it no longer maintains a separate process-local counter.
"""

from scene.physical_object import PhysicalObject


class BufferItem(PhysicalObject):
    """An item occupying a rectangular number of Gridfinity cells.

    Subclasses describe the physical item. Identity and observed physical state
    come from PhysicalObject; cell placement belongs to Buffer.
    """

    def __init__(self, width_cells: int = 1, height_cells: int = 1, *,
                 object_id: str | None = None,
                 dimensions_m: tuple[float, float, float] | None = None) -> None:
        if isinstance(width_cells, bool) or not isinstance(width_cells, int) or width_cells < 1:
            raise ValueError("width_cells must be a positive integer")
        if isinstance(height_cells, bool) or not isinstance(height_cells, int) or height_cells < 1:
            raise ValueError("height_cells must be a positive integer")
        identity = {} if object_id is None else {"object_id": object_id}
        super().__init__(dimensions_m=dimensions_m, **identity)
        self.width_cells = width_cells
        self.height_cells = height_cells

    @property
    def item_id(self) -> str:
        """Compatibility alias for the inherited physical object identity."""
        return self.object_id
