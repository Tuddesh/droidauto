"""Base type for physical objects placed in the Gridfinity buffer."""

from itertools import count


class BufferItem:
    """An item occupying a rectangular number of Gridfinity cells.

    Subclasses describe the physical item. Each instance gets a unique,
    process-local ``item_id``; placement belongs to ``Buffer``.
    """

    _next_item_id = count(1)

    def __init__(self, width_cells: int = 1, height_cells: int = 1) -> None:
        if isinstance(width_cells, bool) or not isinstance(width_cells, int) or width_cells < 1:
            raise ValueError("width_cells must be a positive integer")
        if isinstance(height_cells, bool) or not isinstance(height_cells, int) or height_cells < 1:
            raise ValueError("height_cells must be a positive integer")
        self._item_id = next(BufferItem._next_item_id)
        self.width_cells = width_cells
        self.height_cells = height_cells

    @property
    def item_id(self) -> int:
        """Unique ID for this item instance within the current Python process."""
        return self._item_id
