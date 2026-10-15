"""Gridfinity buffer layout and ArUco-based pose estimation."""

from __future__ import annotations

import cv2
import numpy as np

from .buffer_item import BufferItem
from .items.aruco_tile import ArucoTile


class LocalizationError(RuntimeError):
    """The buffer pose could not be estimated from an image."""


class Buffer:
    """A rectangular Gridfinity buffer with known ArUco tile placements.

    Cell ``(column, row)`` is indexed from the physical lower-left corner as
    viewed from above with the marker tops facing the viewer. +X goes right,
    +Y goes toward the far/top edge, and +Z points out of the marker faces.
    ``origin`` selects which cell's lower-left corner is coordinate (0, 0, 0).
    The marker top plane is Z=0. All physical sizes are specified in mm; pose
    translation returned by ``localize`` is in metres.
    """

    def __init__(
        self,
        columns: int,
        rows: int,
        camera_matrix: np.ndarray,
        distortion_coefficients: np.ndarray | None = None,
        cell_size_mm: float = 42.0,
        origin: tuple[int, int] = (0, 0),
        dictionary: str = "DICT_4X4_50",
    ) -> None:
        if isinstance(columns, bool) or not isinstance(columns, int) or columns < 1:
            raise ValueError("columns must be a positive integer")
        if isinstance(rows, bool) or not isinstance(rows, int) or rows < 1:
            raise ValueError("rows must be a positive integer")
        if not np.isfinite(cell_size_mm) or cell_size_mm <= 0:
            raise ValueError("cell_size_mm must be finite and positive")
        if (
            len(origin) != 2
            or any(isinstance(value, bool) or not isinstance(value, int) for value in origin)
            or not (0 <= origin[0] < columns and 0 <= origin[1] < rows)
        ):
            raise ValueError("origin must name a cell within the buffer")
        if not hasattr(cv2.aruco, dictionary):
            raise ValueError(f"Unknown ArUco dictionary: {dictionary}")

        camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        if camera_matrix.shape != (3, 3) or not np.isfinite(camera_matrix).all():
            raise ValueError("camera_matrix must be a finite 3x3 array")
        if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
            raise ValueError("camera focal lengths must be positive")
        if distortion_coefficients is None:
            distortion_coefficients = np.zeros(5, dtype=np.float64)
        distortion_coefficients = np.asarray(
            distortion_coefficients, dtype=np.float64
        ).reshape(-1)
        if distortion_coefficients.size not in (4, 5, 8, 12, 14) or not np.isfinite(
            distortion_coefficients
        ).all():
            raise ValueError("distortion_coefficients must contain 4, 5, 8, 12, or 14 finite values")

        self.columns = columns
        self.rows = rows
        self.cell_size_mm = float(cell_size_mm)
        self.origin = origin
        self.dictionary_name = dictionary
        self.camera_matrix = camera_matrix.copy()
        self.distortion_coefficients = distortion_coefficients.copy()
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary)
        )
        self._cells: dict[tuple[int, int], BufferItem] = {}
        self._placed_item_ids: set[int] = set()

    def place(self, item: BufferItem, column: int, row: int) -> None:
        """Record an item's physical placement, starting at its lower-left cell.

        Every cell in a multi-cell footprint must be within the buffer and
        empty. An item can be placed only once in this buffer. This changes
        only the layout model; it does not command a robot.
        """
        if not isinstance(item, BufferItem):
            raise TypeError("item must be a BufferItem")
        if isinstance(column, bool) or not isinstance(column, int):
            raise ValueError("column must be an integer")
        if isinstance(row, bool) or not isinstance(row, int):
            raise ValueError("row must be an integer")
        if item.item_id in self._placed_item_ids:
            raise ValueError(f"item {item.item_id} is already placed in the buffer")

        footprint = [
            (x, y)
            for y in range(row, row + item.height_cells)
            for x in range(column, column + item.width_cells)
        ]
        if any(not (0 <= x < self.columns and 0 <= y < self.rows) for x, y in footprint):
            raise ValueError("item footprint extends outside the buffer")
        if any(cell in self._cells for cell in footprint):
            raise ValueError("item footprint overlaps an occupied cell")
        for cell in footprint:
            self._cells[cell] = item
        self._placed_item_ids.add(item.item_id)

    def item_at(self, column: int, row: int) -> BufferItem | None:
        """Return the item occupying a cell, or ``None`` if the cell is empty."""
        if not (0 <= column < self.columns and 0 <= row < self.rows):
            raise IndexError("cell is outside the buffer")
        return self._cells.get((column, row))

    def cell_center_m(self, column: int, row: int) -> np.ndarray:
        """Return a cell center on the marker plane in buffer coordinates."""
        if not (0 <= column < self.columns and 0 <= row < self.rows):
            raise IndexError("cell is outside the buffer")
        return np.array(
            [
                (column - self.origin[0] + 0.5) * self.cell_size_mm / 1000,
                (row - self.origin[1] + 0.5) * self.cell_size_mm / 1000,
                0.0,
            ],
            dtype=np.float64,
        )

    def localize(self, image_rgb: np.ndarray) -> np.ndarray:
        """Return the 4x4 transform ``T_camera_buffer`` from a calibrated RGB image.

        For a buffer-frame point ``p_buffer`` in metres,
        ``p_camera = T_camera_buffer @ [*p_buffer, 1]``. Requires at least one
        visible marker registered by ``place``. Marker print orientation must
        match the buffer axes: its top edge points along +X.
        """
        image_rgb = np.asarray(image_rgb)
        if image_rgb.ndim != 3 or image_rgb.shape[2] != 3 or image_rgb.dtype != np.uint8:
            raise ValueError("image_rgb must be an HxWx3 uint8 RGB image")

        markers = {
            item.marker_id: (column, row, item)
            for (column, row), item in self._cells.items()
            if isinstance(item, ArucoTile)
        }
        if not markers:
            raise LocalizationError("no ArUco tiles have been placed in the buffer")

        gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
        detector = cv2.aruco.ArucoDetector(
            self._dictionary, cv2.aruco.DetectorParameters()
        )
        detected_corners, detected_ids, _rejected = detector.detectMarkers(gray)
        if detected_ids is None:
            raise LocalizationError("no known ArUco markers were detected")

        object_points = []
        image_points = []
        for corners, marker_id in zip(detected_corners, np.asarray(detected_ids).reshape(-1)):
            placement = markers.get(int(marker_id))
            if placement is None:
                continue
            column, row, item = placement
            center = self.cell_center_m(column, row)
            half_size = item.marker_size_mm / 2000
            object_points.extend(
                [
                    center + (-half_size, half_size, 0),
                    center + (half_size, half_size, 0),
                    center + (half_size, -half_size, 0),
                    center + (-half_size, -half_size, 0),
                ]
            )
            image_points.extend(np.asarray(corners).reshape(4, 2))

        if not object_points:
            raise LocalizationError("no placed ArUco markers were detected")

        object_points = np.ascontiguousarray(object_points, dtype=np.float64)
        image_points = np.ascontiguousarray(image_points, dtype=np.float64)
        success, rotation_vector, translation = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.distortion_coefficients,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success or not np.isfinite(rotation_vector).all() or not np.isfinite(translation).all():
            raise LocalizationError("pose estimation failed")

        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = cv2.Rodrigues(rotation_vector)[0]
        transform[:3, 3] = translation.reshape(3)
        return transform
