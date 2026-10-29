"""Gridfinity layout and localization on the shared physical-object model.

Buffer owns cell geometry, layout occupants and its ArUco pose estimators.
Identity, dimensions and observed pose come from PhysicalObject; reset policy
and manipulation of scene objects belong to the reset manager and planner.
"""

from __future__ import annotations

from itertools import combinations, product

import cv2
import numpy as np

from .buffer_item import BufferItem
from .items.aruco_tile import ArucoTile
from scene.physical_object import PhysicalObject


class LocalizationError(RuntimeError):
    """The buffer pose could not be estimated from an image."""


class Buffer(PhysicalObject):
    """A rectangular Gridfinity buffer with known ArUco tile placements.

    Cell ``(column, row)`` is indexed from the physical lower-left corner as
    viewed from above using the declared grid axes. +X goes right,
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
        marker_orientation: str = "aligned",
        *,
        object_id: str | None = None,
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

        identity = {} if object_id is None else {"object_id": object_id}
        super().__init__(dimensions_m=(columns * cell_size_mm / 1000,
                                      rows * cell_size_mm / 1000, 0.0), **identity)
        self.columns = columns
        self.rows = rows
        self.cell_size_mm = float(cell_size_mm)
        self.origin = origin
        self.dictionary_name = dictionary
        if marker_orientation not in ("aligned", "independent"):
            raise ValueError("marker_orientation must be aligned or independent")
        self.marker_orientation = marker_orientation
        self.camera_matrix = camera_matrix.copy()
        self.distortion_coefficients = distortion_coefficients.copy()
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary)
        )
        self._cells: dict[tuple[int, int], BufferItem] = {}
        self._placed_object_ids: set[str] = set()
        self.last_localization: dict | None = None
        self.last_detection: dict | None = None
        self.last_joint_localization: dict | None = None

    def marker_cells(self) -> dict[int, tuple[int, int]]:
        """Return marker ID to cell assignments, rejecting ambiguous layouts."""
        markers = {}
        for cell, item in self._cells.items():
            if not isinstance(item, ArucoTile):
                continue
            if item.marker_id in markers:
                raise LocalizationError(f"marker {item.marker_id} is placed more than once")
            if item.dictionary != self.dictionary_name:
                raise LocalizationError("tile and buffer ArUco dictionaries differ")
            if item.marker_id >= self._dictionary.bytesList.shape[0]:
                raise LocalizationError(f"marker {item.marker_id} is outside the dictionary")
            markers[item.marker_id] = cell
        return markers

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
        if item.object_id in self._placed_object_ids:
            raise ValueError(f"item {item.object_id} is already placed in the buffer")

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
        self._placed_object_ids.add(item.object_id)

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
        visible marker in aligned mode, or two in independent mode. Independent
        mode estimates each printed marker's yaw while fitting the buffer pose.
        """
        self.last_localization = None
        self.last_detection = None
        self.last_joint_localization = None
        self.marker_cells()  # A duplicate physical ID cannot define a unique pose.
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
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(self._dictionary, parameters)
        detected_corners, detected_ids, _rejected = detector.detectMarkers(gray)
        self.last_detection = {
            "ids": [] if detected_ids is None else np.asarray(detected_ids).reshape(-1).tolist(),
            "corners": [np.asarray(corner).copy() for corner in detected_corners],
            "rejected_candidates": len(_rejected),
        }
        if detected_ids is None:
            raise LocalizationError("no known ArUco markers were detected")

        if self.marker_orientation == "independent":
            return self._localize_independent(markers, detected_corners, detected_ids)

        object_points = []
        image_points = []
        visible_ids = []
        for corners, marker_id in zip(detected_corners, np.asarray(detected_ids).reshape(-1)):
            placement = markers.get(int(marker_id))
            if placement is None:
                continue
            if int(marker_id) in visible_ids:
                raise LocalizationError(f"marker {marker_id} appears more than once in the image")
            visible_ids.append(int(marker_id))
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
        points_camera = object_points @ transform[:3, :3].T + transform[:3, 3]
        if np.any(points_camera[:, 2] <= 0):
            raise LocalizationError("estimated markers are behind the camera")
        projected, _ = cv2.projectPoints(
            object_points, rotation_vector, translation,
            self.camera_matrix, self.distortion_coefficients,
        )
        residuals = projected.reshape(-1, 2) - image_points
        rms = float(np.sqrt(np.mean(np.sum(residuals ** 2, axis=1))))
        if not np.isfinite(rms):
            raise LocalizationError("non-finite reprojection error")
        self.last_localization = {
            "visible_marker_ids": sorted(visible_ids),
            "reprojection_rms_px": rms,
        }
        return transform

    def localize_stereo(self, image_rgb, right_buffer, right_image_rgb, left_camera_right):
        """Jointly fit both images, using only observed corners in each eye.

        Marker yaw is shared across eyes. Two distinct known marker IDs across
        the pair suffice; an eye with zero or one detected marker is allowed.
        Missing detections never become synthetic corner observations.
        """
        if self.marker_orientation != "independent" or right_buffer.marker_orientation != "independent":
            raise ValueError("Joint stereo localization requires independent marker orientations")
        if self.marker_cells() != right_buffer.marker_cells():
            raise ValueError("Stereo buffers must describe the same marker placements")
        if (self.cell_size_mm != right_buffer.cell_size_mm or self.origin != right_buffer.origin
                or self.dictionary_name != right_buffer.dictionary_name
                or any(self.item_at(*cell).marker_size_mm != right_buffer.item_at(*cell).marker_size_mm
                       for cell in self.marker_cells().values())):
            raise ValueError("Stereo buffers must use the same physical dimensions and axes")
        left_camera_right = np.asarray(left_camera_right, dtype=float)
        if left_camera_right.shape != (4, 4) or not np.isfinite(left_camera_right).all():
            raise ValueError("left_camera_right must be a finite 4x4 transform")
        for grid, image in ((self, image_rgb), (right_buffer, right_image_rgb)):
            try:
                grid.localize(image)
            except LocalizationError:
                # Detection remains available even if this eye alone cannot
                # determine the board pose. The joint fit checks observability.
                pass
        markers = {item.marker_id: (column, row, item)
                   for (column, row), item in self._cells.items() if isinstance(item, ArucoTile)}
        left_detection = self.last_detection or {"corners": [], "ids": []}
        right_detection = right_buffer.last_detection or {"corners": [], "ids": []}
        return self._localize_independent(
            markers, left_detection["corners"], np.array(left_detection["ids"]),
            extra_views=[(right_buffer, right_detection["corners"],
                          np.array(right_detection["ids"]), left_camera_right)],
        )

    def _localize_independent(self, markers, detected_corners, detected_ids, extra_views=None):
        """Fit a common board pose and one arbitrary in-plane yaw per marker.

        Square-marker IPPE supplies possible normals and metric centers for
        initialization. Known center-to-center grid directions establish board
        yaw. All detected corners then enter a joint pixel-space least-squares
        fit; marker print yaw is a nuisance parameter, not a buffer axis.
        """
        from scipy.optimize import least_squares

        ids, centers, squares, observations, hypotheses, angle_indices = [], [], [], [], [], []
        views = []
        inputs = [(self, detected_corners, detected_ids, np.eye(4)), *(extra_views or [])]
        for grid, corners_list, ids_array, reference_camera in inputs:
            first = len(centers)
            seen = set()
            for corners, marker_id in zip(corners_list, np.asarray(ids_array).reshape(-1)):
                marker_id = int(marker_id)
                if marker_id not in markers:
                    continue
                if marker_id in seen:
                    raise LocalizationError(f"marker {marker_id} appears more than once in the image")
                seen.add(marker_id)
                column, row, tile = markers[marker_id]
                half = tile.marker_size_mm / 2000
                square = np.array([[-half, half, 0], [half, half, 0],
                                   [half, -half, 0], [-half, -half, 0]], dtype=float)
                observed = np.asarray(corners, dtype=float).reshape(4, 2)
                count, rotations, translations, *_ = cv2.solvePnPGeneric(
                    square, observed, grid.camera_matrix, grid.distortion_coefficients,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                candidates = [(reference_camera[:3, :3] @ cv2.Rodrigues(r)[0],
                               reference_camera[:3, :3] @ t.reshape(3) + reference_camera[:3, 3])
                              for r, t in zip(rotations, translations)
                              if np.isfinite(r).all() and np.isfinite(t).all() and t.reshape(3)[2] > 0]
                if not count or not candidates:
                    raise LocalizationError(f"cannot estimate a square pose for marker {marker_id}")
                if marker_id not in ids:
                    ids.append(marker_id)
                angle_indices.append(ids.index(marker_id))
                centers.append(self.cell_center_m(column, row))
                squares.append(square)
                observations.append(observed)
                hypotheses.append(candidates)
            views.append((grid, reference_camera, first, len(centers)))
        if len(ids) < 2:
            raise LocalizationError(f"Need at least two distinct known marker IDs across the available views; detected {sorted(ids)}")
        centers, squares = np.asarray(centers), np.asarray(squares)
        observations = np.asarray(observations).reshape(-1, 2)

        def object_points(parameters):
            angles = parameters[6:][angle_indices]
            cosine, sine = np.cos(angles), np.sin(angles)
            points = squares.copy()
            points[:, :, 0] = cosine[:, None] * squares[:, :, 0] - sine[:, None] * squares[:, :, 1]
            points[:, :, 1] = sine[:, None] * squares[:, :, 0] + cosine[:, None] * squares[:, :, 1]
            return (points + centers[:, None, :]).reshape(-1, 3)

        def residual(parameters):
            points = object_points(parameters)
            rotation = cv2.Rodrigues(parameters[:3])[0]
            projected = np.empty_like(observations)
            for grid, reference_camera, first, last in views:
                if first == last:
                    continue
                eye_rotation = reference_camera[:3, :3].T @ rotation
                eye_translation = reference_camera[:3, :3].T @ (parameters[3:6] - reference_camera[:3, 3])
                pixels, _ = cv2.projectPoints(points[first * 4:last * 4], cv2.Rodrigues(eye_rotation)[0],
                                              eye_translation, grid.camera_matrix, grid.distortion_coefficients)
                projected[first * 4:last * 4] = pixels.reshape(-1, 2)
            return (projected - observations).reshape(-1)

        starts = []
        for first, second in combinations(range(len(centers)), 2):
            if angle_indices[first] == angle_indices[second]:
                continue
            direction_board = centers[second] - centers[first]
            direction_board /= np.linalg.norm(direction_board)
            perpendicular_board = np.cross([0, 0, 1], direction_board)
            for (r1, t1), (r2, t2) in product(hypotheses[first], hypotheses[second]):
                normal = r1[:, 2] + r2[:, 2]
                if np.linalg.norm(normal) < 1e-8:
                    continue
                normal /= np.linalg.norm(normal)
                direction_camera = t2 - t1
                direction_camera -= normal * np.dot(normal, direction_camera)
                if np.linalg.norm(direction_camera) < 1e-8:
                    continue
                direction_camera /= np.linalg.norm(direction_camera)
                perpendicular_camera = np.cross(normal, direction_camera)
                rotation = (np.outer(direction_camera, direction_board)
                            + np.outer(perpendicular_camera, perpendicular_board)
                            + np.outer(normal, [0, 0, 1]))
                translation = ((t1 - rotation @ centers[first]) + (t2 - rotation @ centers[second])) / 2
                angles = []
                for marker_index in range(len(ids)):
                    candidates = [candidate for index, candidates in zip(angle_indices, hypotheses)
                                  if index == marker_index for candidate in candidates]
                    r, _ = max(candidates, key=lambda candidate: np.dot(normal, candidate[0][:, 2]))
                    relative = rotation.T @ r
                    angles.append(np.arctan2(relative[1, 0] - relative[0, 1], relative[0, 0] + relative[1, 1]))
                initial = np.r_[cv2.Rodrigues(rotation)[0].reshape(3), translation, angles]
                cost = float(np.sum(residual(initial) ** 2))
                if np.isfinite(cost):
                    starts.append((cost, initial))
        solutions = []
        for _, initial in sorted(starts, key=lambda item: item[0])[:8]:
            fitted = least_squares(residual, initial, method="trf", x_scale="jac", max_nfev=120,
                                   ftol=1e-9, xtol=1e-9, gtol=1e-9)
            parameters = fitted.x
            if not np.isfinite(parameters).all():
                continue
            rotation = cv2.Rodrigues(parameters[:3])[0]
            points = object_points(parameters) @ rotation.T + parameters[3:6]
            valid = True
            for _, reference_camera, first, last in views:
                if first == last:
                    continue
                eye_points = (points[first * 4:last * 4] - reference_camera[:3, 3]) @ reference_camera[:3, :3]
                normal = reference_camera[:3, :3].T @ rotation[:, 2]
                if np.any(eye_points[:, 2] <= 0) or np.dot(normal, eye_points.mean(axis=0)) >= 0:
                    valid = False
                    break
            if not valid:
                continue
            error = residual(parameters).reshape(-1, 2)
            rms = float(np.sqrt(np.mean(np.sum(error ** 2, axis=1))))
            if np.isfinite(rms):
                solutions.append((rms, parameters))
        if not solutions:
            raise LocalizationError("no valid buffer pose fits the independently rotated markers")
        rms, parameters = min(solutions, key=lambda solution: solution[0])
        result = np.eye(4)
        result[:3, :3] = cv2.Rodrigues(parameters[:3])[0]
        result[:3, 3] = parameters[3:6]
        all_error = residual(parameters).reshape(-1, 4, 2)
        joint_yaws = {str(i): float(np.degrees(np.arctan2(np.sin(a), np.cos(a))))
                      for i, a in zip(ids, parameters[6:])}
        self.last_joint_localization = {
            "visible_marker_ids": sorted(ids), "reprojection_rms_px": rms,
            "marker_yaw_degrees": joint_yaws,
        }
        for grid, _, first, last in views:
            if first == last:
                grid.last_localization = None
                continue
            eye_ids = [ids[i] for i in angle_indices[first:last]]
            eye_errors = all_error[first:last]
            grid.last_localization = {
                "visible_marker_ids": sorted(eye_ids),
                "reprojection_rms_px": float(np.sqrt(np.mean(np.sum(eye_errors ** 2, axis=2)))),
                "marker_yaw_degrees": {str(i): joint_yaws[str(i)] for i in eye_ids},
                "orientation_mode": "independent", "pose_source": "joint_stereo" if extra_views else "monocular",
                "joint_marker_ids": sorted(ids),
                "per_marker_rms_px": {str(i): float(np.sqrt(np.mean(np.sum(e ** 2, axis=1))))
                                      for i, e in zip(eye_ids, eye_errors)},
            }
        return result

    def reprojection_rms(self, camera_buffer: np.ndarray) -> float:
        """Evaluate a pose against the last detected corners and estimated yaws."""
        if self.last_localization is None or self.last_detection is None:
            raise LocalizationError("localize an image successfully before evaluating another pose")
        matrix = np.asarray(camera_buffer, dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("camera_buffer must be a finite 4x4 transform")
        cells = self.marker_cells()
        points, observed = [], []
        angles = self.last_localization.get("marker_yaw_degrees", {})
        for marker_id, corners in zip(self.last_detection["ids"], self.last_detection["corners"]):
            if marker_id not in cells:
                continue
            column, row = cells[marker_id]
            tile = self.item_at(column, row)
            half = tile.marker_size_mm / 2000
            square = np.array([[-half, half, 0], [half, half, 0],
                               [half, -half, 0], [-half, -half, 0]])
            angle = np.radians(angles.get(str(marker_id), 0))
            cosine, sine = np.cos(angle), np.sin(angle)
            rotation = np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])
            points.extend(square @ rotation.T + self.cell_center_m(column, row))
            observed.extend(np.asarray(corners).reshape(4, 2))
        projected, _ = cv2.projectPoints(np.asarray(points), cv2.Rodrigues(matrix[:3, :3])[0],
                                         matrix[:3, 3], self.camera_matrix, self.distortion_coefficients)
        return float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - observed) ** 2, axis=1))))

    def draw_localization(self, image_rgb: np.ndarray) -> np.ndarray:
        """Annotate the image from the last localize() call without redetecting.

        Green outlines match this layout; red outlines are detected IDs absent
        from it. This remains available when detection succeeds but PnP or the
        caller's quality requirements reject the pose.
        """
        image = np.asarray(image_rgb).copy()
        if self.last_detection is None:
            return image
        known = self.marker_cells()
        for marker_id, corners in zip(self.last_detection["ids"], self.last_detection["corners"]):
            points = np.rint(corners).astype(np.int32).reshape(4, 2)
            color = (0, 220, 0) if marker_id in known else (255, 60, 60)
            cv2.polylines(image, [points], True, color, 2)
            label = f"ID {marker_id}"
            label += f" cell {known[marker_id]}" if marker_id in known else " NOT IN LAYOUT"
            x, y = points[0]
            cv2.putText(image, label, (max(0, x), max(18, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        return image
