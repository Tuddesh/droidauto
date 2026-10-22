"""JSON configuration shared by capture, planning and execution."""

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from buffer import Buffer
from buffer.items import ArucoTile


def positive(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    return float(value)


class StationConfig:
    """Paths are resolved relative to the configuration file, never the shell cwd."""

    def __init__(self, path):
        self.path = Path(path).resolve()
        self.data = json.loads(self.path.read_text())
        self.layout = self.data["buffer"]
        self.camera = self.data["camera"]
        self.task = self.data["task"]
        self.motion = self.data["motion"]
        for name in ("hover_height_m", "transit_height_m", "dwell_s"):
            positive(self.task[name], name)
        if self.task["transit_height_m"] < self.task["hover_height_m"]:
            raise ValueError("transit_height_m must be at least hover_height_m")
        for name in ("rate_hz", "speed_m_s", "rotation_speed_rad_s", "position_tolerance_m",
                     "rotation_tolerance_rad", "settle_timeout_s", "max_tracking_error_m",
                     "max_tracking_error_rad"):
            positive(self.motion[name], name)
        if self.motion["max_tracking_error_m"] < self.motion["position_tolerance_m"]:
            raise ValueError("Tracking distance bound must be at least the arrival tolerance")
        if self.motion["max_tracking_error_rad"] < self.motion["rotation_tolerance_rad"]:
            raise ValueError("Tracking rotation bound must be at least the arrival tolerance")
        for name in ("max_reprojection_rms_px", "max_capture_age_s"):
            positive(self.camera[name], name)
        if self.camera["view"] not in ("left", "right"):
            raise ValueError("camera.view must be left or right")
        count = self.camera["min_visible_markers"]
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("min_visible_markers must be a positive integer")
        # Constructing the layout validates cell dimensions, items and overlaps.
        grid = self.make_buffer(np.eye(3))
        ids = grid.marker_cells()
        if count > len(ids):
            raise ValueError("min_visible_markers exceeds the number of placed markers")
        order = self.task["marker_order"]
        if not order or any(isinstance(i, bool) or not isinstance(i, int) or i not in ids for i in order):
            raise ValueError("marker_order must contain IDs placed in the buffer")
        if len(set(order)) != len(order):
            raise ValueError("marker_order must not repeat IDs (the task already loops)")

    def resolve(self, path) -> Path:
        path = Path(path).expanduser()
        return path if path.is_absolute() else self.path.parent / path

    @property
    def layout_digest(self) -> str:
        return hashlib.sha256(json.dumps(self.layout, sort_keys=True).encode()).hexdigest()

    def make_buffer(self, camera_matrix, distortion=None) -> Buffer:
        layout = self.layout
        grid = Buffer(
            columns=layout["columns"], rows=layout["rows"],
            camera_matrix=camera_matrix, distortion_coefficients=distortion,
            cell_size_mm=layout["cell_size_mm"], origin=tuple(layout["origin"]),
            dictionary=layout["dictionary"],
        )
        for marker in layout["markers"]:
            grid.place(ArucoTile(marker["id"], marker["size_mm"], layout["dictionary"]),
                       marker["column"], marker["row"])
        return grid
