"""Station hardware, buffer geometry and calibration configuration.

This loader retains the existing physical/camera setup and path resolution.
It does not require marker-hover task settings. Scene/reset construction from
configuration remains future work; the core scaffold is dependency-injected.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np

from buffer import Buffer
from buffer.items import ArucoTile
from scene import Cube


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
        self.layout.setdefault("marker_orientation", "independent")
        self.camera.setdefault("stereo_max_position_error_m", 0.02)
        self.camera.setdefault("stereo_max_rotation_error_rad", 0.1)
        roi = self.camera.get("auto_exposure_roi")
        if roi is not None:
            roi = np.asarray(roi, dtype=float)
            if (roi.shape != (4,) or not np.isfinite(roi).all() or np.any(roi < 0)
                    or np.any(roi[2:] <= 0) or roi[0]+roi[2] > 1.000001 or roi[1]+roi[3] > 1.000001):
                raise ValueError("camera.auto_exposure_roi must be normalized [x,y,width,height] inside the image")
        if "tool_left_camera_pose" in self.camera:
            from scene.transforms import pose_to_transform
            pose_to_transform(self.camera["tool_left_camera_pose"])
        for name in ("max_reprojection_rms_px",):
            positive(self.camera[name], name)
        if self.camera["view"] not in ("left", "right", "stereo"):
            raise ValueError("camera.view must be left, right or stereo")
        for name in ("stereo_max_position_error_m", "stereo_max_rotation_error_rad"):
            positive(self.camera[name], name)
        count = self.camera["min_visible_markers"]
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("min_visible_markers must be a positive integer")
        # Constructing the layout validates cell dimensions, items and overlaps.
        grid = self.make_buffer(np.eye(3))
        ids = grid.marker_cells()
        if count > len(ids):
            raise ValueError("min_visible_markers exceeds the number of placed markers")

    def resolve(self, path) -> Path:
        path = Path(path).expanduser()
        return path if path.is_absolute() else self.path.parent / path

    def save_marker_binding(self, object_id: str, marker_id: int, dictionary: str) -> None:
        """Persist one confirmed cube marker identity in this station config.

        Discovery and selecting the cube belong to the localization integration.
        Existing bindings must match; only the selected object's marker fields
        are written. The image-processing service is never called here.
        """
        data = json.loads(self.path.read_text())
        matches = [item for item in data.get("objects", []) if item.get("object_id") == object_id]
        if len(matches) != 1 or matches[0].get("type") != "cube":
            raise ValueError("Expected exactly one configured cube with the requested object_id")
        item = matches[0]
        marker = item.get("marker", {})
        cube = Cube(object_id=object_id, marker_id=marker.get("id"),
                    marker_dictionary=marker.get("dictionary"))
        cube.bind_marker(marker_id, dictionary)
        marker.update(family=cube.marker_family, id=cube.marker_id, dictionary=cube.marker_dictionary)
        item["marker"] = marker
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", dir=self.path.parent, delete=False) as file:
                temporary = Path(file.name)
                json.dump(data, file, indent=2, allow_nan=False)
                file.write("\n")
            os.chmod(temporary, self.path.stat().st_mode & 0o777)
            os.replace(temporary, self.path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        self.data["objects"] = data["objects"]

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
            marker_orientation=layout["marker_orientation"],
            object_id=layout.get("object_id"),
        )
        for marker in layout["markers"]:
            grid.place(ArucoTile(marker["id"], marker["size_mm"], layout["dictionary"],
                                 object_id=marker.get("object_id")),
                       marker["column"], marker["row"])
        return grid
