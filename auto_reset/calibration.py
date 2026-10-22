"""One stationary image establishes a buffer pose that subsequent tasks reuse.

Detection and pose estimation belong to Buffer.localize(). This module only
coordinates acquisition, composes reference frames and persists the result.
"""

from dataclasses import dataclass
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .transforms import pose_error, pose_to_transform, transform


@dataclass
class BufferCalibration:
    record: dict
    source_image_rgb: np.ndarray | None = None

    def __post_init__(self):
        if self.record["version"] != 1:
            raise ValueError("Unsupported buffer calibration version")
        for name in ("base_buffer", "base_tool", "tool_camera", "camera_buffer"):
            transform(self.record[name])
        composed = (transform(self.record["base_tool"]) @ transform(self.record["tool_camera"])
                    @ transform(self.record["camera_buffer"]))
        if not np.allclose(composed, self.record["base_buffer"], atol=1e-7):
            raise ValueError("Calibration frame composition is inconsistent")

    @property
    def base_buffer(self) -> np.ndarray:
        return transform(self.record["base_buffer"])

    @property
    def scan_tool_pose(self) -> np.ndarray:
        return transform(self.record["base_tool"])

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text()))

    def check_config(self, config):
        if self.record["layout_digest"] != config.layout_digest:
            raise ValueError("Buffer layout changed; use the matching layout or recalibrate")
        camera = self.record["camera"]
        if (camera["serial"] != str(config.camera["serial"])
                or camera["view"] != config.camera["view"]):
            raise ValueError("Saved calibration belongs to a different camera/view")
        quality = self.record["quality"]
        if (not np.isfinite(quality["reprojection_rms_px"])
                or quality["reprojection_rms_px"] > config.camera["max_reprojection_rms_px"]
                or len(quality["visible_marker_ids"]) < config.camera["min_visible_markers"]):
            raise ValueError("Saved calibration does not meet configured image-quality requirements")
        self.make_buffer(config)  # Also validate stored intrinsics.

    def make_buffer(self, config):
        camera = self.record["camera"]
        return config.make_buffer(camera["matrix"], camera["distortion"])

    def save(self, path):
        """Keep the single source image alongside an immutable calibration record."""
        path = Path(path)
        image_path = path.with_suffix(".png")
        if path.exists() or image_path.exists():
            raise FileExistsError(f"Calibration already exists at {path}; archive it before recalibrating")
        if self.source_image_rgb is None:
            raise ValueError("Only a newly captured calibration has a source image to save")
        path.parent.mkdir(parents=True, exist_ok=True)
        success, png = cv2.imencode(".png", cv2.cvtColor(self.source_image_rgb, cv2.COLOR_RGB2BGR))
        if not success:
            raise RuntimeError("Could not encode calibration image")
        # Exclusive creation avoids silently replacing a pose trusted by another task.
        with image_path.open("xb") as output:
            output.write(png.tobytes())
        self.record["source_image"] = image_path.name
        with path.open("x") as output:
            json.dump(self.record, output, indent=2, allow_nan=False)
            output.write("\n")


def load_hand_eye(path, serial: str, view: str) -> np.ndarray:
    """Read DROID's existing camera-to-tool pose, not a new hand-eye calibration."""
    key = f"{serial}_{view}"
    data = json.loads(Path(path).read_text())
    if key not in data:
        raise ValueError(f"No hand-eye calibration for {key} in {path}")
    return pose_to_transform(data[key]["pose"])


def capture_calibration(config, robot, camera) -> BufferCalibration:
    """Capture once while stationary, then call the configured buffer's localize().

    Open the camera before calling this function so camera startup does not
    separate the robot-state measurements from the image acquisition.
    """
    tool_camera = load_hand_eye(config.resolve(config.camera["hand_eye_file"]),
                               str(config.camera["serial"]), config.camera["view"])
    before = robot.require_stationary()
    frame = camera.capture()
    after = robot.require_stationary()
    translation, rotation = pose_error(before.tool_pose, after.tool_pose)
    if (translation > 0.002 or rotation > 0.01
            or np.max(np.abs(after.joints - before.joints)) > 0.005):
        raise RuntimeError("Robot moved during capture; the image cannot establish a stationary pose")
    age = time.time() - frame.timestamp_s
    if not np.isfinite(age) or age < -0.05 or age > config.camera["max_capture_age_s"]:
        raise RuntimeError("Camera frame is stale or camera/PC clocks differ")
    if (frame.timestamp_s < before.timestamp_s - 0.05
            or frame.timestamp_s > after.timestamp_s + 0.05):
        raise RuntimeError("Image timestamp falls outside the stationary robot-state interval")
    if frame.serial != str(config.camera["serial"]) or frame.view != config.camera["view"]:
        raise ValueError("Captured image does not match the configured camera/view")

    grid = config.make_buffer(frame.camera_matrix, frame.distortion)
    camera_buffer = grid.localize(frame.image_rgb)
    quality = grid.last_localization
    if (len(quality["visible_marker_ids"]) < config.camera["min_visible_markers"]
            or quality["reprojection_rms_px"] > config.camera["max_reprojection_rms_px"]):
        raise RuntimeError(f"Single-image localization did not meet configured requirements: {quality}")
    # Stationarity was checked around the image; use the measured pre-image pose.
    base_buffer = before.tool_pose @ tool_camera @ camera_buffer
    result = BufferCalibration({
        "version": 1,
        "created_at_s": time.time(),
        "image_timestamp_s": frame.timestamp_s,
        "robot_timestamp_s": before.timestamp_s,
        "layout": config.layout,
        "layout_digest": config.layout_digest,
        "base_buffer": base_buffer.tolist(),
        "base_tool": before.tool_pose.tolist(),
        "tool_camera": tool_camera.tolist(),
        "camera_buffer": camera_buffer.tolist(),
        "hand_eye_file": str(config.resolve(config.camera["hand_eye_file"])),
        "camera": {"serial": frame.serial, "view": frame.view,
                   "image_size_px": [frame.image_rgb.shape[1], frame.image_rgb.shape[0]],
                   "matrix": frame.camera_matrix.tolist(), "distortion": frame.distortion.tolist()},
        "quality": quality,
    }, source_image_rgb=frame.image_rgb.copy())
    result.check_config(config)
    return result
