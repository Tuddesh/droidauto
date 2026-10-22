"""One stationary image establishes a buffer pose that subsequent tasks reuse.

Detection and pose estimation belong to Buffer.localize(). This module only
coordinates acquisition, composes reference frames and persists the result.
"""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import tempfile
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
            raise ValueError("Buffer layout/localization model changed; click Recalibrate")
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
        """Replace the active calibration after a successful new capture."""
        path = Path(path)
        image_path = path.with_suffix(".png")
        if self.source_image_rgb is None:
            raise ValueError("Only a newly captured calibration has a source image to save")
        path.parent.mkdir(parents=True, exist_ok=True)
        success, png = cv2.imencode(".png", cv2.cvtColor(self.source_image_rgb, cv2.COLOR_RGB2BGR))
        if not success:
            raise RuntimeError("Could not encode calibration image")
        self.record["source_image"] = image_path.name
        payload = (json.dumps(self.record, indent=2, allow_nan=False) + "\n").encode()
        for destination, contents in ((image_path, png.tobytes()), (path, payload)):
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as output:
                    temporary = Path(output.name)
                    output.write(contents)
                os.replace(temporary, destination)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)


def load_hand_eye(path, serial: str, view: str) -> np.ndarray:
    """Read DROID's existing camera-to-tool pose, not a new hand-eye calibration."""
    key = f"{serial}_{view}"
    data = json.loads(Path(path).read_text())
    if key not in data:
        raise ValueError(f"No hand-eye calibration for {key} in {path}")
    return pose_to_transform(data[key]["pose"])


def capture_calibration(config, robot, camera, report=None) -> BufferCalibration:
    """Localize each eye of one stationary capture using Buffer.localize().

    Each eye uses its own intrinsics and hand-eye transform. Pixel fit is
    checked per eye; stereo agreement is checked after conversion to base.
    """
    before = robot.require_stationary()
    stereo = config.camera["view"] == "stereo"
    print("Capturing one stationary stereo pair..." if stereo else "Capturing one wrist-camera image...", flush=True)
    frames = camera.capture_pair() if stereo else [camera.capture()]
    try:
        after = robot.require_stationary()
        # Record acquisition age before diagnostic PNG encoding takes any time.
        captured_at = time.time()
    finally:
        if report is not None:
            for frame in frames:
                report.captured(frame)
    translation, rotation = pose_error(before.tool_pose, after.tool_pose)
    if (translation > 0.002 or rotation > 0.01
            or np.max(np.abs(after.joints - before.joints)) > 0.005):
        raise RuntimeError("Robot moved during capture; the image cannot establish a stationary pose")
    if stereo and abs(frames[0].timestamp_s - frames[1].timestamp_s) > 1e-6:
        raise RuntimeError("Stereo images do not share one capture timestamp")
    expected_views = {"left", "right"} if stereo else {config.camera["view"]}
    if len(frames) != len(expected_views) or {frame.view for frame in frames} != expected_views:
        raise RuntimeError("Captured views do not match the selected camera mode")

    accepted, eyes, grids = [], {}, {}
    for frame in frames:
        grid = config.make_buffer(frame.camera_matrix, frame.distortion)
        grids[frame.view] = grid
        eye = {"view": frame.view, "accepted": False}
        eyes[frame.view] = eye
        try:
            age = captured_at - frame.timestamp_s
            if not np.isfinite(age) or age < -0.05 or age > config.camera["max_capture_age_s"]:
                raise RuntimeError("Camera frame is stale or camera/PC clocks differ")
            if (frame.timestamp_s < before.timestamp_s - 0.05
                    or frame.timestamp_s > after.timestamp_s + 0.05):
                raise RuntimeError("Image timestamp falls outside the stationary robot-state interval")
            if frame.serial != str(config.camera["serial"]):
                raise ValueError("Captured image belongs to a different camera")
            print(f"Localizing buffer in {frame.view} eye...", flush=True)
            try:
                camera_buffer = grid.localize(frame.image_rgb)
            finally:
                if report is not None:
                    report.localized(grid, frame.view)
            quality = grid.last_localization
            eye["quality"] = quality
            visible_count = len(quality["visible_marker_ids"])
            if visible_count < config.camera["min_visible_markers"]:
                raise RuntimeError(f"matched {visible_count} markers; need {config.camera['min_visible_markers']}")
            if quality["reprojection_rms_px"] > config.camera["max_reprojection_rms_px"]:
                raise RuntimeError(f"RMS {quality['reprojection_rms_px']:.3f} px exceeds {config.camera['max_reprojection_rms_px']:.3f} px")
            tool_camera = load_hand_eye(config.resolve(config.camera["hand_eye_file"]),
                                       frame.serial, frame.view)
            base_buffer = before.tool_pose @ tool_camera @ camera_buffer
            eye.update({"accepted": True, "matrix": frame.camera_matrix.tolist(),
                        "distortion": frame.distortion.tolist(), "tool_camera": tool_camera.tolist(),
                        "camera_buffer": camera_buffer.tolist(), "base_buffer": base_buffer.tolist()})
            accepted.append((frame, base_buffer, tool_camera, quality))
        except (RuntimeError, ValueError, KeyError, OSError, cv2.error) as exc:
            eye["error"] = str(exc)
            print(f"{frame.view} eye rejected: {exc}", flush=True)
        finally:
            if report is not None:
                report.eye_result(frame.view, eye)
    if not accepted:
        raise RuntimeError("Localization rejected in all views: " + "; ".join(
            f"{name}: {eye.get('error', 'failed')}" for name, eye in eyes.items()))

    agreement = None
    if len(accepted) == 2:
        distance, angle = pose_error(accepted[0][1], accepted[1][1])
        agreement = {"position_error_m": distance, "rotation_error_rad": angle}
        if report is not None:
            report.data["stereo_agreement"] = agreement
        if (distance > config.camera["stereo_max_position_error_m"]
                or angle > config.camera["stereo_max_rotation_error_rad"]):
            raise RuntimeError(f"Stereo poses disagree: {distance * 1000:.1f} mm / {np.degrees(angle):.2f} deg; "
                               f"limits {config.camera['stereo_max_position_error_m'] * 1000:.1f} mm / "
                               f"{np.degrees(config.camera['stereo_max_rotation_error_rad']):.2f} deg")
    # Pixel-fit weights are a heuristic, not calibrated pose uncertainty.
    weights = np.array([len(q["visible_marker_ids"]) / max(q["reprojection_rms_px"], 0.25) ** 2
                        for _, _, _, q in accepted])
    weights /= weights.sum()
    base_buffer = np.eye(4)
    base_buffer[:3, 3] = sum(w * pose[:3, 3] for w, (_, pose, _, _) in zip(weights, accepted))
    averaged_rotation = sum(w * pose[:3, :3] for w, (_, pose, _, _) in zip(weights, accepted))
    u, _, vt = np.linalg.svd(averaged_rotation)
    base_buffer[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    fused_errors = {}
    for eye_frame, _, eye_tool_camera, _ in accepted:
        eye_pose = np.linalg.inv(before.tool_pose @ eye_tool_camera) @ base_buffer
        rms = grids[eye_frame.view].reprojection_rms(eye_pose)
        fused_errors[eye_frame.view] = rms
        eyes[eye_frame.view]["combined_pose_rms_px"] = rms
        if report is not None:
            report.eye_result(eye_frame.view, eyes[eye_frame.view])
        if not np.isfinite(rms) or rms > config.camera["max_reprojection_rms_px"]:
            raise RuntimeError(f"Combined pose has {rms:.3f} px RMS in {eye_frame.view}, exceeding "
                               f"{config.camera['max_reprojection_rms_px']:.3f} px; check per-eye hand-eye calibration")
    reference_index = int(np.argmax(weights))
    frame, _, tool_camera, _ = accepted[reference_index]
    # Store the fused pose expressed in the reference eye for the existing
    # transform-chain invariant. Actual per-eye PnP poses are retained in eyes.
    camera_buffer = np.linalg.inv(before.tool_pose @ tool_camera) @ base_buffer
    quality = {
        "visible_marker_ids": sorted({i for _, _, _, q in accepted for i in q["visible_marker_ids"]}),
        "reprojection_rms_px": max(fused_errors.values()),
        "per_eye_fused_rms_px": fused_errors,
        "accepted_views": [f.view for f, _, _, _ in accepted],
        "reference_view": frame.view,
        "stereo_agreement": agreement,
    }
    result = BufferCalibration({
        "version": 1, "created_at_s": time.time(), "image_timestamp_s": frame.timestamp_s,
        "robot_timestamp_s": before.timestamp_s, "layout": config.layout,
        "layout_digest": config.layout_digest, "base_buffer": base_buffer.tolist(),
        "base_tool": before.tool_pose.tolist(), "tool_camera": tool_camera.tolist(),
        "camera_buffer": camera_buffer.tolist(), "eyes": eyes,
        "hand_eye_file": str(config.resolve(config.camera["hand_eye_file"])),
        "camera": {"serial": frame.serial, "view": config.camera["view"], "reference_view": frame.view,
                   "image_size_px": [frame.image_rgb.shape[1], frame.image_rgb.shape[0]],
                   "matrix": frame.camera_matrix.tolist(), "distortion": frame.distortion.tolist()},
        "quality": quality,
    }, source_image_rgb=frame.image_rgb.copy())
    if report is not None:
        result.record["diagnostics_directory"] = str(report.directory)
    result.check_config(config)
    return result
