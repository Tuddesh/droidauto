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
        if camera["view"] == "stereo" and camera.get("stereo_extrinsics_source") != "zed_sdk_rectified":
            raise ValueError("Saved calibration predates the SDK stereo-offset correction; click Recalibrate")
        if "tool_left_camera_pose" in config.camera and not np.allclose(
            self.record.get("tool_left_camera", np.zeros((4, 4))),
            pose_to_transform(config.camera["tool_left_camera_pose"]), atol=1e-9, rtol=0,
        ):
            raise ValueError("Configured camera mounting pose changed; click Recalibrate")
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
    result = pose_to_transform(data[key]["pose"])
    if np.allclose(result, np.eye(4), atol=1e-12, rtol=0):
        raise ValueError(f"Hand-eye entry {key} is an all-zero identity placeholder; "
                         "a measured wrist-camera-to-tool transform is required")
    sibling = f"{serial}_{'right' if view == 'left' else 'left'}"
    if sibling in data and np.allclose(result, pose_to_transform(data[sibling]["pose"]), atol=1e-12, rtol=0):
        raise ValueError(f"Hand-eye entries {key} and {sibling} are identical; "
                         "separate stereo eyes cannot have the same camera-to-tool pose")
    return result


def capture_calibration(config, robot, camera, report=None) -> BufferCalibration:
    """Localize each eye of one stationary capture using Buffer.localize().

    Each eye uses its own intrinsics. The SDK stereo transform puts both
    poses into the left optical frame. Tool mounting is applied afterwards.
    """
    base_tool, robot_timestamps = robot.read_calibration_pose()
    stereo = config.camera["view"] == "stereo"
    print("Capturing one stationary stereo pair..." if stereo else "Capturing one wrist-camera image...", flush=True)
    frames = camera.capture_pair() if stereo else [camera.capture()]
    if report is not None:
        for frame in frames:
            report.captured(frame)
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
            left_camera_eye = frame.left_camera_eye
            if left_camera_eye is None:
                if frame.view == "right":
                    raise ValueError("Right image is missing the SDK stereo transform")
                left_camera_eye = np.eye(4)
            left_camera_eye = transform(left_camera_eye)
            left_buffer = left_camera_eye @ camera_buffer
            eye.update({"accepted": True, "matrix": frame.camera_matrix.tolist(),
                        "distortion": frame.distortion.tolist(), "left_camera_eye": left_camera_eye.tolist(),
                        "camera_buffer": camera_buffer.tolist(), "left_buffer": left_buffer.tolist()})
            accepted.append((frame, left_buffer, left_camera_eye, quality))
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
        agreement = {"position_error_m": distance, "rotation_error_rad": angle,
                     "frame": "left_camera", "extrinsics_source": "zed_sdk_rectified"}
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
    left_buffer = np.eye(4)
    left_buffer[:3, 3] = sum(w * pose[:3, 3] for w, (_, pose, _, _) in zip(weights, accepted))
    averaged_rotation = sum(w * pose[:3, :3] for w, (_, pose, _, _) in zip(weights, accepted))
    u, _, vt = np.linalg.svd(averaged_rotation)
    left_buffer[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    fused_errors = {}
    for eye_frame, _, left_camera_eye, _ in accepted:
        eye_pose = np.linalg.inv(left_camera_eye) @ left_buffer
        rms = grids[eye_frame.view].reprojection_rms(eye_pose)
        fused_errors[eye_frame.view] = rms
        eyes[eye_frame.view]["combined_pose_rms_px"] = rms
        if report is not None:
            report.eye_result(eye_frame.view, eyes[eye_frame.view])
        if not np.isfinite(rms) or rms > config.camera["max_reprojection_rms_px"]:
            raise RuntimeError(f"Combined pose has {rms:.3f} px RMS in {eye_frame.view}, exceeding "
                               f"{config.camera['max_reprojection_rms_px']:.3f} px")
    reference_index = int(np.argmax(weights))
    frame, _, left_camera_reference, _ = accepted[reference_index]
    if report is not None:
        report.data["left_buffer"] = left_buffer.tolist()
        report.data["accepted_views"] = [f.view for f, _, _, _ in accepted]
    # The stereo baseline does not supply the robot's camera mounting pose.
    # A known mount can be supplied explicitly; otherwise load measured DROID
    # hand-eye calibration for the left eye, or derive it from a valid right.
    if "tool_left_camera_pose" in config.camera:
        tool_left_camera = pose_to_transform(config.camera["tool_left_camera_pose"])
        mounting_source = "config.camera.tool_left_camera_pose"
    else:
        try:
            tool_left_camera = load_hand_eye(config.resolve(config.camera["hand_eye_file"]), frame.serial, "left")
            mounting_source = f"{frame.serial}_left"
        except (ValueError, OSError, KeyError) as left_error:
            right_frame = next((f for f in frames if f.view == "right"), None)
            try:
                if right_frame is None or right_frame.left_camera_eye is None:
                    raise ValueError("No right-eye stereo geometry available")
                tool_right_camera = load_hand_eye(config.resolve(config.camera["hand_eye_file"]), frame.serial, "right")
                tool_left_camera = tool_right_camera @ np.linalg.inv(transform(right_frame.left_camera_eye))
                mounting_source = f"{frame.serial}_right + zed_sdk_rectified"
            except (ValueError, OSError, KeyError) as right_error:
                raise ValueError("Stereo localization succeeded, but robot-frame calibration needs a valid "
                                 "camera-to-tool mounting transform. Supply camera.tool_left_camera_pose "
                                 f"[x,y,z,roll,pitch,yaw] in metres/radians or measured DROID hand-eye data. "
                                 f"Left: {left_error}; right: {right_error}") from right_error
    base_buffer = base_tool @ tool_left_camera @ left_buffer
    tool_camera = tool_left_camera @ left_camera_reference
    # Store the fused pose expressed in the reference eye for the existing
    # transform-chain invariant. Actual per-eye PnP poses are retained in eyes.
    camera_buffer = np.linalg.inv(base_tool @ tool_camera) @ base_buffer
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
        "robot_timestamps": robot_timestamps, "stationarity": "operator_confirmed", "layout": config.layout,
        "layout_digest": config.layout_digest, "base_buffer": base_buffer.tolist(),
        "base_tool": base_tool.tolist(), "tool_camera": tool_camera.tolist(),
        "camera_buffer": camera_buffer.tolist(), "eyes": eyes,
        "hand_eye_file": str(config.resolve(config.camera["hand_eye_file"])),
        "mounting_source": mounting_source,
        "tool_left_camera": tool_left_camera.tolist(),
        "camera": {"serial": frame.serial, "view": config.camera["view"], "reference_view": frame.view,
                   "stereo_extrinsics_source": "zed_sdk_rectified",
                   "image_size_px": [frame.image_rgb.shape[1], frame.image_rgb.shape[0]],
                   "matrix": frame.camera_matrix.tolist(), "distortion": frame.distortion.tolist()},
        "quality": quality,
    }, source_image_rgb=frame.image_rgb.copy())
    if report is not None:
        result.record["diagnostics_directory"] = str(report.directory)
    result.check_config(config)
    return result
