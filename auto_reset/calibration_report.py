"""Save and display the exact monocular/stereo inputs, including failed attempts."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import textwrap

import cv2
import numpy as np


class CalibrationReport:
    def __init__(self, directory, show_image=True):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.directory = Path(directory) / "calibration_attempts" / stamp
        self.show_image = show_image
        self.frame = None
        self.frames, self.overlays = {}, {}
        self.annotated_rgb = None
        self.data = {"status": "started", "views": {}}

    @staticmethod
    def _combine(images):
        if not images:
            return None
        height = max(image.shape[0] for image in images)
        return np.hstack([cv2.copyMakeBorder(image, 0, height - image.shape[0], 0, 0,
                                            cv2.BORDER_CONSTANT, value=(25, 25, 25)) for image in images])

    @property
    def raw_image_rgb(self):
        return self._combine([frame.image_rgb for frame in self.frames.values()])

    def _write_image(self, name, image_rgb):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / name
        if not cv2.imwrite(str(path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR),
                           [cv2.IMWRITE_PNG_COMPRESSION, 1]):
            raise RuntimeError(f"Could not save calibration image: {path}")
        return path

    def captured(self, frame):
        if self.frame is None:
            self.frame = frame
        self.frames[frame.view] = frame
        self.overlays[frame.view] = frame.image_rgb.copy()
        path = self._write_image(f"input_{frame.view}.png", frame.image_rgb)
        self.data["views"][frame.view] = {
            "camera_serial": frame.serial, "image_timestamp_s": frame.timestamp_s,
            "camera_matrix": frame.camera_matrix.tolist(), "distortion": frame.distortion.tolist(),
            "image_size_px": [frame.image_rgb.shape[1], frame.image_rgb.shape[0]],
            "input_image": path.name, "image_mean": float(np.mean(frame.image_rgb)),
            "image_min": int(frame.image_rgb.min()), "image_max": int(frame.image_rgb.max()),
            "left_camera_eye": None if frame.left_camera_eye is None else frame.left_camera_eye.tolist(),
        }
        self.annotated_rgb = self._combine(list(self.overlays.values()))
        print(f"Calibration input ({frame.serial}, {frame.view}): {path}", flush=True)

    def localized(self, grid, view=None):
        view = self.frame.view if view is None else view
        self.overlays[view] = grid.draw_localization(self.frames[view].image_rgb)
        detection = grid.last_detection or {}
        detected, known = detection.get("ids", []), grid.marker_cells()
        details = self.data["views"][view]
        details.update({
            "detected_marker_ids": detected,
            "matched_marker_ids": sorted(set(detected) & set(known)),
            "unknown_marker_ids": sorted(set(detected) - set(known)),
            "rejected_candidates": detection.get("rejected_candidates", 0),
            "quality": grid.last_localization,
        })
        self.annotated_rgb = self._combine(list(self.overlays.values()))
        print(f"{view}: detected {detected}; matched {details['matched_marker_ids']}; quality {grid.last_localization}", flush=True)

    def eye_result(self, view, result):
        self.data["views"][view].update({"accepted": result["accepted"],
                                           "error": result.get("error"),
                                           "combined_pose_rms_px": result.get("combined_pose_rms_px")})

    def finish(self, error=None):
        self.data["status"] = "accepted" if error is None else "failed"
        self.data["error"] = None if error is None else str(error)
        if self.frame is None:
            return
        status = "ACCEPTED" if error is None else f"FAILED: {error}"
        print(status, flush=True)
        images = []
        for view, frame in self.frames.items():
            details = self.data["views"][view]
            quality = details.get("quality")
            lines = [f"Camera {frame.serial}, {view} eye",
                     f"Detected: {details.get('detected_marker_ids', [])}; matched: {details.get('matched_marker_ids', [])}"]
            if quality is not None:
                lines.append(f"RMS: {quality['reprojection_rms_px']:.3f} px; marker yaw: {quality.get('marker_yaw_degrees', {})}")
            if details.get("combined_pose_rms_px") is not None:
                lines.append(f"Combined-pose RMS: {details['combined_pose_rms_px']:.3f} px")
            lines.append("Eye accepted" if details.get("accepted") else f"Eye failed: {details.get('error') or 'not localized'}")
            agreement = self.data.get("stereo_agreement")
            if agreement:
                lines.append(f"Stereo difference: {agreement['position_error_m'] * 1000:.1f} mm / {np.degrees(agreement['rotation_error_rad']):.2f} deg")
            lines.append(status)
            wrapped = [part for line in lines for part in textwrap.wrap(line, width=100)]
            banner = np.full((28 * (len(wrapped) + 1), frame.image_rgb.shape[1], 3), 25, dtype=np.uint8)
            for index, line in enumerate(wrapped):
                cv2.putText(banner, line, (12, 25 + index * 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (240, 240, 240), 1, cv2.LINE_AA)
            annotated = np.vstack([banner, self.overlays[view]])
            self._write_image(f"annotated_{view}.png", annotated)
            images.append(annotated)
        self.annotated_rgb = self._combine(images)
        self._write_image("input.png", self.raw_image_rgb)
        self._write_image("annotated.png", self.annotated_rgb)
        (self.directory / "report.json").write_text(json.dumps(self.data, indent=2, allow_nan=False) + "\n")
        print(f"Calibration report: {self.directory / 'report.json'}", flush=True)
        print(f"Annotated image: {self.directory / 'annotated.png'}", flush=True)

    def display(self):
        if self.frame is None or not self.show_image:
            return
        if os.name != "nt" and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            print(f"No desktop display; open images in {self.directory}", flush=True)
            return
        title = f"Calibration {self.frame.serial} - R: raw, A: annotated, Q: close"
        try:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title, 1600 if len(self.frames) == 2 else 1280, 850)
            image = self.annotated_rgb
            while True:
                cv2.imshow(title, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                key = cv2.waitKey(100) & 0xFF
                if key in (27, ord("q")) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key == ord("r"):
                    image = self.raw_image_rgb
                elif key == ord("a"):
                    image = self.annotated_rgb
        except cv2.error as exc:
            print(f"Could not display image: {exc}. Images saved in {self.directory}", flush=True)
        finally:
            cv2.destroyAllWindows()
