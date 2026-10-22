"""Persist and display the exact calibration image, including failed attempts."""

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
        self.annotated_rgb = None
        self.data = {"status": "started"}
        self._lines = []

    def _write_image(self, name, image_rgb):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / name
        if not cv2.imwrite(str(path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR),
                           [cv2.IMWRITE_PNG_COMPRESSION, 1]):
            raise RuntimeError(f"Could not save calibration diagnostic image: {path}")
        return path

    def captured(self, frame):
        self.frame = frame
        self.annotated_rgb = frame.image_rgb.copy()
        path = self._write_image("input.png", frame.image_rgb)
        self.data.update({
            "camera_serial": frame.serial, "view": frame.view,
            "image_timestamp_s": frame.timestamp_s,
            "camera_matrix": frame.camera_matrix.tolist(),
            "distortion": frame.distortion.tolist(),
            "image_size_px": [frame.image_rgb.shape[1], frame.image_rgb.shape[0]],
            "input_image": "input.png",
            "image_mean": float(np.mean(frame.image_rgb)),
            "image_min": int(frame.image_rgb.min()), "image_max": int(frame.image_rgb.max()),
        })
        print(f"Calibration input ({frame.serial}, {frame.view}): {path}", flush=True)

    def localized(self, grid):
        self.annotated_rgb = grid.draw_localization(self.frame.image_rgb)
        detection = grid.last_detection or {}
        detected = detection.get("ids", [])
        known = grid.marker_cells()
        self.data.update({
            "detected_marker_ids": detected,
            "matched_marker_ids": sorted(set(detected) & set(known)),
            "unknown_marker_ids": sorted(set(detected) - set(known)),
            "rejected_candidates": detection.get("rejected_candidates", 0),
            "quality": grid.last_localization,
        })
        self._lines = [f"Detected IDs: {detected}",
                       f"Matched: {self.data['matched_marker_ids']}; unknown: {self.data['unknown_marker_ids']}"]
        if grid.last_localization is not None:
            self._lines.append(f"Reprojection RMS: {grid.last_localization['reprojection_rms_px']:.3f} px")
        print("\n".join(self._lines), flush=True)

    def finish(self, error=None):
        self.data["status"] = "accepted" if error is None else "failed"
        self.data["error"] = None if error is None else str(error)
        if self.frame is None:
            return
        status = "ACCEPTED" if error is None else f"FAILED: {error}"
        print(status, flush=True)
        lines = [f"Camera {self.frame.serial}, {self.frame.view} lens", *self._lines, status]
        wrapped = [part for line in lines for part in textwrap.wrap(line, width=100)]
        banner = np.full((28 * (len(wrapped) + 1), self.annotated_rgb.shape[1], 3), 25, dtype=np.uint8)
        for index, line in enumerate(wrapped):
            cv2.putText(banner, line, (12, 25 + index * 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (240, 240, 240), 1, cv2.LINE_AA)
        self.annotated_rgb = np.vstack([banner, self.annotated_rgb])
        self._write_image("annotated.png", self.annotated_rgb)
        (self.directory / "report.json").write_text(json.dumps(self.data, indent=2, allow_nan=False) + "\n")
        print(f"Calibration report: {self.directory / 'report.json'}", flush=True)
        print(f"Annotated image: {self.directory / 'annotated.png'}", flush=True)

    def display(self):
        if self.frame is None or not self.show_image:
            return
        if os.name != "nt" and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            print("No desktop display detected; open input.png / annotated.png from the report directory.", flush=True)
            return
        title = f"Calibration {self.frame.serial} {self.frame.view} - R: raw, A: annotated, Q: close"
        try:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title, 1280, 850)
            image = self.annotated_rgb
            while True:
                cv2.imshow(title, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                key = cv2.waitKey(100) & 0xFF
                if key in (27, ord("q")) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key == ord("r"):
                    image = self.frame.image_rgb
                elif key == ord("a"):
                    image = self.annotated_rgb
        except cv2.error as exc:
            print(f"Could not display calibration image: {exc}. Saved images remain in {self.directory}", flush=True)
        finally:
            cv2.destroyAllWindows()
