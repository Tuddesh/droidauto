"""Wide-field rectified ZED images (1280 x 720 at 15 FPS on ZED Mini)."""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import cv2
import numpy as np


@dataclass
class CameraFrame:
    image_rgb: np.ndarray
    camera_matrix: np.ndarray
    distortion: np.ndarray
    timestamp_s: float
    serial: str
    view: str
    left_camera_eye: np.ndarray | None = None
    image_settings: dict | None = None


class ZedWristCamera:
    """One SDK session per station lifetime; captures reuse the open camera."""

    def __init__(self, serial: str, view: str = "left", auto_exposure_roi=None):
        if view not in ("left", "right", "stereo"):
            raise ValueError("view must be left, right or stereo")
        self.serial, self.view = str(serial), view
        self._camera = None
        self.auto_exposure_roi = auto_exposure_roi
        self._needs_exposure_settle = True
        self._exposure_settling = None
        self._exposure_frames = {}
        self.aruco_detector = None
        self.exposure_directory = None
        self._exposure_search = None

    def __enter__(self):
        if self._camera is not None:
            return self
        try:
            import pyzed.sl as sl
        except ImportError as exc:
            raise RuntimeError("Install the lab PC's ZED SDK Python bindings into DroidAuto's .conda env") from exc
        self._sl = sl
        camera = sl.Camera()
        params = sl.InitParameters()
        params.set_from_serial_number(int(self.serial))
        params.camera_resolution = sl.RESOLUTION.HD720
        params.camera_fps = 15
        params.depth_mode = sl.DEPTH_MODE.NONE
        params.camera_image_flip = sl.FLIP_MODE.OFF
        params.coordinate_units = sl.UNIT.METER
        params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        status = camera.open(params)
        if status != sl.ERROR_CODE.SUCCESS:
            camera.close()
            raise RuntimeError(f"Cannot open ZED {self.serial}: {status}")
        self._camera = camera
        try:
            # SDK auto exposure supplies the starting point for a manual bracket.
            status = camera.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 1)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Cannot enable ZED auto exposure: {status}")
            self.set_auto_exposure_roi(self.auto_exposure_roi)
        except Exception:
            camera.close()
            self._camera = None
            raise
        return self

    def set_auto_exposure_roi(self, roi):
        """Start a new exposure search inside a normalized working-area rectangle."""
        if roi is not None:
            values = np.asarray(roi, dtype=float)
            if (values.shape != (4,) or not np.isfinite(values).all() or np.any(values < 0)
                    or np.any(values[2:] <= 0) or values[0] + values[2] > 1.000001
                    or values[1] + values[3] > 1.000001):
                raise ValueError("Exposure ROI must be normalized [x,y,width,height] inside the image")
        self.auto_exposure_roi = None if roi is None else list(map(float, roi))
        self._needs_exposure_settle = True

    def _settings(self):
        result = {"mode": "marker_exposure_bracket", "sdk_auto_exposure": False,
                  "metering_roi_normalized": self.auto_exposure_roi,
                  "search": self._exposure_search, "settling": self._exposure_settling}
        for name in ("EXPOSURE", "GAIN"):
            status, value = self._camera.get_camera_settings(getattr(self._sl.VIDEO_SETTINGS, name))
            if status == self._sl.ERROR_CODE.SUCCESS:
                result[name.lower()] = int(value)
        return result

    def _settle_auto_exposure(self):
        """Seed from SDK AE, search shutter steps, then hold settings for the burst.

        Both eyes share one exposure/gain. Recognized markers, their contrast,
        edge sharpness and white clipping rank candidates; unrelated highlights
        never set the exposure. SDK shutter units approximate relative EV steps
        at a fixed frame rate. Limits can truncate the requested -3..+3 range.
        """
        views = ("left", "right") if self.view == "stereo" else (self.view,)
        if not self._needs_exposure_settle:
            self._grab()
            self._exposure_frames = {view: self._read_view(view, include_settings=False) for view in views}
            metadata = self._settings()
            for frame in self._exposure_frames.values():
                frame.image_settings = metadata
            return

        started = time.monotonic()
        sl, camera = self._sl, self._camera
        self._grab()
        first = self._read_view(views[0], include_settings=False)
        height, width = first.image_rgb.shape[:2]
        roi = self.auto_exposure_roi
        if roi is None:
            status = camera.set_camera_settings_roi(sl.VIDEO_SETTINGS.AEC_AGC_ROI, sl.Rect(), sl.SIDE.BOTH, True)
        else:
            x, y, w, h = roi
            left, top = min(width-1, int(x*width)), min(height-1, int(y*height))
            rect = sl.Rect(left, top, min(width-left, max(1, int(w*width))), min(height-top, max(1, int(h*height))))
            status = camera.set_camera_settings_roi(sl.VIDEO_SETTINGS.AEC_AGC_ROI, rect, sl.SIDE.BOTH)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Cannot set ZED exposure metering region: {status}")
        status = camera.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 1)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Cannot enable ZED auto exposure: {status}")
        previous, stable, count = None, 0, 0
        auto_started = time.monotonic()
        while True:
            self._grab()
            count += 1
            settings = self._settings()
            if "exposure" not in settings or "gain" not in settings:
                raise RuntimeError("ZED cannot report exposure/gain for exposure bracketing")
            values = np.array([settings["exposure"], settings["gain"]])
            stable = stable + 1 if previous is not None and np.all(np.abs(values-previous) <= 1) else 0
            previous = values
            elapsed = time.monotonic() - auto_started
            if elapsed >= 2.0 or (elapsed >= 0.5 and stable >= 3):
                break
        baseline_exposure = max(1, int(settings["exposure"]))
        baseline_gain = int(settings["gain"])
        for name, value in (("AEC_AGC", 0), ("GAIN", baseline_gain)):
            status = camera.set_camera_settings(getattr(sl.VIDEO_SETTINGS, name), value)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Cannot lock ZED {name.lower()}: {status}")

        candidates, exposures = [], set()
        best_rank, best_frames, best_index = None, None, None
        for ev in range(-3, 4):
            exposure = int(np.clip(round(baseline_exposure * 2.0**ev), 1, 100))
            if exposure in exposures:
                continue
            exposures.add(exposure)
            status = camera.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, exposure)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Cannot set ZED bracket exposure: {status}")
            # Drain sensor/ISP latency before scoring a synchronized pair.
            for _ in range(3):
                self._grab()
                count += 1
            frames = {view: self._read_view(view, include_settings=False) for view in views}
            actual = self._settings()
            detected, marker_views, quality, per_eye = set(), 0, 0.0, {}
            for view, frame in frames.items():
                pixels = frame.image_rgb
                if roi is not None:
                    x, y, w, h = roi
                    ih, iw = pixels.shape[:2]
                    x0, y0 = min(iw-1, int(x*iw)), min(ih-1, int(y*ih))
                    pixels = pixels[y0:min(ih, y0+max(1, int(h*ih))), x0:min(iw, x0+max(1, int(w*iw)))]
                gray = cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY)
                mask = np.zeros(gray.shape, np.uint8)
                markers = []
                if self.aruco_detector is not None:
                    corners, ids, _, indices = self.aruco_detector.detectMarkersMultiDict(gray)
                    for quad, marker_id, index in zip(corners, [] if ids is None else ids.ravel(),
                                                      [] if indices is None else indices.ravel()):
                        key = (int(index), int(marker_id))
                        detected.add(key)
                        markers.append(list(key))
                        cv2.fillConvexPoly(mask, np.round(quad).astype(np.int32).reshape(4, 2), 255)
                marker_views += len(markers)
                selected = mask > 0 if np.any(mask) else np.ones(gray.shape, bool)
                values = gray[selected]
                dark, light = np.percentile(values, [10, 90])
                contrast = float(light-dark)
                sharpness = float(np.sqrt(cv2.Laplacian(gray, cv2.CV_64F)[selected].var()))
                whites = values > (dark+light)/2
                clipped = float(np.mean(np.any(pixels[selected][whites] >= 250, axis=1))) if np.any(whites) else 0.0
                readability = contrast * sharpness * (1.0-clipped)**2
                # Marker-free eyes must not affect marker-quality scoring.
                quality += readability if markers else 0.0
                per_eye[view] = {"markers": markers, "contrast": contrast, "sharpness": sharpness,
                                 "white_clipped_fraction": clipped, "readability": readability}
            fallback = sum(eye["readability"] for eye in per_eye.values()) if not detected else 0.0
            rank = (len(detected), marker_views, quality if detected else fallback,
                    -abs(float(np.log2(exposure/baseline_exposure))))
            candidate = {"nominal_ev": ev, "relative_ev_approx": float(np.log2(exposure/baseline_exposure)),
                         "requested_exposure": exposure, "exposure": actual["exposure"], "gain": actual["gain"],
                         "unique_markers": len(detected), "marker_views": marker_views,
                         "quality": quality, "eyes": per_eye, "frames": frames}
            candidates.append(candidate)
            if best_rank is None or rank > best_rank:
                best_rank, best_frames, best_index = rank, frames, len(candidates)-1

        winner = candidates[best_index]
        for name in ("EXPOSURE", "GAIN"):
            status = camera.set_camera_settings(getattr(sl.VIDEO_SETTINGS, name), winner[name.lower()])
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Cannot restore selected ZED {name.lower()}: {status}")
        for _ in range(3):
            self._grab()
            count += 1
        self._exposure_search = {"baseline_exposure": baseline_exposure, "baseline_gain": baseline_gain,
                                 "selected_index": best_index, "marker_based": bool(winner["unique_markers"]),
                                 "candidates": [{k: v for k, v in candidate.items() if k != "frames"}
                                                for candidate in candidates]}
        if self.exposure_directory is not None:
            directory = Path(self.exposure_directory) / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            directory.mkdir(parents=True, exist_ok=True)
            tiles = []
            for index, candidate in enumerate(candidates):
                tile = np.full((len(views)*180+58, 320, 3), 25, np.uint8)
                for eye_index, (view, frame) in enumerate(candidate["frames"].items()):
                    path = directory / f"step_{index:02d}_{view}.png"
                    bgr = cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2BGR)
                    if not cv2.imwrite(str(path), bgr):
                        raise OSError(f"Cannot save exposure candidate: {path}")
                    self._exposure_search["candidates"][index].setdefault("images", {})[view] = str(path)
                    tile[eye_index*180:(eye_index+1)*180] = cv2.resize(bgr, (320, 180))
                text = f"{candidate['relative_ev_approx']:+.1f} EV | shutter {candidate['exposure']} | gain {candidate['gain']}"
                cv2.putText(tile, text, (6, len(views)*180+22), cv2.FONT_HERSHEY_SIMPLEX, .45, (240,240,240), 1, cv2.LINE_AA)
                cv2.putText(tile, f"Markers: {candidate['unique_markers']} | eye detections: {candidate['marker_views']}",
                            (6, len(views)*180+44), cv2.FONT_HERSHEY_SIMPLEX, .45, (240,240,240), 1, cv2.LINE_AA)
                if index == best_index:
                    cv2.rectangle(tile, (1,1), (318,tile.shape[0]-2), (0,255,0), 3)
                tiles.append(tile)
            tile_height = tiles[0].shape[0]
            sheet = np.full((int(np.ceil(len(tiles)/4))*tile_height, 4*320, 3), 25, np.uint8)
            for index, tile in enumerate(tiles):
                row, column = divmod(index, 4)
                sheet[row*tile_height:(row+1)*tile_height, column*320:(column+1)*320] = tile
            path = directory / "comparison.png"
            if not cv2.imwrite(str(path), sheet):
                raise OSError(f"Cannot save exposure comparison: {path}")
            self._exposure_search["contact_sheet"] = str(path)
            (directory / "search.json").write_text(json.dumps(self._exposure_search, indent=2, allow_nan=False)+"\n")
            print(f"Exposure bracket: shutter {winner['exposure']}, gain {winner['gain']}; comparison: {path}", flush=True)
        self._exposure_settling = {"sdk_auto_stable": stable >= 3, "frames": count,
                                  "duration_s": time.monotonic()-started}
        self._exposure_frames = best_frames
        self._needs_exposure_settle = False
        metadata = self._settings()
        for frame in self._exposure_frames.values():
            frame.image_settings = metadata

    def capture(self) -> CameraFrame:
        """Return the exact rectified frame checked by the exposure controller."""
        self._settle_auto_exposure()
        return self._exposure_frames["left" if self.view == "stereo" else self.view]

    def capture_pair(self) -> list[CameraFrame]:
        """Return both eyes from the selected-exposure hardware grab and timestamp."""
        self._settle_auto_exposure()
        return [self._exposure_frames[view] if view in self._exposure_frames else self._read_view(view)
                for view in ("left", "right")]

    def _grab(self):
        if self._camera is None:
            raise RuntimeError("Open the camera with a context manager first")
        sl, camera = self._sl, self._camera
        status = camera.grab(sl.RuntimeParameters())
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED capture failed: {status}")

    def _read_view(self, view, include_settings=True) -> CameraFrame:
        sl, camera = self._sl, self._camera
        image = sl.Mat()
        status = camera.retrieve_image(image, sl.VIEW.LEFT if view == "left" else sl.VIEW.RIGHT)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED image retrieval failed: {status}")
        rgb = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2RGB)
        parameters = camera.get_camera_information().camera_configuration.calibration_parameters
        intrinsics = parameters.left_cam if view == "left" else parameters.right_cam
        matrix = np.array([[intrinsics.fx, 0, intrinsics.cx],
                           [0, intrinsics.fy, intrinsics.cy], [0, 0, 1]], dtype=float)
        stamp = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds() / 1000
        # LEFT/RIGHT are rectified views; raw distortion must not be applied again.
        # For rectified LEFT/RIGHT, the SDK gives the right optical origin in
        # the left optical frame. Do not use unrectified factory R/T here.
        left_camera_eye = (np.eye(4) if view == "left"
                           else np.asarray(parameters.stereo_transform.m, dtype=float).copy())
        return CameraFrame(rgb.copy(), matrix, np.zeros(5), stamp, self.serial, view,
                           left_camera_eye, self._settings() if include_settings else None)

    def __exit__(self, *_):
        if self._camera is not None:
            self._camera.close()
            self._camera = None
        self._exposure_frames.clear()
        self._needs_exposure_settle = True
