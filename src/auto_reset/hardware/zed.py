"""Full-resolution rectified ZED images (2208 x 1242 at 15 FPS on ZED Mini)."""

from dataclasses import dataclass
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
    """The lab PC needs the installed ZED SDK's Python bindings in its own env."""

    def __init__(self, serial: str, view: str = "left", auto_exposure_roi=None):
        if view not in ("left", "right", "stereo"):
            raise ValueError("view must be left, right or stereo")
        self.serial, self.view = str(serial), view
        self._camera = None
        self.auto_exposure_roi = auto_exposure_roi
        self._needs_exposure_settle = True
        self._exposure_settling = None
        self._exposure_frames = {}
        self._highlight_stats = {}
        self._exposure_warning_reported = False

    def __enter__(self):
        try:
            import pyzed.sl as sl
        except ImportError as exc:
            raise RuntimeError("Install the lab PC's ZED SDK Python bindings into DroidAuto's .conda env") from exc
        self._sl = sl
        camera = sl.Camera()
        params = sl.InitParameters()
        params.set_from_serial_number(int(self.serial))
        params.camera_resolution = sl.RESOLUTION.HD2K
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
            # Our image-feedback controller owns exposure/gain. The SDK's
            # average-brightness AE can clip white markers against a dark table.
            status = camera.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 0)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Cannot enable image-feedback exposure control: {status}")
            self.set_auto_exposure_roi(self.auto_exposure_roi)
        except Exception:
            camera.close()
            self._camera = None
            raise
        return self

    def set_auto_exposure_roi(self, roi):
        """Meter a normalized rectangle using automatic highlight-priority control."""
        if roi is not None:
            values = np.asarray(roi, dtype=float)
            if (values.shape != (4,) or not np.isfinite(values).all() or np.any(values < 0)
                    or np.any(values[2:] <= 0) or values[0] + values[2] > 1.000001
                    or values[1] + values[3] > 1.000001):
                raise ValueError("Exposure ROI must be normalized [x,y,width,height] inside the image")
        self.auto_exposure_roi = None if roi is None else list(map(float, roi))
        self._needs_exposure_settle = True

    def _settings(self):
        result = {"mode": "highlight_priority_auto", "sdk_auto_exposure": False,
                  "metering_roi_normalized": self.auto_exposure_roi,
                  "highlight_target_rgb": 240, "highlights": self._highlight_stats,
                  "settling": self._exposure_settling}
        for name in ("EXPOSURE", "GAIN"):
            status, value = self._camera.get_camera_settings(getattr(self._sl.VIDEO_SETTINGS, name))
            if status == self._sl.ERROR_CODE.SUCCESS:
                result[name.lower()] = int(value)
        return result

    def _settle_auto_exposure(self):
        """Adapt measured exposure/gain to protect highlights in both selected eyes.

        Pixel level 240 leaves headroom below 8-bit clipping. A 3x3 median
        excludes isolated hot pixels; coherent bright patches still drive
        exposure down. Settings are selected from image feedback, not preset
        shutter/gain values. The returned frame is the image actually measured.
        """
        started = time.monotonic()
        views = ("left", "right") if self.view == "stereo" else (self.view,)
        required_stable = 3 if self._needs_exposure_settle else 1
        stable_count, pending_frames, count = 0, 0, 0
        limited, protected = False, False
        best_frames, best_settings, best_stats, best_peak = None, None, None, -1
        while True:
            self._grab()
            self._exposure_frames = {view: self._read_view(view, include_settings=False) for view in views}
            count += 1
            stats = {}
            for view, frame in self._exposure_frames.items():
                pixels = frame.image_rgb
                if self.auto_exposure_roi is not None:
                    x, y, w, h = self.auto_exposure_roi
                    height, width = pixels.shape[:2]
                    pixels = pixels[int(y*height):min(height,max(int(y*height)+1,int((y+h)*height))),
                                    int(x*width):min(width,max(int(x*width)+1,int((x+w)*width)))]
                # Protect every color channel, not just grayscale brightness.
                brightest = np.ascontiguousarray(pixels.max(axis=2))
                filtered = cv2.medianBlur(brightest, 3)
                stats[view] = {"peak_rgb": int(filtered.max()),
                               "clipped_fraction": float(np.mean(brightest >= 250))}
            self._highlight_stats = stats
            peak = max(item["peak_rgb"] for item in stats.values())
            protected = peak <= 244
            elapsed = time.monotonic() - started
            if pending_frames:
                pending_frames -= 1
                if elapsed < 3.0:
                    continue
                limited = True
                break
            settings = self._settings()
            if "exposure" not in settings or "gain" not in settings:
                raise RuntimeError("ZED cannot report exposure/gain for automatic highlight control")
            exposure, gain = settings["exposure"], settings["gain"]
            if protected and peak > best_peak:
                best_frames, best_settings, best_stats, best_peak = self._exposure_frames, settings, stats, peak
            if 212 <= peak <= 244:
                stable_count += 1
                if stable_count >= required_stable:
                    break
            else:
                stable_count = 0
            if elapsed >= 3.0:
                limited = True
                break
            change = None
            if peak > 244:
                # Reduce amplification first, preserving low-noise marker edges.
                if gain > 0:
                    change = ("GAIN", max(0, gain-max(1, int(np.ceil(gain*0.5)))))
                elif exposure > 0:
                    factor = 0.65 if peak >= 250 else 240.0/peak
                    change = ("EXPOSURE", max(0, min(exposure-1, int(exposure*factor))))
            elif peak < 212:
                factor = min(1.3, 240.0/max(peak, 1))
                if exposure < 100:
                    change = ("EXPOSURE", min(100, max(exposure+1, int(np.ceil(exposure*factor)))))
                elif gain < 100:
                    change = ("GAIN", min(100, gain+max(1, int(np.ceil(max(gain,1)*0.15)))))
            if change is None:
                if stable_count:
                    continue
                limited = True
                break
            status = self._camera.set_camera_settings(getattr(self._sl.VIDEO_SETTINGS,change[0]),change[1])
            if status != self._sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Cannot adjust ZED {change[0].lower()}: {status}")
            pending_frames = 2  # Drain sensor/ISP latency before judging a change.
        # Prefer a measured darker frame to a clipped final trial, including
        # when discrete shutter steps straddle the highlight target.
        if best_frames is not None and (not protected or best_peak > peak):
            for name in ("EXPOSURE", "GAIN"):
                value = best_settings[name.lower()]
                status = self._camera.set_camera_settings(getattr(self._sl.VIDEO_SETTINGS,name),value)
                if status != self._sl.ERROR_CODE.SUCCESS:
                    raise RuntimeError(f"Cannot restore highlight-protected {name.lower()}: {status}")
            self._exposure_frames = best_frames
            self._highlight_stats = best_stats
            protected = True
        self._exposure_settling = {"stable": stable_count >= required_stable,
                                  "highlight_protected": protected, "limited": limited,
                                  "frames": count, "duration_s": time.monotonic()-started}
        self._needs_exposure_settle = False
        if not protected and not self._exposure_warning_reported:
            print("Exposure warning: highlights still clip at the available settings; inspect the saved image.",flush=True)
            self._exposure_warning_reported = True
        elif protected:
            self._exposure_warning_reported = False
        metadata = self._settings()
        for frame in self._exposure_frames.values():
            frame.image_settings = metadata

    def capture(self) -> CameraFrame:
        """Return the exact rectified frame checked by the exposure controller."""
        self._settle_auto_exposure()
        return self._exposure_frames["left" if self.view == "stereo" else self.view]

    def capture_pair(self) -> list[CameraFrame]:
        """Return both eyes from the exposure-checked hardware grab and timestamp."""
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
