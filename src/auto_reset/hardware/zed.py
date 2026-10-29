"""Full-resolution rectified ZED images (2208 x 1242 at 15 FPS on ZED Mini)."""

from dataclasses import dataclass
from collections import deque
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
            status = camera.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 1)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Cannot enable ZED automatic exposure/gain: {status}")
            self.set_auto_exposure_roi(self.auto_exposure_roi)
        except Exception:
            camera.close()
            self._camera = None
            raise
        return self

    def set_auto_exposure_roi(self, roi):
        """Meter a normalized [x,y,width,height] rectangle; exposure stays AUTO."""
        sl = self._sl
        rectangle = sl.Rect()
        if roi is not None:
            values = np.asarray(roi, dtype=float)
            if (values.shape != (4,) or not np.isfinite(values).all() or np.any(values < 0)
                    or np.any(values[2:] <= 0) or values[0] + values[2] > 1.000001
                    or values[1] + values[3] > 1.000001):
                raise ValueError("Exposure ROI must be normalized [x,y,width,height] inside the image")
            resolution = self._camera.get_camera_information().camera_configuration.resolution
            rectangle = sl.Rect(int(values[0] * resolution.width), int(values[1] * resolution.height),
                                max(1, int(values[2] * resolution.width)), max(1, int(values[3] * resolution.height)))
        status = self._camera.set_camera_settings_roi(sl.VIDEO_SETTINGS.AEC_AGC_ROI,
                                                      rectangle, sl.SIDE.BOTH, roi is None)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Cannot set automatic-exposure metering region: {status}")
        self.auto_exposure_roi = None if roi is None else list(map(float, roi))
        self._needs_exposure_settle = True

    def _settings(self):
        result = {"mode": "automatic", "metering_roi_normalized": self.auto_exposure_roi,
                  "settling": self._exposure_settling}
        for name in ("EXPOSURE", "GAIN"):
            status, value = self._camera.get_camera_settings(getattr(self._sl.VIDEO_SETTINGS, name))
            if status == self._sl.ERROR_CODE.SUCCESS:
                result[name.lower()] = int(value)
        return result

    def _settle_auto_exposure(self):
        if not self._needs_exposure_settle:
            return
        print("Allowing automatic exposure/gain to adapt to the metering region...", flush=True)
        started = time.monotonic()
        history = deque(maxlen=6)
        views = ("left", "right") if self.view == "stereo" else (self.view,)
        stable = False
        count = 0
        while True:
            self._grab()
            measurements = []
            for view in views:
                frame = self._read_view(view, include_settings=False)
                gray = cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2GRAY)
                if self.auto_exposure_roi is not None:
                    x, y, w, h = self.auto_exposure_roi
                    height, width = gray.shape
                    gray = gray[int(y*height):max(int(y*height)+1, int((y+h)*height)),
                                int(x*width):max(int(x*width)+1, int((x+w)*width))]
                measurements.extend(np.percentile(gray[::4, ::4], [10, 50, 90]).tolist())
            settings = self._settings()
            measurements.extend(settings.get(name, -1) for name in ("exposure", "gain"))
            history.append(measurements)
            count += 1
            elapsed = time.monotonic() - started
            stable = bool(len(history) == history.maxlen and np.max(np.ptp(np.asarray(history), axis=0)) <= 3)
            if (elapsed >= 0.75 and stable) or elapsed >= 3.0:
                break
        self._exposure_settling = {"stable": stable, "frames": count, "duration_s": elapsed}
        self._needs_exposure_settle = False
        print(f"Automatic exposure: {self._settings()}", flush=True)

    def capture(self) -> CameraFrame:
        """Capture one final image after auto exposure adapts; no image averaging."""
        self._settle_auto_exposure()
        self._grab()
        return self._read_view("left" if self.view == "stereo" else self.view)

    def capture_pair(self) -> list[CameraFrame]:
        """Return both rectified eyes from the same hardware grab and timestamp."""
        self._settle_auto_exposure()
        self._grab()
        return [self._read_view("left"), self._read_view("right")]

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
