"""Native-resolution rectified ZED images from one explicitly selected camera."""

from dataclasses import dataclass

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


class ZedWristCamera:
    """The lab PC needs the installed ZED SDK's Python bindings in its own env."""

    def __init__(self, serial: str, view: str = "left"):
        if view not in ("left", "right", "stereo"):
            raise ValueError("view must be left, right or stereo")
        self.serial, self.view = str(serial), view
        self._camera = None

    def __enter__(self):
        try:
            import pyzed.sl as sl
        except ImportError as exc:
            raise RuntimeError("Install the lab PC's ZED SDK Python bindings into DroidAuto's .conda env") from exc
        self._sl = sl
        camera = sl.Camera()
        params = sl.InitParameters()
        params.set_from_serial_number(int(self.serial))
        params.camera_resolution = sl.RESOLUTION.HD720
        params.camera_fps = 30
        params.depth_mode = sl.DEPTH_MODE.NONE
        params.camera_image_flip = sl.FLIP_MODE.OFF
        status = camera.open(params)
        if status != sl.ERROR_CODE.SUCCESS:
            camera.close()
            raise RuntimeError(f"Cannot open ZED {self.serial}: {status}")
        self._camera = camera
        return self

    def capture(self) -> CameraFrame:
        """Grab exactly one image. No averaging or multi-image buffer calibration."""
        self._grab()
        return self._read_view("left" if self.view == "stereo" else self.view)

    def capture_pair(self) -> list[CameraFrame]:
        """Return both rectified eyes from the same hardware grab and timestamp."""
        self._grab()
        return [self._read_view("left"), self._read_view("right")]

    def _grab(self):
        if self._camera is None:
            raise RuntimeError("Open the camera with a context manager first")
        sl, camera = self._sl, self._camera
        status = camera.grab(sl.RuntimeParameters())
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED capture failed: {status}")

    def _read_view(self, view) -> CameraFrame:
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
        return CameraFrame(rgb.copy(), matrix, np.zeros(5), stamp, self.serial, view)

    def __exit__(self, *_):
        if self._camera is not None:
            self._camera.close()
            self._camera = None
