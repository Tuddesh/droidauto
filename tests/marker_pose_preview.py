#!/usr/bin/env python3
"""Manual 5x5-marker camera test using the existing ZED and Cube objects.

This diagnostic estimates poses locally for preview only. Production image
localization remains an external service. No DROID connection or robot command
is made. Camera coordinates are +X right, +Y down, +Z forward; dimensions are
metres internally and displayed XYZ is millimetres, RPY is XYZ degrees.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np

from auto_reset.hardware import ZedWristCamera
from scene import Cube
from scene.transforms import transform_to_pose

ROOT = Path(__file__).resolve().parents[1]
DICTIONARIES = ("DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
                "DICT_ARUCO_ORIGINAL")


class MarkerPreview:
    """Diagnostic detector; merge dictionary aliases for the same marker corners."""

    def __init__(self, dictionary="auto", marker_id=None):
        self.marker_id = marker_id
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        names = DICTIONARIES if dictionary == "auto" else (dictionary,)
        self.detectors = [(name, cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name)), parameters)) for name in names]

    def detect(self, image_rgb):
        gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
        detections = []
        for name, detector in self.detectors:
            corners, ids, _ = detector.detectMarkers(gray)
            for points, marker_id in zip(corners, [] if ids is None else ids.ravel()):
                marker_id = int(marker_id)
                if self.marker_id is not None and marker_id != self.marker_id:
                    continue
                points = np.asarray(points, dtype=np.float64).reshape(4, 2)
                duplicate = next((d for d in detections if d["id"] == marker_id
                                  and np.max(np.abs(d["corners"] - points)) < 2), None)
                if duplicate is None:
                    detections.append({"id": marker_id, "corners": points, "dictionaries": [name]})
                else:
                    duplicate["dictionaries"].append(name)
        return detections


def estimate_pose(corners, frame, marker_size_m):
    """Fit square IPPE candidates; return the positive-depth pose with lowest RMS."""
    h = marker_size_m / 2
    objects = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    result = cv2.solvePnPGeneric(objects, corners, frame.camera_matrix, frame.distortion,
                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    if result[0]:
        for rvec, tvec in zip(result[1], result[2]):
            if not np.isfinite(np.r_[rvec.ravel(), tvec.ravel()]).all():
                continue
            rvec, tvec = cv2.solvePnPRefineLM(objects, corners, frame.camera_matrix,
                                            frame.distortion, rvec, tvec)
            rotation = cv2.Rodrigues(rvec)[0]
            if np.any((rotation @ objects.T + tvec.reshape(3, 1))[2] <= 0):
                continue
            projected = cv2.projectPoints(objects, rvec, tvec, frame.camera_matrix, frame.distortion)[0]
            rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - corners)**2, axis=1))))
            pose = np.eye(4)
            pose[:3, :3], pose[:3, 3] = rotation, tvec.ravel()
            candidates.append((rms, pose, rvec, tvec))
    if not candidates:
        raise ValueError("No finite marker pose in front of camera")
    return min(candidates, key=lambda item: item[0])


def render(frame, cube, detector):
    """Outline markers and annotate their camera-relative pose and cube center."""
    image = cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2BGR)
    lines = [f"Camera {frame.serial} / {frame.view.upper()} | marker size {cube.marker_size_m*1000:g} mm",
             "CAMERA coordinates: X right, Y down, Z forward | XYZ mm | XYZ roll/pitch/yaw deg",
             "Q/Esc quit | P print poses | S save image/JSON | R meter selected region | A full-image auto exposure"]
    results = []
    detections = detector.detect(frame.image_rgb)
    for detection in detections:
        points = detection["corners"]
        cv2.polylines(image, [np.rint(points).astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.circle(image, tuple(np.rint(points[0]).astype(int)), 4, (0, 0, 255), -1)
        anchor = (int(max(5, points[:, 0].min())), int(max(25, points[:, 1].min() - 8)))
        cv2.putText(image, f"ID {detection['id']}", anchor, cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2)
        try:
            rms, pose, rvec, tvec = estimate_pose(points, frame, cube.marker_size_m)
            cv2.drawFrameAxes(image, frame.camera_matrix, frame.distortion, rvec, tvec,
                              cube.marker_size_m * .65)
            center = cube.center_from_marker(pose)
            values = transform_to_pose(pose)
            angles = np.rad2deg(values[3:])
            record = {"marker_id": detection["id"], "matching_dictionaries": detection["dictionaries"],
                      "marker_size_m": cube.marker_size_m, "T_camera_marker": pose.tolist(),
                      "marker_xyz_camera_m": values[:3], "marker_rpy_xyz_deg": angles.tolist(),
                      "cube_center_camera_m": center.tolist(), "reprojection_rms_px": rms}
            results.append(record)
            if len(results) <= 5:
                xyz = np.asarray(values[:3]) * 1000
                lines.extend([f"ID {detection['id']} | {', '.join(detection['dictionaries'])}",
                              f"    marker XYZ: {xyz[0]:+.1f}, {xyz[1]:+.1f}, {xyz[2]:+.1f} | RMS {rms:.2f} px",
                              f"    marker RPY: {angles[0]:+.1f}, {angles[1]:+.1f}, {angles[2]:+.1f} | cube center XYZ: "
                              + ", ".join(f"{v:+.1f}" for v in center*1000)])
        except (ValueError, cv2.error) as exc:
            lines.append(f"ID {detection['id']}: pose unavailable: {str(exc).splitlines()[-1]}")
    if not detections:
        lines.append("No marker decoded in the selected dictionaries. Check dictionary, visible border and image quality.")
    elif not results:
        lines.append(f"Decoded {len(detections)} marker(s), but pose estimation failed; see errors above.")
    lines.append("All XYZ estimates assume the displayed marker size. Use --marker-id after identifying the cube.")
    banner = np.full((28 * len(lines) + 12, image.shape[1], 3), 25, dtype=np.uint8)
    for i, line in enumerate(lines):
        cv2.putText(banner, line, (10, 24 + 28*i), cv2.FONT_HERSHEY_SIMPLEX, .53,
                    (235, 235, 235), 1, cv2.LINE_AA)
    payload = {"camera_serial": frame.serial, "view": frame.view, "image_timestamp_s": frame.timestamp_s,
               "camera_matrix": frame.camera_matrix.tolist(), "image_settings": frame.image_settings,
               "markers": results}
    return np.vstack([image, banner]), payload


def save_capture(directory, frame, annotated, payload):
    """Save exact image, annotated preview and measured values under ignored runs/."""
    directory.mkdir(parents=True, exist_ok=True)
    for name, image in (("input.png", cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2BGR)),
                         ("preview.png", annotated)):
        if not cv2.imwrite(str(directory / name), image):
            raise OSError(f"Could not save {directory / name}")
    (directory / "poses.json").write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
    print("Saved camera test:", directory, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = ROOT / "configs/station.local.json"
    parser.add_argument("--config", type=Path, default=default if default.exists() else ROOT / "configs/station.example.json")
    parser.add_argument("--serial", help="Override the configured wrist camera serial")
    parser.add_argument("--view", choices=("left", "right"), default="left")
    parser.add_argument("--dictionary", choices=("auto", *DICTIONARIES), default="auto")
    parser.add_argument("--marker-id", type=int, help="Only show this marker ID")
    parser.add_argument("--frames", type=int, help="Stop after this many frames")
    parser.add_argument("--no-display", action="store_true", help="Save a camera capture without a desktop window")
    args = parser.parse_args()
    if args.frames is not None and args.frames < 1:
        parser.error("--frames must be positive")
    config = json.loads(args.config.read_text())
    cube = Cube(object_id="cube-1")
    serial = args.serial or str(config["camera"]["serial"])
    detector = MarkerPreview(args.dictionary, args.marker_id)
    title = f"DroidAuto marker pose - {serial} {args.view}"
    limit = args.frames if args.frames is not None else (1 if args.no_display else None)
    print(f"Using ZedWristCamera {serial}, {args.view} eye, marker {cube.marker_size_m*1000:g} mm", flush=True)
    print("Camera only: no DROID connection, no robot initialization or movement.", flush=True)
    count = 0
    try:
        with ZedWristCamera(serial, args.view, auto_exposure_roi=config["camera"].get("auto_exposure_roi")) as camera:
            if not args.no_display:
                cv2.namedWindow(title, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(title, 1280, 920)
            while True:
                frame = camera.capture()
                annotated, payload = render(frame, cube, detector)
                count += 1
                key = -1
                if not args.no_display:
                    cv2.imshow(title, annotated)
                    key = cv2.waitKey(1) & 0xff
                if key in (ord("p"), ord("P")):
                    print(json.dumps(payload, indent=2), flush=True)
                if args.no_display or key in (ord("s"), ord("S")):
                    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
                    save_capture(ROOT / "runs/marker_pose_preview" / stamp, frame, annotated, payload)
                    print(json.dumps(payload["markers"], indent=2), flush=True)
                if key in (ord("r"), ord("R")):
                    rectangle = cv2.selectROI("Select auto-exposure region", cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2BGR), False)
                    cv2.destroyWindow("Select auto-exposure region")
                    x, y, w, h = rectangle
                    if w and h:
                        height, width = frame.image_rgb.shape[:2]
                        camera.set_auto_exposure_roi([x/width, y/height, w/width, h/height])
                if key in (ord("a"), ord("A")):
                    camera.set_auto_exposure_roi(None)
                if key in (27, ord("q"), ord("Q")) or limit is not None and count >= limit:
                    break
                if not args.no_display and cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
