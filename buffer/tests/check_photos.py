"""Inspect buffer photos; run as ``python -m buffer.tests.check_photos``."""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from buffer import Buffer, LocalizationError
from buffer.items import ArucoTile


CONFIG_PATH = Path(__file__).with_name("photo_cases.json")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".webp"}


def load_cases(path: Path = CONFIG_PATH) -> dict:
    """Read layouts and optional per-photo settings from JSON."""
    with path.open(encoding="utf-8") as file:
        return json.load(file)


def discover_photo_cases(config: dict, config_dir: Path) -> list[dict]:
    """Treat every image in a layout-named folder as a case for that layout."""
    cases = []
    overrides = config.get("photo_overrides", {})
    for layout_name in config["layouts"]:
        folder = config_dir / "photos" / layout_name
        if not folder.is_dir():
            continue
        for path in sorted(folder.iterdir()):
            if not path.is_file():
                continue
            if path.suffix.lower() in {".heic", ".heif"}:
                raise ValueError(f"Convert HEIC/HEIF to JPEG or PNG first: {path}")
            if path.suffix.lower() not in IMAGE_EXTENSIONS:
                continue
            relative_path = path.relative_to(config_dir).as_posix()
            case = {"image": relative_path, "layout": layout_name}
            case.update(overrides.get(relative_path, {}))
            case["layout"] = layout_name
            cases.append(case)
    return cases


def check_photo(case: dict, layouts: dict, config_dir: Path) -> dict:
    """Report detections and, when calibrated, buffer pose and reprojection error."""
    layout = layouts[case["layout"]]
    image_path = config_dir / case["image"]
    image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise FileNotFoundError(f"Could not read photo: {image_path}")
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, layout["dictionary"])
    )
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY))
    detected_ids = [] if ids is None else [int(value) for value in ids.reshape(-1)]
    marker_by_id = {marker["id"]: marker for marker in layout["markers"]}
    known_ids = sorted(set(detected_ids) & marker_by_id.keys())
    expected_ids = set(case.get("expected_visible_ids", []))
    missing_ids = sorted(expected_ids - set(detected_ids))
    issues = []
    if not known_ids:
        issues.append("No configured marker was detected")
    if missing_ids:
        issues.append(f"Expected marker IDs not detected: {missing_ids}")

    report = {
        "image": case["image"],
        "layout": case["layout"],
        "image_size_px": [image_bgr.shape[1], image_bgr.shape[0]],
        "detected_ids": detected_ids,
        "known_visible_ids": known_ids,
        "unconfigured_detected_ids": sorted(set(detected_ids) - marker_by_id.keys()),
        "issues": issues,
        "planar_reprojection_rms_px": None,
        "pose_camera_buffer": None,
        "reprojection_rms_px": None,
    }

    object_points = []
    image_points = []
    half_size_m = layout["marker_size_mm"] / 2000.0
    offsets = np.array(
        [
            [-half_size_m, half_size_m, 0],
            [half_size_m, half_size_m, 0],
            [half_size_m, -half_size_m, 0],
            [-half_size_m, -half_size_m, 0],
        ]
    )
    for detected_id, detected_corners in zip(detected_ids, corners):
        marker = marker_by_id.get(detected_id)
        if marker is None:
            continue
        center = np.array([
            (marker["column"] - layout["origin"][0] + 0.5) * layout["cell_size_mm"] / 1000,
            (marker["row"] - layout["origin"][1] + 0.5) * layout["cell_size_mm"] / 1000,
            0.0,
        ])
        object_points.extend(center + offsets)
        image_points.extend(np.asarray(detected_corners).reshape(4, 2))
    object_points = np.asarray(object_points, dtype=np.float64)
    image_points = np.asarray(image_points, dtype=np.float64)

    if len(known_ids) >= 2:
        homography, _ = cv2.findHomography(object_points[:, :2], image_points)
        if homography is None:
            issues.append("Could not fit detected markers to the configured grid")
        else:
            projected = cv2.perspectiveTransform(
                object_points[:, :2].reshape(-1, 1, 2), homography
            ).reshape(-1, 2)
            residual = projected - image_points
            planar_rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
            report["planar_reprojection_rms_px"] = planar_rms
            if (
                "max_planar_reprojection_error_px" in case
                and planar_rms > case["max_planar_reprojection_error_px"]
            ):
                issues.append(
                    f"Planar reprojection RMS {planar_rms:.2f} px exceeds "
                    f"{case['max_planar_reprojection_error_px']} px"
                )

    camera_matrix = case.get("camera_matrix")
    if camera_matrix is None:
        report["mode"] = "detection_only_no_camera_calibration"
        return report

    if "calibration_image_size_px" not in case:
        raise ValueError("calibration_image_size_px is required with camera_matrix")
    if case["calibration_image_size_px"] != report["image_size_px"]:
        raise ValueError("calibration_image_size_px does not match the photo")

    grid = Buffer(
        columns=layout["columns"],
        rows=layout["rows"],
        camera_matrix=np.asarray(camera_matrix, dtype=np.float64),
        distortion_coefficients=case.get("distortion_coefficients"),
        cell_size_mm=layout["cell_size_mm"],
        origin=tuple(layout["origin"]),
        dictionary=layout["dictionary"],
    )
    for marker in layout["markers"]:
        grid.place(
            ArucoTile(marker["id"], layout["marker_size_mm"], layout["dictionary"]),
            marker["column"],
            marker["row"],
        )

    report["mode"] = "pose"
    try:
        pose = grid.localize(image_rgb)
    except LocalizationError as error:
        issues.append(str(error))
        return report

    report["pose_camera_buffer"] = pose.tolist()
    rotation_vector = cv2.Rodrigues(pose[:3, :3])[0]
    projected = cv2.projectPoints(
        object_points,
        rotation_vector,
        pose[:3, 3],
        grid.camera_matrix,
        grid.distortion_coefficients,
    )[0].reshape(-1, 2)
    residual = projected - image_points
    rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    report["reprojection_rms_px"] = rms
    if "max_reprojection_error_px" in case and rms > case["max_reprojection_error_px"]:
        issues.append(
            f"Reprojection RMS {rms:.2f} px exceeds "
            f"{case['max_reprojection_error_px']} px"
        )
    return report


def main() -> None:
    config = load_cases()
    cases = discover_photo_cases(config, CONFIG_PATH.parent)
    if not cases:
        print("No photos found in buffer/tests/photos/<layout-name>/ yet.")
        return
    failed = False
    for case in cases:
        report = check_photo(case, config["layouts"], CONFIG_PATH.parent)
        print(json.dumps(report, indent=2))
        failed |= bool(report["issues"])
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
