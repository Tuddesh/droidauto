"""Fit configured cube markers in rectified camera images and retain diagnostics.

The caller owns capture, robot feedback and the shared OpenCV detector. This
adapter fits one marker pose jointly across the available eyes, then uses the
cube's existing marker-mount conversion to produce its robot-base center pose.
"""

import json
import os

import cv2
import numpy as np
from scipy.optimize import least_squares

from scene import Cube
from scene.transforms import transform, transform_to_pose


def localize_cubes(frames, detector, dictionary_names, objects, base_left_camera,
                   directory, *, review=False):
    """Return (base-frame cube poses, diagnostics); poses=None requests a retake.

Only explicitly bound dictionary/ID pairs are matched. Missing, duplicated or
unfittable markers leave their cube unobserved. Stereo uses both eyes' corners
in one reprojection fit; one visible eye also works. No RMS rejection threshold
is imposed. Inputs and annotated frozen frames are saved for every attempt.
"""
    base_left_camera = transform(base_left_camera)
    directory.mkdir(parents=True, exist_ok=True)
    report = {"directory": str(directory), "base_left_camera": base_left_camera.tolist(),
              "eyes": {}, "objects": {}}
    bindings, samples, poses, overlays = {}, {}, {}, {}
    for obj in objects.values():
        if not isinstance(obj, Cube):
            continue
        key = (obj.marker_dictionary, obj.marker_id)
        if None in key:
            report["objects"][obj.object_id] = {"error": "Cube marker dictionary and ID are not bound"}
            continue
        if key in bindings:
            raise ValueError(f"Two cubes are bound to the same marker: {key}")
        bindings[key] = obj
        samples[obj.object_id] = []
        report["objects"][obj.object_id] = {"dictionary": key[0], "id": key[1],
                                               "error": "Configured marker not detected"}

    for frame in frames:
        rgb = frame.image_rgb
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        corners, ids, rejected, indices = detector.detectMarkersMultiDict(gray)
        annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        eye = {"timestamp_s": frame.timestamp_s, "markers": [],
               "serial": frame.serial, "matrix": frame.camera_matrix.tolist(),
               "distortion": frame.distortion.tolist(),
               "left_camera_eye": None if frame.left_camera_eye is None else frame.left_camera_eye.tolist(),
               "rejected_candidates": len(rejected)}
        report["eyes"][frame.view] = eye
        overlays[frame.view] = annotated
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        left_eye = frame.left_camera_eye
        if left_eye is None:
            if frame.view != "left":
                eye["error"] = "Right eye has no stereo mounting transform"
                continue
            left_eye = np.eye(4)
        camera_left = np.linalg.inv(transform(left_eye))
        for points, marker_id, index in zip(corners, [] if ids is None else ids.ravel(),
                                            [] if indices is None else indices.ravel()):
            dictionary = dictionary_names[int(index)]
            marker_id = int(marker_id)
            points = np.asarray(points, dtype=np.float64).reshape(4, 2)
            eye["markers"].append({"dictionary": dictionary, "id": marker_id,
                                    "corners": points.tolist()})
            location = tuple(np.round(points[0]).astype(int))
            cv2.putText(annotated, f"{dictionary}:{marker_id}", location,
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1, cv2.LINE_AA)
            obj = bindings.get((dictionary, marker_id))
            if obj is not None:
                samples[obj.object_id].append((frame, points, camera_left))

    for key, obj in bindings.items():
        observations = samples[obj.object_id]
        result = report["objects"][obj.object_id]
        if not observations:
            continue
        if len({frame.view for frame, _, _ in observations}) != len(observations):
            result["error"] = "The configured marker appears more than once in an eye"
            continue
        half = obj.marker_size_m / 2
        points = np.array([[-half, half, 0], [half, half, 0],
                           [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
        # Optimize Rodrigues rotation + translation in the left optical frame.
        residual = lambda values: np.concatenate([
            (cv2.projectPoints(
                points, cv2.Rodrigues(camera_left[:3, :3] @ cv2.Rodrigues(values[:3])[0])[0],
                camera_left[:3, :3] @ values[3:] + camera_left[:3, 3],
                frame.camera_matrix, frame.distortion,
            )[0].reshape(4, 2) - measured).ravel()
            for frame, measured, camera_left in observations
        ])
        try:
            seeds = []
            for frame, measured, camera_left in observations:
                success, rotations, translations, _ = cv2.solvePnPGeneric(
                    points, measured, frame.camera_matrix, frame.distortion,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not success:
                    continue
                for rotation, translation in zip(rotations, translations):
                    camera_marker = np.eye(4)
                    camera_marker[:3, :3] = cv2.Rodrigues(rotation)[0]
                    camera_marker[:3, 3] = translation.ravel()
                    left_marker = np.linalg.inv(camera_left) @ camera_marker
                    seeds.append(np.r_[cv2.Rodrigues(left_marker[:3, :3])[0].ravel(),
                                          left_marker[:3, 3]])
            if not seeds:
                raise ValueError("No marker pose could be solved")
            seed = min(seeds, key=lambda values: np.sum(residual(values) ** 2))
            fit = least_squares(residual, seed, method="lm")
            if not fit.success or not np.isfinite(fit.x).all():
                raise ValueError(f"Marker pose fit failed: {fit.message}")
            left_marker = np.eye(4)
            left_marker[:3, :3] = cv2.Rodrigues(fit.x[:3])[0]
            left_marker[:3, 3] = fit.x[3:]
            for frame, _, camera_left in observations:
                camera_marker = camera_left @ left_marker
                depths = (points @ camera_marker[:3, :3].T + camera_marker[:3, 3])[:, 2]
                if np.any(depths <= 0):
                    raise ValueError("Marker pose places corners behind the camera")
            base_marker = base_left_camera @ left_marker
            base_cube = obj.pose_from_marker(base_marker)
            poses[obj.object_id] = base_cube
            errors = residual(fit.x).reshape(-1, 4, 2)
            result.pop("error", None)
            result.update(base_marker=base_marker.tolist(), base_cube=base_cube.tolist(),
                          base_xyz_m=base_cube[:3, 3].tolist(),
                          base_rpy_deg=np.degrees(transform_to_pose(base_cube)[3:]).tolist(),
                          views=[frame.view for frame, _, _ in observations],
                          reprojection_rms_px=float(np.sqrt(np.mean(np.sum(errors ** 2, axis=2)))),
                          per_eye_rms_px={frame.view: float(np.sqrt(np.mean(np.sum(error ** 2, axis=1))))
                                          for (frame, _, _), error in zip(observations, errors)})
            for frame, _, camera_left in observations:
                camera_marker = camera_left @ left_marker
                cv2.drawFrameAxes(overlays[frame.view], frame.camera_matrix, frame.distortion,
                                  cv2.Rodrigues(camera_marker[:3, :3])[0], camera_marker[:3, 3],
                                  obj.marker_size_m / 2)
        except (ValueError, RuntimeError, cv2.error) as exc:
            poses.pop(obj.object_id, None)
            result["error"] = str(exc)

    panels = []
    height = max(frame.image_rgb.shape[0] for frame in frames)
    width = max(frame.image_rgb.shape[1] for frame in frames)
    for frame in frames:
        annotated = overlays[frame.view]
        panel = np.full((height + 160, width, 3), 25, dtype=np.uint8)
        panel[:annotated.shape[0], :annotated.shape[1]] = annotated
        lines = [f"{frame.view.upper()} | FROZEN cube observation",
                 "1/2 eyes | A accept | R retake | P print poses | Q cancel"]
        for object_id, result in report["objects"].items():
            if "error" in result:
                lines.append(f"{object_id}: {result['error']}")
            else:
                xyz = np.round(poses[object_id][:3, 3], 4).tolist()
                lines.append(f"{object_id}: base XYZ {xyz} m | RMS {result['reprojection_rms_px']:.3f} px")
        for index, line in enumerate(lines):
            cv2.putText(panel, line[:200], (12, height + 30 + 34 * index),
                        cv2.FONT_HERSHEY_SIMPLEX, .65, (240, 240, 240), 1, cv2.LINE_AA)
        paths = {}
        for suffix, image in (("input", cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2BGR)),
                              ("fit", annotated), ("review", panel)):
            path = directory / f"{frame.view}_{suffix}.png"
            if not cv2.imwrite(str(path), image):
                raise OSError(f"Could not save object scan image: {path}")
            paths[suffix] = str(path)
        report["eyes"][frame.view]["images"] = paths
        panels.append(panel)
    report_path = directory / "evaluation.json"
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"Cube scan images and evaluation: {directory}", flush=True)
    for object_id, result in report["objects"].items():
        if "error" in result:
            print(f"{object_id}: {result['error']}", flush=True)
        else:
            pose = transform_to_pose(poses[object_id])
            print(f"{object_id}: base XYZ (m) {pose[:3]}, RPY (deg) "
                  f"{np.degrees(pose[3:]).tolist()}, RMS {result['reprojection_rms_px']:.3f} px", flush=True)
    if review:
        title = "DroidAuto - frozen cube scan"
        gui = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")) and bool(cv2.currentUIFramework())
        if gui:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title, 1280, 850)
        selected = 0
        try:
            while True:
                if gui:
                    cv2.imshow(title, panels[selected])
                    key = cv2.waitKey(50) & 0xff
                    choice = chr(key).lower() if key < 128 else ""
                    if key == 27 or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                        choice = "q"
                else:
                    choice = input("Frozen cube scan [a=accept, r=retake, p=print poses, q=cancel]> ").strip().lower()
                if choice in ("1", "2"):
                    selected = min(int(choice) - 1, len(panels) - 1)
                elif choice == "p":
                    print(json.dumps(report["objects"], indent=2), flush=True)
                elif choice == "r":
                    return None, report
                elif choice in ("q", "quit"):
                    raise RuntimeError("Cube scan cancelled by operator")
                elif choice == "a":
                    report["accepted_by_operator"] = True
                    report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
                    break
        finally:
            if gui:
                cv2.destroyWindow(title)
    return poses, report
