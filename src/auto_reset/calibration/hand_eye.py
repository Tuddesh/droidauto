"""ChArUco observations and camera-to-tool calibration; no hardware startup."""

import itertools

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from scene.transforms import pose_error, transform, transform_to_pose


class CharucoObservation:
    """Detect a known printed board in rectified left-camera images."""

    def __init__(self, columns=12, rows=9, square_m=0.030, marker_m=0.022,
                 dictionary="DICT_5X5_100", legacy=False, min_corners=12, max_rms_px=2.0):
        if columns < 3 or rows < 3 or not 0 < marker_m < square_m:
            raise ValueError("Expected at least 3x3 squares and 0 < marker size < square size")
        if not dictionary.startswith("DICT_5X5_") or not hasattr(cv2.aruco, dictionary):
            raise ValueError("Select an OpenCV DICT_5X5_* dictionary matching the printed board")
        self.board = cv2.aruco.CharucoBoard(
            (columns, rows), square_m, marker_m,
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)))
        self.board.setLegacyPattern(legacy)
        self.min_corners, self.max_rms_px = min_corners, max_rms_px
        self.specification = dict(columns=columns, rows=rows, square_m=square_m,
                                  marker_m=marker_m, dictionary=dictionary, legacy=legacy)

    def detect(self, frame):
        params = cv2.aruco.CharucoParameters()
        params.cameraMatrix = frame.camera_matrix
        params.distCoeffs = frame.distortion
        detector_params = cv2.aruco.DetectorParameters()
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.CharucoDetector(self.board, params, detector_params)
        image = cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2BGR)
        corners, ids, markers, marker_ids = detector.detectBoard(image)
        drawn = image.copy()
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(drawn, markers, marker_ids)
        count = 0 if ids is None else len(ids)
        info = {"corner_count": count, "marker_ids": [] if marker_ids is None else marker_ids.ravel().tolist(),
                "accepted": False, "reason": f"Need {self.min_corners} ChArUco corners; found {count}"}
        if count:
            cv2.aruco.drawDetectedCornersCharuco(drawn, corners, ids)
        if count < self.min_corners:
            return drawn, info
        if self.board.checkCharucoCornersCollinear(ids):
            info["reason"] = "Detected board corners are collinear"
            return drawn, info
        obj, pixels = self.board.matchImagePoints(corners, ids)
        ok, rvec, tvec = cv2.solvePnP(obj, pixels, frame.camera_matrix, frame.distortion)
        if not ok:
            info["reason"] = "Board pose estimation failed"
            return drawn, info
        rvec, tvec = cv2.solvePnPRefineLM(obj, pixels, frame.camera_matrix, frame.distortion, rvec, tvec)
        camera_board = np.eye(4)
        camera_board[:3, :3] = cv2.Rodrigues(rvec)[0]
        camera_board[:3, 3] = tvec.ravel()
        projected = cv2.projectPoints(obj, rvec, tvec, frame.camera_matrix, frame.distortion)[0]
        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - pixels.reshape(-1, 2))**2, axis=1))))
        in_front = np.all((camera_board[:3, :3] @ obj.reshape(-1, 3).T + tvec.reshape(3, 1))[2] > 0)
        info.update(camera_board=camera_board.tolist(), reprojection_rms_px=rms,
                    charuco_ids=ids.ravel().tolist(), corners_px=corners.reshape(-1, 2).tolist())
        cv2.drawFrameAxes(drawn, frame.camera_matrix, frame.distortion, rvec, tvec, 0.06)
        info["accepted"] = bool(in_front and np.isfinite(rms) and rms <= self.max_rms_px)
        info["reason"] = f"{count} corners; RMS {rms:.2f} px" + ("" if info["accepted"] else " — rejected")
        return drawn, info


def _mean_pose(poses):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_matrix(np.array([p[:3, :3] for p in poses])).mean().as_matrix()
    result[:3, 3] = np.mean([p[:3, 3] for p in poses], axis=0)
    return result


def _vector(pose):
    return np.r_[pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_rotvec()]


def _matrix(vector):
    pose = np.eye(4)
    pose[:3, 3] = vector[:3]
    pose[:3, :3] = Rotation.from_rotvec(vector[3:]).as_matrix()
    return pose


def _fit(base_tools, camera_boards):
    candidates = []
    for method in (cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_TSAI):
        try:
            rotation, translation = cv2.calibrateHandEye(
                [p[:3, :3] for p in base_tools], [p[:3, 3] for p in base_tools],
                [p[:3, :3] for p in camera_boards], [p[:3, 3] for p in camera_boards], method=method)
            mount = np.eye(4)
            mount[:3, :3], mount[:3, 3] = rotation, translation.ravel()
            mount = transform(mount)
        except (cv2.error, ValueError):
            continue
        fixed = _mean_pose([a @ mount @ b for a, b in zip(base_tools, camera_boards)])

        def residual(vector):
            x, y = _matrix(vector[:6]), _matrix(vector[6:])
            result = []
            for a, b in zip(base_tools, camera_boards):
                predicted = a @ x @ b
                result.extend((predicted[:3, 3] - y[:3, 3]) / 0.005)
                result.extend(Rotation.from_matrix(y[:3, :3].T @ predicted[:3, :3]).as_rotvec() / np.deg2rad(1))
            return result

        fit = least_squares(residual, np.r_[_vector(mount), _vector(fixed)],
                            loss="soft_l1", max_nfev=250)
        if fit.success and np.isfinite(fit.x).all():
            candidates.append((float(np.mean(np.square(residual(fit.x)))),
                               _matrix(fit.x[:6]), _matrix(fit.x[6:])))
    if not candidates:
        raise ValueError("Hand-eye solver failed; capture more varied wrist rotations")
    _, mount, fixed = min(candidates, key=lambda item: item[0])
    return mount, fixed


def solve_hand_eye(samples, min_samples=8):
    """Inputs base<-tool and camera<-board; output tool<-left optical camera.

    The same stationary board must be observed from every wrist pose. No
    samples are silently removed. Leave-one-out results assess sensitivity.
    """
    if len(samples) < min_samples:
        raise ValueError(f"Need at least {min_samples} captures; have {len(samples)}. Aim for 15–25.")
    base_tools = [transform(s["base_tool"]) for s in samples]
    camera_boards = [transform(s["observation"]["camera_board"]) for s in samples]
    rotations = np.array([Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).as_rotvec()
                          for a, b in itertools.combinations(base_tools, 2)])
    singular = np.linalg.svd(rotations, compute_uv=False) / np.sqrt(len(rotations))
    if singular[1] < np.deg2rad(8):
        raise ValueError("Insufficient rotation about different axes. Tilt wrist in two different directions, not just translate or turn about one axis.")
    mount, fixed = _fit(base_tools, camera_boards)
    errors = np.array([pose_error(a @ mount @ b, fixed) for a, b in zip(base_tools, camera_boards)])
    held_out, mount_variation = [], []
    for index in range(len(samples)):
        keep = [i for i in range(len(samples)) if i != index]
        try:
            x, y = _fit([base_tools[i] for i in keep], [camera_boards[i] for i in keep])
            held_out.append(pose_error(base_tools[index] @ x @ camera_boards[index], y))
            mount_variation.append(pose_error(x, mount))
        except (ValueError, cv2.error):
            raise ValueError("Leaving out one capture makes calibration unsolvable; collect more diverse poses") from None
    held_out, mount_variation = np.array(held_out), np.array(mount_variation)
    quality = {
        "sample_count": len(samples), "rotation_excitation_rms_deg": np.rad2deg(singular).tolist(),
        "board_position_rms_m": float(np.sqrt(np.mean(errors[:, 0]**2))),
        "board_rotation_rms_deg": float(np.rad2deg(np.sqrt(np.mean(errors[:, 1]**2)))),
        "board_position_max_m": float(errors[:, 0].max()),
        "board_rotation_max_deg": float(np.rad2deg(errors[:, 1].max())),
        "held_out_position_rms_m": float(np.sqrt(np.mean(held_out[:, 0]**2))),
        "held_out_rotation_rms_deg": float(np.rad2deg(np.sqrt(np.mean(held_out[:, 1]**2)))),
        "leave_one_out_mount_position_max_m": float(mount_variation[:, 0].max()),
        "leave_one_out_mount_rotation_max_deg": float(np.rad2deg(mount_variation[:, 1].max())),
        "per_sample": [{"index": i + 1, "board_position_error_m": float(e[0]),
                        "board_rotation_error_deg": float(np.rad2deg(e[1])),
                        "held_out_position_error_m": float(h[0]),
                        "held_out_rotation_error_deg": float(np.rad2deg(h[1]))}
                       for i, (e, h) in enumerate(zip(errors, held_out))],
    }
    # Consistency thresholds are practical acceptance criteria, not a claim of
    # absolute accuracy. Shared robot/intrinsic/print errors can remain hidden.
    quality["passed"] = bool(quality["held_out_position_rms_m"] <= 0.010
                             and quality["held_out_rotation_rms_deg"] <= 2.0
                             and quality["leave_one_out_mount_position_max_m"] <= 0.020
                             and quality["leave_one_out_mount_rotation_max_deg"] <= 3.0)
    return {"tool_left_camera": mount.tolist(), "tool_left_camera_pose": transform_to_pose(mount),
            "base_board": fixed.tolist(), "quality": quality,
            "convention": "T_tool_left_camera maps rectified left optical coordinates into DROID's reported end-effector frame; metres, fixed-axis XYZ radians"}
