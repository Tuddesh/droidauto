"""Rigid transforms in metres; DROID poses use fixed-axis XYZ Euler angles."""

import cv2
import numpy as np


def transform(value) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise ValueError("Expected a finite 4x4 transform")
    rotation = result[:3, :3]
    if (not np.allclose(result[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
        raise ValueError("Expected a rigid, right-handed transform")
    return result.copy()


def pose_to_transform(pose) -> np.ndarray:
    values = np.asarray(pose, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("Expected [x, y, z, roll, pitch, yaw]")
    roll, pitch, yaw = values[3:]
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = values[:3]
    return result


def transform_to_pose(value) -> list[float]:
    matrix = transform(value)
    rotation = matrix[:3, :3]
    pitch = np.arctan2(-rotation[2, 0], np.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(np.cos(pitch)) > 1e-8:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return [*matrix[:3, 3].tolist(), float(roll), float(pitch), float(yaw)]


def pose_error(actual, target) -> tuple[float, float]:
    actual, target = transform(actual), transform(target)
    angle = cv2.Rodrigues(actual[:3, :3].T @ target[:3, :3])[0]
    return float(np.linalg.norm(actual[:3, 3] - target[:3, 3])), float(np.linalg.norm(angle))


def interpolate(start, end, fraction: float) -> np.ndarray:
    """Linear translation and shortest rotation interpolation."""
    start, end = transform(start), transform(end)
    result = np.eye(4)
    rotation_vector = cv2.Rodrigues(start[:3, :3].T @ end[:3, :3])[0]
    result[:3, :3] = start[:3, :3] @ cv2.Rodrigues(rotation_vector * fraction)[0]
    result[:3, 3] = start[:3, 3] * (1 - fraction) + end[:3, 3] * fraction
    return result
