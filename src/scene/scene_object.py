"""Scene objects share marker geometry and explicit object-relative tool grasps.

SceneObject can describe arbitrary shapes with a measured marker mount and
support-plane coordinate. Cube retains the existing top-marker/top-grasp
shortcuts. Poses, observations and identity come from PhysicalObject.
"""

from dataclasses import dataclass, field
import math
import operator

import numpy as np

from .physical_object import PhysicalObject
from .transforms import transform


@dataclass(kw_only=True, eq=False)
class SceneObject(PhysicalObject):
    """A manipulation object; its origin need not be its geometric center.

    marker_pose is T_object_marker. support_z_m is the resting surface's Z in
    object coordinates; omitted values use the bounding-box bottom, or zero
    when dimensions are unknown. Explicit grasps are T_object_tool.
    """

    grasps: list[np.ndarray] = field(default_factory=list, repr=False)
    manipulation: dict[str, object] = field(default_factory=dict)
    marker_pose: np.ndarray | None = field(default=None, repr=False)
    support_z_m: float | None = None
    marker_family: str = field(default="5x5", init=False)
    marker_dictionary: str | None = None
    marker_id: int | None = None
    marker_size_m: float = 0.040
    marker_center_xy_m: tuple[float, float] = (0.0, 0.0)
    marker_yaw_rad: float | None = None

    def grasp_poses(self) -> list[np.ndarray]:
        """Return configured T_object_tool candidates, without selecting a grasp."""
        if not self.grasps:
            raise ValueError(f"No tool grasp transform configured for {self.object_id}")
        return [transform(pose) for pose in self.grasps]

    @property
    def marker_plane_z_m(self) -> float:
        """Marker plane height in object coordinates, explicit or on the top face."""
        if self.marker_pose is not None:
            return float(transform(self.marker_pose)[2, 3])
        if self.dimensions_m is None:
            raise ValueError("Set marker.pose for an object without top-face dimensions")
        return self.dimensions_m[2] / 2

    def bind_marker(self, marker_id: int, dictionary: str) -> None:
        """Bind one confirmed marker; matching repeats are allowed, rebinding is not.

        The concrete dictionary comes from localization rather than guessing a
        pattern dictionary. This method does not choose among visible candidates.
        StationConfig.save_marker_binding() can persist the resulting identity.
        """
        supported = {"DICT_4X4_50": 50, "DICT_4X4_100": 100,
                     "DICT_4X4_250": 250, "DICT_4X4_1000": 1000,
                     "DICT_5X5_50": 50, "DICT_5X5_100": 100,
                     "DICT_5X5_250": 250, "DICT_5X5_1000": 1000,
                     "DICT_ARUCO_ORIGINAL": 1024}
        if dictionary not in supported:
            raise ValueError("Marker must use DICT_4X4_*, DICT_5X5_* or DICT_ARUCO_ORIGINAL")
        if isinstance(marker_id, (bool, np.bool_)):
            raise ValueError("Marker ID must be an integer")
        try:
            marker_id = operator.index(marker_id)
        except TypeError:
            raise ValueError("Marker ID must be an integer") from None
        if not 0 <= marker_id < supported[dictionary]:
            raise ValueError("Marker ID is outside the reported dictionary")
        if self.marker_id is not None and self.marker_id != marker_id:
            raise ValueError(f"Object is already bound to marker {self.marker_id}")
        if self.marker_dictionary is not None and self.marker_dictionary != dictionary:
            raise ValueError(f"Object is already bound to {self.marker_dictionary}")
        self.marker_id, self.marker_dictionary = marker_id, dictionary
        self.marker_family = "4x4" if dictionary.startswith("DICT_4X4_") else "5x5"

    def marker_mount_transform(self) -> np.ndarray:
        """Return T_object_marker, using an explicit mount or the top-face shortcut."""
        if self.marker_pose is not None:
            return transform(self.marker_pose)
        if self.marker_yaw_rad is None or not math.isfinite(self.marker_yaw_rad):
            raise ValueError("Set marker_yaw_rad relative to the object axes before converting full orientation")
        x, y = self.marker_center_xy_m
        cosine, sine = math.cos(self.marker_yaw_rad), math.sin(self.marker_yaw_rad)
        return transform([[cosine, -sine, 0, x], [sine, cosine, 0, y],
                          [0, 0, 1, self.marker_plane_z_m], [0, 0, 0, 1]])

    def center_from_marker(self, marker_pose: np.ndarray) -> np.ndarray:
        """Return the object origin in the marker observation's reference frame.

        Explicit mounts use the full inverse transform. A centered top marker
        only needs its outward normal and height, without object-edge yaw.
        """
        if self.marker_pose is not None or not np.allclose(self.marker_center_xy_m, (0.0, 0.0), atol=0, rtol=0):
            return self.pose_from_marker(marker_pose)[:3, 3]
        marker_pose = transform(marker_pose)
        return marker_pose[:3, 3] - marker_pose[:3, 2] * self.marker_plane_z_m

    def pose_from_marker(self, marker_pose: np.ndarray) -> np.ndarray:
        """Return T_reference_object = T_reference_marker @ inverse(T_object_marker).

        This pure conversion neither localizes images nor changes object state.
        A six-value pose can first be converted with scene.transforms helpers.
        """
        return transform(transform(marker_pose) @ np.linalg.inv(self.marker_mount_transform()))


@dataclass(kw_only=True, eq=False)
class Cube(SceneObject):
    """A marker-topped cuboid with a geometric-center origin.

    The object origin is its geometric center. X/Y span the square base and
    +Z points toward the top marker. The marker's +Z must point
    outward from that face. Its ID and concrete ArUco dictionary can be
    bound once from a confirmed localization. Marker yaw relative to the
    cube's physical X/Y edges is explicit; centering does not establish yaw.
    Images are localized externally. Top grasps require a measured tool offset.
    """

    dimensions_m: tuple[float, float, float] = (0.04, 0.04, 0.02)
    marker_face: str = field(default="top", init=False)

    def top_grasp(self, tool_to_grasp_m: float, *, grasp_center_z_m: float = 0.0,
                  yaw_rad: float = 0.0) -> np.ndarray:
        """Return T_cube_tool for a downward tool-Z grasp.

        tool_to_grasp_m is the measured distance from DROID's reported tool
        origin to the selected TCP along tool +Z. The TCP is at
        cube (0, 0, grasp_center_z_m); yaw chooses the tool's X/Y orientation.
        This offset must be measured for the actual gripper/tool frame.
        """
        if (not np.isfinite([tool_to_grasp_m, grasp_center_z_m, yaw_rad]).all()
                or tool_to_grasp_m < 0):
            raise ValueError("Grasp offset must be nonnegative and all grasp parameters finite")
        if abs(grasp_center_z_m) > self.dimensions_m[2] / 2:
            raise ValueError("Gripping point must be within the cube height")
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        return transform([[c, s, 0, 0], [s, -c, 0, 0],
                          [0, 0, -1, tool_to_grasp_m + grasp_center_z_m], [0, 0, 0, 1]])

    def grasp_poses(self) -> list[np.ndarray]:
        """Use explicit grasps or build one from measured top-grasp parameters."""
        if self.grasps:
            return super().grasp_poses()
        if self.manipulation.get("tool_to_grasp_m") is None:
            return super().grasp_poses()
        return [self.top_grasp(self.manipulation["tool_to_grasp_m"],
                              grasp_center_z_m=self.manipulation.get("grasp_center_z_m", 0.0),
                              yaw_rad=self.manipulation.get("grasp_yaw_rad", 0.0))]
