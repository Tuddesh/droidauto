# Physical scene models

`PhysicalObject` owns identity, optional dimensions and the observed pose/frame/
timestamp. `Buffer`, `BufferItem` and `SceneObject` inherit that data.

`SceneObject` describes a manipulation object without assuming a shape. It owns
marker geometry and explicit `grasps` (T_object_tool). `Cube` adds the existing
geometric-center/top-marker and generated top-grasp shortcuts. There is no
separate bowl class.

## Frames and geometry

All transforms are T_parent_child in metres. Configured poses accept a 4×4
matrix or `[x,y,z,roll,pitch,yaw]` with fixed-axis XYZ radians.

- `marker.pose` is T_object_marker. It can describe an offset or rotated mount.
- `support_z_m` is the support surface's Z in object coordinates. If omitted,
  it is minus half the object height when dimensions are known, otherwise zero.
- `grasps` are T_object_tool, using DROID's reported flange (`panda_link8`).
  They already include the flange-to-gripping-point offset.
- Cube configurations can keep `marker.center_xy_m` and `marker.yaw_rad`
  instead of `marker.pose`, and generate a top grasp from
  `manipulation.tool_to_grasp_m` (currently 167 mm).

The existing `bind_marker`, `marker_mount_transform`, `marker_plane_z_m`,
`center_from_marker` and `pose_from_marker` now belong to `SceneObject` and
remain inherited by `Cube`. Marker binding supports DICT_4X4_*, DICT_5X5_*
and DICT_ARUCO_ORIGINAL. A marker's mounting yaw must be known to recover
object-edge orientation; centering alone does not establish it.

```text
T_reference_object = T_reference_marker @ inverse(T_object_marker)
pickup tool pose = T_base_object @ T_object_tool
```

The shared localizer uses these mounts for both 3D and planar fitting. Planar
objects must rest upright with a horizontal, upward-facing marker; its height
is `marker_plane_z_m - support_z_m` above the support surface.

## Configured scene

[The station example](../../configs/station.example.json) defines:

| Object | Geometry and marker | Grasp |
| --- | --- | --- |
| `cube-7` (`cube`) | 33 mm cube; centered 25 mm DICT_4X4_50 / 7 top marker | Downward grasp at geometric center; marker/edge alignment assumed 0°. |
| `bowl-36` (`object`) | Origin at the outside support center; centered 35 mm DICT_4X4_50 / 36 marker, 4 mm above it | Rim point 70 mm above marker and 40 mm radially beyond its +X/+Y corner. |

The bowl grasp radius is 64.749 mm; its height is 74 mm above the support.
The explicit flange pose includes the 167 mm tool offset. Tool X is set radial
(yaw 45° relative to the marker); its agreement with the physical gripper's
closing axis remains to be checked. Bowl exterior dimensions have not been
invented from the grasp radius.

[The target example](../../configs/reset.example.json) puts `cube-7` in cell
(1,3), and centers `bowl-36` on the 2×2 region beginning at (4,1). Explicit
`size_cells` reservations allow physical overhang; they do not model collisions.
Cell targets derive support height from `planar_scene`, or accept an explicit
`support_height_m` in the buffer-marker frame when no planar model is used.
