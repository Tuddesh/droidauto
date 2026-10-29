# Physical scene models

This package prepares the shared object model for automatic scene reset.

```text
PhysicalObject (dataclass)
├── Buffer                         buffer/buffer.py
├── BufferItem                     buffer/buffer_item.py
│   └── ArucoTile                   buffer/items/aruco_tile.py
└── SceneObject                    scene_object.py
    └── Cube                       scene_object.py
```

## Shared parameters

`PhysicalObject` owns `object_id`, `dimensions_m`, `pose`, `pose_frame`, and
`observed_at`. Subclasses add their own parameters rather than another identity
or observed pose. Supply a configured `object_id` for identity across sessions;
the generated default UUID only identifies a newly constructed instance.

`pose` means `T_reference_object`: it maps object coordinates into the frame
named by `pose_frame`. Translation is in metres. Buffer observations may be
expressed in the robot base frame, while task-object observations may use the
named buffer frame. A matrix without its reference frame is not a complete
observation. `dimensions_m=None` means physical extents are unspecified.

Buffer's dimensions describe its grid reference-plane extent, with zero Z
thickness; they do not model the fixture's solid volume. BufferItem adds the
integer cell footprint. Its read-only `item_id` property is a compatibility
alias for the inherited `object_id`, a string.
Buffer placement uses `object_id` directly.

`SceneObject` adds object-relative grasp frames and manipulation parameters.
`Cube` defines the project's **40 x 40 x 20 mm** manipulation block using
inherited `dimensions_m=(0.04, 0.04, 0.02)`. Its origin is the geometric center,
X/Y follow the square base, and +Z points toward the top face. The top marker
plane is at `marker_plane_z_m=0.01` relative to that origin.

```python
from scene import Cube

cube = Cube(object_id="cube-1")
```

The marker is **38 mm wide**, centered on the top face. Defaults are
`marker_size_m=0.038`, `marker_center_xy_m=(0, 0)` and `marker_family="5x5"`.
Its numeric ID and concrete dictionary remain unset until a confirmed
localization supplies them. Being centered does not determine its printed yaw
relative to the physical block edges; `marker_yaw_rad` remains explicit.

## Marker binding and conversion

```python
# Bind the ID/dictionary reported by the localization service once the cube
# has been identified. No image detector is implemented in the object model.
cube.bind_marker(marker_id, dictionary)

# Persist it in the station's local config for subsequent launches.
station_config.save_marker_binding(cube.object_id, marker_id, dictionary)

# T_reference_marker: origin at marker center, +Z out of the top face.
center = cube.center_from_marker(marker_pose)
```

A matching repeated binding is allowed. A different ID/dictionary is rejected
rather than silently changing the physical object's identity. The concrete
`DICT_5X5_50`, `_100`, `_250`, or `_1000` dictionary comes from the localization
integration; the code does not guess a dictionary size or select the first
marker seen in the scene. Binding is in-memory on the cube; saving it through
`StationConfig` explicitly persists it. Runtime object loading remains pending.

The centered-marker conversion is:

```text
cube center = marker position - 0.010 * marker outward Z axis
```

This follows the marker normal when the block is tilted. It does not subtract
10 mm along an unrelated camera/world Z axis. Center conversion does not
require knowing marker yaw relative to the block edges.

For a full cube pose with axes along the physical block edges, configure the
marker mounting yaw (radians, measured about cube +Z):

```python
cube.marker_yaw_rad = known_marker_yaw_relative_to_cube
cube_pose = cube.pose_from_marker(marker_pose)
```

`marker_mount_transform()` returns `T_cube_marker` and `pose_from_marker()`
computes `T_reference_marker @ inverse(T_cube_marker)`. Full orientation
conversion requires a specified yaw and does not silently assume alignment.
The marker coordinate convention must match the colleague's output; the
[OpenCV marker-pose documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
describes the usual centered marker frame. Six-value observations can be
converted to matrices with `scene.transforms.pose_to_transform()` when their
rotation convention is fixed-axis XYZ radians.

These conversion methods are pure geometry. They do not run localization,
update the object's observed pose, generate grasps or command hardware.

## Prepared interfaces

- `PhysicalObject.update_pose(pose, reference_frame, timestamp)`
- `PhysicalObject.mark_unobserved()`
- `SceneObject.grasp_poses()`

The observation/grasp methods intentionally raise `NotImplementedError`.
The concrete supplied-tool adapter API is deferred until integration.
`Buffer.localize()` and
`Buffer.localize_stereo()` retain their existing implementations and ownership.
They return camera-relative estimates; future manager observation code will
compose them into the desired reference frame and update shared object state.

Desired placements belong to `auto_reset.TargetConfiguration`; they do not
overwrite the objects' observed poses. See the [reset interfaces](../auto_reset/README.md).
