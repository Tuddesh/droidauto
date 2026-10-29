# Physical scene models

```text
PhysicalObject
├── Buffer
├── BufferItem
│   └── ArucoTile
└── SceneObject
    └── Cube
```

`PhysicalObject` owns `object_id`, `dimensions_m`, `pose`, `pose_frame` and
`observed_at`. `update_pose(pose, reference_frame, timestamp)` copies a rigid
transform and records provenance. `mark_unobserved()` clears the observation.
A pose is T_reference_object in metres. Configured IDs persist across sessions;
a generated UUID only identifies that individual instance. Buffer dimensions
span the marker reference plane and do not model the fixture's solid volume.

`SceneObject` adds object-relative `grasps` and `manipulation` parameters.
`grasp_poses()` returns configured T_object_tool matrices, where tool is the
frame reported by DROID, not an assumed fingertip frame.

## Cube

The block is **40 x 40 x 20 mm**, with a centered **38 mm** marker on top.
Its origin is the geometric center, X/Y follow its base edges, and +Z points
out of the top face. The marker plane is at Z=+10 mm. The station example
records **DICT_ARUCO_ORIGINAL, ID 13**. No image detection is implemented here.

```python
from scene import Cube

cube = Cube(object_id="cube-1")
cube.bind_marker(13, "DICT_ARUCO_ORIGINAL")
```

Marker identity can be persisted through `StationConfig.save_marker_binding`.
Matching repeated bindings are allowed; changing an existing identity is
rejected. DICT_5X5_50/100/250/1000 and DICT_ARUCO_ORIGINAL are supported 5x5
families. Center conversion uses:

```text
cube center = marker position - 0.010 * marker outward Z axis
```

`center_from_marker(T_reference_marker)` performs this geometry without needing
marker yaw. For edge-aligned full orientation, supply `marker_yaw_rad` relative
to the block's edges. Then `marker_mount_transform()` returns T_cube_marker and
`pose_from_marker()` computes T_reference_marker @ inverse(T_cube_marker).
These are pure transforms; they do not localize images or update observations.

## Grasp configuration

An explicit `grasps` list of T_cube_tool matrices (or six-value poses in station
JSON) overrides generated top grasps. Otherwise configure:

```json
{"manipulation":{"tool_to_grasp_m":null,"grasp_center_z_m":0.0,"grasp_yaw_rad":0.0}}
```

Replace null with the **measured** distance from DROID's tool origin to the
gripping point along tool +Z. There is no guessed default for this hardware
measurement. `Cube.top_grasp(tool_to_grasp_m, grasp_center_z_m=0, yaw_rad=0)`
returns a tool-Z-down T_cube_tool with the gripping point at the selected cube
height. The planner uses `grasp_poses()[0]` for both pickup and placement.
The current helper assumes a downward top grasp on an upright cube.

Grasp tool yaw is relative to cube edges, independently of marker mounting yaw.
Scene observations passed directly to the reset stack must already describe
the physical cube frame, rather than an unconverted camera/marker frame.
