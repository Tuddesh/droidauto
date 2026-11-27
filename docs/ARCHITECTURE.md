# Current architecture

DroidAuto implements scene resetting only. The packages `auto_reset`, `scene`
and `buffer` live directly under `src/`. The application entry point is
`python -m auto_reset`, also used by the Docker image.

## Implementation status

| Area | Current state |
| --- | --- |
| Physical data | Identity, dimensions, copied observed poses, named frames and timestamps. |
| Objects | SceneObject owns marker mounts and explicit grasps; Cube retains top-face shortcuts. Example: 33 mm cube / ID 7 and rim-grasped bowl / ID 36, both DICT_4X4_50. |
| Grasp geometry | Explicit object/tool transforms or a downward top grasp from a measured tool offset. |
| Targets | Buffer-relative poses and cell regions; BufferItem footprints reserve multiple cells and permit explicit overhang. |
| Planning | Pick/place endpoints; CUDA cuRobo IK with configured joint bounds and arm self-collision; no scene/path collision planner or grasp search. |
| Execution | DROID arm/gripper commands, joint-velocity settling, cooperative cancellation and failure logging. |
| Feedback | Fresh task-object observations compared with targets after each manipulation; completion is measured. |
| Localization | Shared ArUco detector; object poses fitted from one/stereo images or supplied; fixed buffer pose captured/cached. |
| Existing utilities | Buffer ArUco/stereo localization, hand-eye/buffer calibration and independent ZED acquisition. |
| Startup | JSON document/JSON-lines requests; optional DROID connection and explicit controller initialization. |
| Container | Headless Python runtime; DROID is external, camera SDK is not included. |

## Object ownership

```text
PhysicalObject
├── Buffer
├── BufferItem
│   └── ArucoTile
└── SceneObject
    └── Cube

ResetManager
├── Buffer and registered SceneObjects
├── TargetConfiguration
├── ResetPlanner
├── DroidRobot (tool-state read for capture; motion only for execution)
├── ZedWristCamera and StationConfig (buffer and object acquisition)
├── BufferCalibration (optional saved buffer pose)
└── EventLog (optional)
```

Objects own physical data and grasp descriptions. `Buffer` owns grid geometry,
fixtures and its independent localization methods. Desired placements belong
to `TargetConfiguration`, separately from measured poses. `ResetPlanner`
composes grasp/endpoints; `DroidRobot` solves IK locally with cuRobo on CUDA and
delegates joint execution to DROID. Each stage's endpoint joints are solved
before that stage moves.
The manager owns sequencing, final verification and outcome state. No separate
reset-result object or driver protocol hierarchy is needed. The `localize_objects`
image adapter handles marker fitting and diagnostics without owning hardware.

## Frames

All transforms are T_parent_child and translations are metres. Object
observations supplied to the manager are T_base_object. Buffer detection
composes T_base_tool @ T_tool_camera @ T_camera_buffer. Requested poses
are T_buffer_object; grasp transforms are T_object_tool. The tool frame must
match the frame DROID reports. Cell targets place the object origin using its support-plane coordinate,
deriving floor height from the planar station geometry when available.

With `planar_scene`, Buffer.localize_planar() projects marker corners onto the
configured buffer marker plane and fits X/Y/yaw. The example table is at base
Z = −40 mm, the buffer marker plane at +20 mm, and its support surface at
+8 mm. Independently rotated buffer markers are matched by their centers.
Object localization projects corners onto each object's marker plane above
the table or buffer support surface. Marker mounts and support coordinates
come from SceneObject, so an inside-bottom bowl marker is not treated as a
cube's top face. Planar fitting requires upright objects with upward-facing
horizontal markers. Both stereo eyes share one detector; either eye alone
also works. Free-3D fitting remains available without a planar model.

`SceneObject.pose_from_marker()` converts the fitted marker pose through
T_object_marker. Supplied observations and targets use the same support
geometry. Cached calibration and taught scenes must match the station's
height model. The existing manager and planner handle generic SceneObjects
through their object-relative grasp transforms.
For camera-based resets, the planner first generates lift/transit/scan endpoints
with the optical axes down and the object marker 250 mm ahead of the stereo
midpoint. The measured camera-to-tool mount determines the flange pose. The
manager captures a fresh observation there and replans the transfer. An unusable
close-up falls back to the coarse pose with a logged warning. Supplied-pose
resets keep the direct transfer path.

## Reset lifecycle

```text
observe scene → compare with target → choose one off-target object → plan/execute manipulation
      ↑                                                                  ↓
      └────────────────── return to observation pose ─────────────────────┘
compare finds all requested objects within tolerance → completed (success true)
```

Exceptions produce `failed`; a cooperative cancellation produces `cancelled`.
Task-object poses are invalidated when a transfer starts. Every completed
manipulation is followed by another scene scan; commanded targets never become
observations. The first observed off-target object in target order is selected
again from each new scene. Missing detections cannot establish success. With no
observed off-target object, the loop rescans until objects are visible or cancelled.
There is no fixed retry limit. Controller/capture exceptions terminate as failures.
State is held in memory for the process lifetime; JSONL logs retain events.
The JSON-lines process retains the camera and manager between reset requests.

The planner assumes a clear transit corridor. It sends endpoints without dense
interpolation or intermediate positional-error checks; DROID joint-space moves
are not guaranteed to follow straight Cartesian segments. Neither collision
avoidance nor recovery from an emergency-stopped server is implemented.
Collision-aware path planning is required before unattended operation. It must
include the table, buffer, gripper, camera and carried object, and validate the
joint paths actually executed between endpoints. This remains unimplemented
and unconfigured.
