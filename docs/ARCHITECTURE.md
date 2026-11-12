# Current architecture

DroidAuto implements scene resetting only. The packages `auto_reset`, `scene`
and `buffer` live directly under `src/`. The application entry point is
`python -m auto_reset`, also used by the Docker image.

## Implementation status

| Area | Current state |
| --- | --- |
| Physical data | Identity, dimensions, copied observed poses, named frames and timestamps. |
| Cube | 40 x 40 x 20 mm, centered 40 mm top marker, DICT_ARUCO_ORIGINAL ID 5, aligned marker/cube edges. |
| Grasp geometry | Explicit object/tool transforms or a downward top grasp from a measured tool offset. |
| Targets | Buffer-relative poses and one-cell placements using object dimensions and a supplied floor height. |
| Planning | Pick/place endpoints; CUDA cuRobo IK with configured joint bounds and arm self-collision; no scene/path collision planner or grasp search. |
| Execution | DROID arm/gripper commands, joint-velocity settling, cooperative cancellation and failure logging. |
| Feedback | Fresh task-object observations compared with targets after each manipulation; completion is measured. |
| Localization | Shared ArUco detector; cube poses fitted from one/stereo images or supplied; fixed buffer pose captured/cached. |
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
reset-result object or driver protocol hierarchy is needed. The `localize_cubes`
image adapter handles marker fitting and diagnostics without owning hardware.

## Frames

All transforms are T_parent_child and translations are metres. Object
observations supplied to the manager are T_base_object. Buffer detection
composes T_base_tool @ T_tool_camera @ T_camera_buffer. Requested poses
are T_buffer_object; grasp transforms are T_object_tool. The tool frame must
match the frame DROID reports. Cell targets put the geometric center half the
object height above the explicitly supplied buffer support plane.

Pure marker-to-cube geometry remains available on `Cube`; a marker center is
10 mm outward from the block center along its local top normal. Full edge-aligned
orientation additionally needs known marker mounting yaw. Object-marker
detection uses the shared detector through `ResetManager.observe(detect_objects=True)`;
buffer poses remain fitted by the existing Buffer methods. Cube pose fitting
jointly minimizes corner reprojection error over the available stereo eyes.
For camera-based resets, the planner first generates lift/transit/scan endpoints
with the optical axes down and the top marker 250 mm ahead of the stereo
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
