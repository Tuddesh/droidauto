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
| Verification | Final supplied observations compared with targets; result retained in manager state. |
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
before that stage begins moving.
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
camera/supplied object poses + detected/cached buffer → planning → executing → awaiting_verification
                                         ↓ supplied final measurements
                                      completed (success true/false)
```

Exceptions produce `failed`; a cooperative cancellation produces `cancelled`.
Pickup poses are invalidated when their transfer starts. Completing motion
does not establish placement success. Final measurements are required for a
verified outcome; no localization output is synthesized from commanded poses.
State is held in memory for the process lifetime; JSONL logs retain events.
The JSON-lines process should remain running between reset and verification.
Camera-based reset requests capture a fresh overview inside the reset operation.
Objects already within the final-placement tolerances are skipped and retain
their observations; only objects with actual transfer actions are invalidated.

The planner assumes a clear transit corridor. It sends endpoints without dense
interpolation or intermediate positional-error checks; DROID joint-space moves
are not guaranteed to follow straight Cartesian segments. Neither collision
avoidance nor recovery from an emergency-stopped server is implemented.

Collision-aware path planning is necessary before unattended resets. It must
model the table, buffer walls/fixtures, Robotiq gripper, camera and carried object,
and check the motion between waypoints as well as the endpoints. Endpoint IK,
arm self-collision checks and fingertip-height clipping do not provide this.
The joint paths sent to DROID must follow the paths checked by the planner.
This requirement is deferred: no collision-planning implementation or scene
collision configuration is provided.
