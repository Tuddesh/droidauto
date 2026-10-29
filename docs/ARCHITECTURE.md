# Current architecture

DroidAuto implements scene resetting only. The packages `auto_reset`, `scene`
and `buffer` live directly under `src/`. The application entry point is
`python -m auto_reset`, also used by the Docker image.

## Implementation status

| Area | Current state |
| --- | --- |
| Physical data | Identity, dimensions, copied observed poses, named frames and timestamps. |
| Cube | 40 x 40 x 20 mm, centered 38 mm top marker, DICT_ARUCO_ORIGINAL ID 13 in station config. |
| Grasp geometry | Explicit object/tool transforms or a downward top grasp from a measured tool offset. |
| Targets | Buffer-relative poses and one-cell placements using object dimensions and a supplied floor height. |
| Planning | Simple pick/place endpoint sequence; no collision-aware path planner or grasp search. |
| Execution | DROID arm/gripper commands, joint-velocity settling, cooperative cancellation and failure logging. |
| Verification | Final supplied observations compared with targets; result retained in manager state. |
| Localization | Deferred; runtime accepts supplied poses and does not process images. |
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
├── DroidRobot (optional for offline planning)
└── EventLog (optional)
```

Objects own physical data and grasp descriptions. `Buffer` owns grid geometry,
fixtures and its independent localization methods. Desired placements belong
to `TargetConfiguration`, separately from measured poses. `ResetPlanner`
composes grasp/endpoints; `DroidRobot` delegates IK and robot control to DROID.
The manager owns sequencing, final verification and outcome state. No separate
reset-result object, driver protocol hierarchy or localizer adapter is needed.

## Frames

All transforms are T_parent_child and translations are metres. Observations
supplied to the manager are T_base_object and T_base_buffer. Requested poses
are T_buffer_object; grasp transforms are T_object_tool. The tool frame must
match the frame DROID reports. Cell targets put the geometric center half the
object height above the explicitly supplied buffer support plane.

Pure marker-to-cube geometry remains available on `Cube`; a marker center is
10 mm outward from the block center along its local top normal. Full edge-aligned
orientation additionally needs known marker mounting yaw. No marker image
detector or preview is part of the reset runtime.

## Reset lifecycle

```text
supplied scene → planning → executing → awaiting_verification
                                         ↓ supplied final measurements
                                      completed (success true/false)
```

Exceptions produce `failed`; a cooperative cancellation produces `cancelled`.
Old task-object poses are invalidated when execution starts. Completing motion
does not establish placement success. Final measurements are required for a
verified outcome; no localization output is synthesized from commanded poses.
State is held in memory for the process lifetime; JSONL logs retain events.
The JSON-lines process should remain running between reset and verification.

The planner assumes a clear transit corridor. It sends endpoints without dense
interpolation or intermediate positional-error checks; DROID joint-space moves
are not guaranteed to follow straight Cartesian segments. Neither collision
avoidance nor recovery from an emergency-stopped server is implemented.
