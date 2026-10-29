# Current architecture

DroidAuto prepares automatic physical scene resetting on DROID. The source packages are `auto_reset`, `scene` and `buffer`, directly
under `src/`. `pyproject.toml` installs these packages; `droidauto` is the
Python distribution name.

## Implementation status

| Area | Current state |
| --- | --- |
| Shared physical-object data | Dataclasses and inheritance are present. Observation-update methods are placeholders. |
| Single object | `Cube`: 40 x 40 x 20 mm, centered 38 mm 5x5 top marker at object Z=+10 mm. |
| Marker conversion | Center conversion and full-pose mounting composition implemented; physical-edge yaw must be configured for full orientation. Marker identity can be bound and saved once supplied. |
| Buffer geometry/localization | Implemented layout, cell geometry, ArUco and stereo localization. |
| Reset orchestration | `ResetManager` interface and internal state; workflow methods raise `NotImplementedError`. |
| Desired scene | `TargetConfiguration` data; target editing/resolution methods raise `NotImplementedError`. |
| Reset planning | `ResetPlanner` interface; no reset or pick-and-place algorithm. |
| Robot/camera access | Concrete DROID and ZED adapters; explicit hardware operations. |
| Calibration | Buffer calibration data/utilities and hand-eye estimation. |
| Configuration/logging | Station JSON loader and flushed JSONL event logger. |
| Production entry points | None; `scripts/` is empty. |
| Diagnostics/tests | Manual marker-pose camera preview in `tests/`; no automated project test suite. |

Exploratory tools live under `experiments/`. They are independent of the core
reset manager.

## Object hierarchy

```text
PhysicalObject
├── Buffer
├── BufferItem
│   └── ArucoTile
└── SceneObject
    └── Cube
```

`PhysicalObject` owns `object_id`, `dimensions_m`, `pose`, `pose_frame` and
`observed_at`. Configured object IDs identify the same physical entity across
sessions. Generated default UUIDs identify individual constructed instances.
`BufferItem.item_id` is a read-only alias for the same string `object_id`.

`Buffer` owns grid geometry, layout occupants and buffer localization.
`BufferItem` adds a cell footprint; `ArucoTile` adds marker parameters.
`SceneObject` adds grasp definitions and manipulation parameters. `Cube` uses
those inherited fields. Objects do not carry reset policy or robot drivers.

The configured manipulation block has a 40 x 40 mm base and 20 mm height. Its
frame origin is the geometric center; +Z points toward the top marker face.
The 38 mm marker is centered on top. Its numeric ID and concrete 5x5
dictionary can be bound from a confirmed localization and saved to the station
config. Cube center is 10 mm inward along the marker normal. Full cube-edge
orientation conversion uses an explicit marker mounting yaw. Automatic image
localization, object loading and observation updates remain unimplemented.

## Reset ownership

`ResetManager` references:

- the `Buffer` and registered `SceneObject` instances;
- a `TargetConfiguration`;
- a `ResetPlanner`;
- a supplied localization-tool adapter whose concrete API is not implemented;
- a `DroidRobot`, `ZedWristCamera`, optional `BufferCalibration` and `EventLog`.

The manager owns reset execution and outcome state. `manager.state` exposes a
read-only top-level view of phase, current action, success, scene-before/after,
placement errors, failure reason and timing. Initial success is `None`.
There is no separate reset-result object.

The declared workflow is observe → plan → execute → observe → verify. These
manager operations are placeholders and do not command hardware. Planning
belongs to `ResetPlanner`; the concrete robot backend owns IK/motion dispatch.

## Coordinate conventions

A physical object's `pose` is `T_reference_object`, mapping object coordinates
into the named `pose_frame`. Translation is in metres. Target configurations
store `T_buffer_object` matrices and identify their reference buffer using
`buffer_id`. Desired and observed placements are separate data.

Buffer coordinates use +X across columns, +Y across rows and +Z out of the
marker plane. `Buffer.localize()` returns `T_camera_buffer`; it does not itself
establish robot-base coordinates. Calibration composes:

```text
T_base_buffer = T_base_tool @ T_tool_camera @ T_camera_buffer
```

Camera mounting must use the same tool frame reported by the robot backend.
The SDK's rectified stereo transform supplies the eye-to-eye offset.

## Source organization

- `src/auto_reset/{manager,planner,target_configuration}.py`: core reset objects.
- `src/auto_reset/hardware/`: concrete DROID and ZED adapters.
- `src/auto_reset/calibration/`: buffer calibration and hand-eye estimation.
- `src/auto_reset/io/`: station configuration and event logging.
- `src/scene/`: shared physical data, task objects and transform helpers.
- `src/buffer/`: Gridfinity layout, fixtures and buffer localization.

The reset request is the application boundary. Inference systems, policy
execution and evaluation orchestration are external to this repository.

For installation and configuration, see the [project README](../README.md).
For the backend boundary, see [DROID integration](DROID_SERVER.md).
