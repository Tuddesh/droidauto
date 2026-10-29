# Reset framework

This package prepares automatic scene restoration between policy rollouts.
Reset workflows remain explicit `NotImplementedError` placeholders.

## Core objects

Only these modules belong at the top level:

| Module | Object / responsibility |
| --- | --- |
| `manager.py` | `ResetManager`: owns observe/plan/execute/verify and internal outcome state. |
| `planner.py` | `ResetPlanner`: future reset-method integration and pick/place planning. |
| `target_configuration.py` | `TargetConfiguration`: desired object placements in buffer coordinates. |

`manager.state` exposes phase, current action, success, scene-before/after,
placement errors, failure reason and timing. Success starts unknown (`None`).
There is no separate reset-result object.

The manager references a `Buffer`, registered `SceneObject` instances, target
configuration, planner, supplied localization-tool adapter, `DroidRobot`,
`ZedWristCamera`, optional calibration and optional event log. Extra abstract
backend/localizer protocols are deferred until an integration requires them.

## Supporting packages

| Package | Contents |
| --- | --- |
| `hardware/` | Concrete DROID and ZED integrations. Controller startup belongs to `DroidRobot`. |
| `calibration/` | Existing `BufferCalibration` and hand-eye estimation. |
| `io/` | Existing `StationConfig`, `EventLog` and generic log-path naming. |

Shared pose conversion/validation is in `scene.transforms`, alongside the
[physical data models](../scene/README.md). Buffer localization remains owned
by [`Buffer`](../buffer/README.md).

## Prepared methods

These operations are declarations, not implemented reset behavior:

- `ResetManager.register_object()`, `remove_object()`, `observe()`, `plan()`,
  `reset()`, `cancel()`, `_execute()`, `_verify()`, `_record()`;
- `TargetConfiguration.set_pose()`, `set_cell()`, `remove()`, `pose_for()`;
- `ResetPlanner.plan()`, `plan_pick_place()`;
- future reset gripper operations `DroidRobot.open_gripper()`, `close_gripper()`.

Targets use the shared `object_id` and name their buffer with `buffer_id`.
Cell-target resolution will require actual object geometry during future
planner integration. Observed and desired poses are separate data.

The existing robot state/endpoint, image acquisition, exposure adaptation,
calibration and event-recording behavior is retained. No connection or reset
starts when the package is imported. The known DROID endpoint-IK workaround
and server recovery limitations remain hardware-adapter concerns.

## Imports

After installing the repository (`python -m pip install --no-deps -e .`):

```python
from auto_reset import ResetManager, ResetPlanner, TargetConfiguration
from auto_reset.hardware import DroidRobot, ZedWristCamera
from auto_reset.calibration import BufferCalibration
from auto_reset.io import StationConfig, EventLog
from scene import PhysicalObject, SceneObject, Cube
from buffer import Buffer, BufferItem
```

The root package exports only the three core reset objects. Current ownership,
implementation status and coordinate conventions are described in
[the architecture document](../../docs/ARCHITECTURE.md).
