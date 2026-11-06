# Reset framework

The framework accepts camera or supplied base-frame observations and executes
a simple cube pick-and-place through DROID. Buffer localization and cube
observation use the station's shared multi-dictionary ArUco detector.

## Objects and ownership

| Object | Methods / responsibility |
| --- | --- |
| `ResetManager` | `register_object`, `remove_object`, `observe`, `plan`, `reset`, `cancel`, `verify`; owns execution and internal outcome state. |
| `ResetPlanner` | `plan`, `plan_pick_place`; composes grasps and endpoints. |
| `TargetConfiguration` | `set_pose`, `set_cell`, `remove`, `pose_for`; owns requested buffer-relative object placements. |
| `DroidRobot` | Explicit connection/initialization, state reads, arm endpoints, gripper commands, `wait_until_stopped`. |
| `StationConfig` | Constructs buffer geometry, cube objects, planner and targets; reads poses and persists marker binding. |
| `EventLog` | Flushed JSONL event records, including failures. |

The manager references a buffer, robot, planner, registered scene objects and
optional event log, plus camera/station configuration for buffer acquisition.
A saved BufferCalibration supplies the fixed pose without another capture. `manager.state`
returns a detached snapshot of phase, current action, success, scene-before/
after, placement errors, failure reason and timing. There is no result object.

## Geometry and execution

Inputs use T_base_object and T_base_buffer. Targets use T_buffer_object.
Grasps use T_object_tool, where tool means DROID's reported tool frame.
The planner computes:

```text
pickup tool pose = T_base_object @ T_object_tool
placement tool pose = T_base_buffer @ T_buffer_object @ T_object_tool
```

`TargetConfiguration.set_cell(..., scene_object=cube, support_height_m=z)`
uses the actual cube height to place its center above the supplied cell floor.
The floor height is relative to the marker plane; it is not assumed to be zero.
Fixture-occupied cells and oversized rotated footprints are rejected.

A transfer is: initial lift at current XY → open → above pickup → pickup →
close → lift → above destination → place → open → retreat. Transit height is
absolute base Z, raised as needed above pickup/place tool heights by the
configured lift clearance. There is no scene collision model, trajectory optimizer,
client-side interpolation or positional-error gate. These are endpoints;
DROID's joint-space trajectory between them need not be Cartesian-straight.
The first configured grasp is selected; automatic grasp optimization is absent.

Before execution, cuRobo solves all endpoint poses on the configured CUDA GPU,
with the station's joint bounds and Panda arm self-collision model. Solutions
are selected near the preceding joint configuration and stored as seven joint
angles per move. A failed solve prevents the entire command sequence from
starting. Execution sends only joint targets to DROID, without remote IK.
The Robotiq and scene are not included in collision geometry.

Each arm command is blocking, followed by a sustained-low-joint-velocity wait.
A server-reported unsuccessful command or exception aborts the remaining
sequence. Motion RPCs are not retried. Gripper endpoints use normalized 0/1.
Cancellation is cooperative between blocking commands, not an emergency stop.
An interrupted transfer performs no automatic gripper opening or recovery.

## Supplied observations and verification

`manager.observe({object_id: T_base_object, ...})` accepts task-object poses
and detects the buffer if it has no known pose. Supplied buffer poses are
rejected. Omitted task objects become unobserved; the fixed buffer pose is
retained. `observe(detect_buffer=True)` explicitly rescans. Capture uses the
existing Buffer detector and saves the calibration/source image. The station
owns one open camera session; buffer and cube observations reuse it until
shutdown. Python callers likewise pass an already-open camera to the manager.
Calling `observe()` without arguments returns a snapshot.

`observe(detect_objects=True)` captures the configured cube markers and converts
their poses through the measured camera mount into robot-base cube-center poses.
Both eyes contribute to one joint reprojection fit when visible. One eye also
works. Marker ID/dictionary and mounting yaw must match the cube configuration.
Missing or ambiguous markers clear the object's previous pose. Frozen inputs,
overlays and evaluation JSON are saved under `runs/reset/object_scans/`; interactive
review supports accept, retake, print and cancel. `state['last_observation']`
contains detections, fitted poses, errors and image paths. `detect_buffer=True`
can be combined with object detection to refresh the buffer too.

`manager.plan(target, current_tool_pose=...)` is offline; omit the tool pose to
read it from a connected robot. `solve_ik=True` additionally needs measured
starting joints and the configured local CUDA solver. `manager.reset(target)` reads the actual tool
pose, plans before sending commands, logs each action and invalidates moved
objects' previous poses. It finishes at `awaiting_verification`, success unknown.

`manager.verify({object_id: measured_final_pose, ...})` computes position error
in metres and orientation error in radians against the requested placements.
These tolerances assess the final reset result; they do not gate intermediate
motion. Missing objects fail verification. The buffer is treated as fixed until
an explicit rescan.
Verification is optional: another reset may use fresh pickup observations,
leaving the preceding result unverified in the log.
Any moved, failed or cancelled transfer requires fresh poses before replanning.

## JSON request entry point

Run `python -m auto_reset --config CONFIG`. Each stdin line is one JSON
request; each stdout line is one JSON response. The process retains manager
state. `--request FILE` instead runs one JSON document and exits. A robot
connection supplies the tool pose for capture; `--execute` enables reset motion. Controller
relaunch requires the separate `--initialize-controllers` flag.

Poses may be rigid 4x4 matrices or `[x,y,z,roll,pitch,yaw]` with metres and
fixed-axis XYZ radians. Object observation poses are base-relative; target poses are
buffer-relative. The buffer is detected once or loaded from calibration.
`--scan-buffer` forces a stationary recapture; `--dummy` supplies only object poses. `timestamp_s` optionally records supplied observation time.
`--detect-objects` instead uses camera cube poses and the example buffer target;
it plans unless `--execute` is also present. `--review-buffer` enables frozen
review for both buffer and cube scans; `--no-review` disables interactive review.

Example request shapes (poses here are illustrative):

```json
{"operation":"observe","observations":{"cube-1":[0.45,0.15,0.06,0,0,0]}}
{"operation":"plan","current_tool_pose":[0.4,0,0.4,3.141592653589793,0,0],"targets":{"cube-1":{"cell":[2,2],"support_height_m":0.0,"yaw_rad":0.0}}}
{"operation":"reset","targets":{"cube-1":{"cell":[2,2],"support_height_m":0.0}}}
{"operation":"verify","observations":{"cube-1":[0.505,-0.045,0.06,0,0,0]}}
{"operation":"state"}
```

A target may instead be `{"pose": [x,y,z,roll,pitch,yaw]}`. A plan/reset
request may include an `observations` batch. A `plan` request never moves the
robot; without a supplied starting tool pose, it needs a robot state connection.
An `observe` request without a batch captures cube poses and uses the cached
buffer; include `"detect_buffer": true` to rescan it. Its response includes the
observed poses. Plan/reset requests can include `"detect_objects": true` instead
of an `observations` batch.
Plan requests can set `"solve_ik": true` to include local IK joint targets.
The `--detect-objects` one-shot mode includes this joint-planning step.
The default operation is `plan`. Errors are returned with manager state and log
path; one-shot errors exit nonzero. In streaming mode subsequent requests can
correct input or provide new measurements. Verification must use actual final
measurements, not copied requested targets.

Logs live under the configured `log_directory`, resolved relative to CONFIG.
The stream is sequential. Python callers may invoke `cancel()` from another
thread; stdin requests cannot interrupt a blocking reset command.
