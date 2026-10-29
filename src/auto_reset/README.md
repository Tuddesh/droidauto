# Reset framework

The framework accepts supplied base-frame observations and executes a simple
cube pick-and-place through DROID. Image localization is not integrated.

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
optional event log. It has no camera/localizer dependency. `manager.state`
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
configured lift clearance. There is no collision model, trajectory optimizer,
client-side interpolation or positional-error gate. These are endpoints;
DROID's joint-space trajectory between them need not be Cartesian-straight.
The first configured grasp is selected; automatic grasp optimization is absent.

Each arm command is blocking, followed by a sustained-low-joint-velocity wait.
A server-reported unsuccessful command or exception aborts the remaining
sequence. Motion RPCs are not retried. Gripper endpoints use normalized 0/1.
Cancellation is cooperative between blocking commands, not an emergency stop.
An interrupted transfer performs no automatic gripper opening or recovery.

## Supplied observations and verification

`manager.observe({object_id: T_base_object, ...})` accepts measured poses.
Omitted task objects become unobserved; the fixed buffer pose is retained if
not included. All matrices are copied and validated before updating a batch.
Calling `observe()` without arguments returns a scene snapshot.

`manager.plan(target, current_tool_pose=...)` is offline; omit the tool pose to
read it from a connected robot. `manager.reset(target)` reads the actual tool
pose, plans before sending commands, logs each action and invalidates moved
objects' previous poses. It finishes at `awaiting_verification`, success unknown.

`manager.verify({object_id: measured_final_pose, ...})` computes position error
in metres and orientation error in radians against the requested placements.
These tolerances assess the final reset result; they do not gate intermediate
motion. Missing objects fail verification. Replacing the fixed buffer pose in
the final batch permits comparison relative to the latest supplied buffer pose.
Verification is optional while localization is deferred: another reset may use
new supplied pickup poses, leaving the preceding result unverified in the log.
Any moved, failed or cancelled transfer requires fresh poses before replanning.

## JSON request entry point

Run `python -m auto_reset --config CONFIG`. Each stdin line is one JSON
request; each stdout line is one JSON response. The process retains manager
state. `--request FILE` instead runs one JSON document and exits. The default
has no robot connection; `--execute` enables reset execution. Controller
relaunch requires the separate `--initialize-controllers` flag.

Poses may be rigid 4x4 matrices or `[x,y,z,roll,pitch,yaw]` with metres and
fixed-axis XYZ radians. Observation poses are base-relative; target poses are
buffer-relative. `timestamp_s` optionally records supplied observation time.

Example request shapes (poses here are illustrative):

```json
{"operation":"observe","observations":{"buffer-main":[0.4,-0.15,0.05,0,0,0],"cube-1":[0.45,0.15,0.06,0,0,0]}}
{"operation":"plan","current_tool_pose":[0.4,0,0.4,3.141592653589793,0,0],"targets":{"cube-1":{"cell":[2,2],"support_height_m":0.0,"yaw_rad":0.0}}}
{"operation":"reset","targets":{"cube-1":{"cell":[2,2],"support_height_m":0.0}}}
{"operation":"verify","observations":{"cube-1":[0.505,-0.045,0.06,0,0,0]}}
{"operation":"state"}
```

A target may instead be `{"pose": [x,y,z,roll,pitch,yaw]}`. A plan/reset
request may include an `observations` batch. A `plan` request never moves the
robot; without a supplied starting tool pose, it needs a robot state connection.
The default operation is `plan`. Errors are returned with manager state and log
path; one-shot errors exit nonzero. In streaming mode subsequent requests can
correct input or provide new measurements. Verification must use actual final
measurements, not copied requested targets.

Logs live under the configured `log_directory`, resolved relative to CONFIG.
The stream is sequential. Python callers may invoke `cancel()` from another
thread; stdin requests cannot interrupt a blocking reset command.
