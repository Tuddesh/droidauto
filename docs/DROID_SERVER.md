# DROID server and client architecture

This note describes the upstream `droid-dataset/droid` control server and the
client boundary relevant to DroidAuto. It was researched against the upstream
`main` branch and official DROID documentation on 2026-10-08. The software on
the actual robot may be a fork or a different revision; its code and runtime
configuration must be checked before integration.

## Executive summary

The upstream **DROID server** is a small ZeroRPC service intended to run on the
NUC connected to the Franka system. It wraps a `FrankaRobot` Python object and
binds it to TCP port `4242`. The server exposes lifecycle, motion-command, and
robot-state methods to a client on the evaluation laptop.

It is useful to distinguish two server layers on the NUC:

1. The DROID ZeroRPC server exposes `FrankaRobot` to another computer.
2. `FrankaRobot` launches and talks to local Polymetis robot and gripper
   services, which in turn communicate with the Franka and Robotiq hardware.

The DROID server is therefore a remote robot-control adapter. It does not run a
VLA model, acquire the DROID cameras, score trials, reset task objects, or manage
an evaluation campaign.

## Upstream architecture

```text
Evaluation laptop/workstation
┌──────────────────────────────────────────────────────────────┐
│ DroidAuto evaluation client                                 │
│  ├─ cameras and calibrated observations                     │
│  ├─ policy client / inference integration                   │
│  ├─ rollout and reset state machine                         │
│  ├─ outcome and initial-state measurement                   │
│  └─ trial logging                                           │
│                              │                               │
│                 ZeroRPC over TCP, port 4242                  │
└──────────────────────────────┼───────────────────────────────┘
                               │
NUC                            ▼
┌──────────────────────────────────────────────────────────────┐
│ scripts/server/run_server.py                                │
│  └─ ZeroRPC server wrapping FrankaRobot                     │
│          ├─ Polymetis RobotInterface on localhost           │
│          ├─ Polymetis GripperInterface on localhost         │
│          └─ IK and command conversion                       │
└──────────────────────────────┬───────────────────────────────┘
                               ▼
                     Franka arm + Robotiq gripper
```

Official DROID setup documentation assigns the real-time robot/controller work
to the NUC and camera data, GUI, and user-client code to the laptop. Cameras are
physically attached to and read by the laptop, not transported through the
DROID ZeroRPC server.

## What the server actually runs

The upstream server entry point is only a few lines:

```python
robot_client = FrankaRobot()
s = zerorpc.Server(robot_client)
s.bind("tcp://0.0.0.0:4242")
s.run()
```

Because it binds to `0.0.0.0`, it listens on every NUC network interface. The
upstream entry point shows no authentication, authorization, or encrypted
transport. Port `4242` must therefore be limited to the trusted, isolated robot
network by topology and firewall rules. It must never be exposed to an
untrusted LAN or the Internet.

The host-installation instructions start it on the NUC with:

```bash
conda activate polymetis-local
python scripts/server/run_server.py
```

The official Docker workflow instead starts the NUC compose stack under
`.docker/nuc/`. The installed DROID revision determines the exact command and
must be recorded with each experimental setup.

## Client connection

The upstream client wrapper is `droid.misc.server_interface.ServerInterface`.
It creates a `zerorpc.Client(heartbeat=20)` and connects to:

```text
tcp://<nuc-ip>:4242
```

By default, constructing `ServerInterface` also calls `launch_controller()` and
`launch_robot()`. That means connection and hardware initialization are coupled
and the constructor has significant physical side effects. DroidAuto should
place this behind an explicit, logged lifecycle step rather than allowing robot
startup to happen implicitly during object construction.

The upstream wrapper converts NumPy command arrays to Python lists before RPC.
Robot replies are composed mainly of lists, dictionaries, numeric values, and
timestamps, which ZeroRPC can serialize.

## Relevant remote API

The public methods used by the upstream client fall into these groups:

| Purpose | Methods | Notes |
|---|---|---|
| Lifecycle | `launch_controller`, `launch_robot`, `kill_controller` | Starts or stops local Polymetis processes and interfaces. |
| Unified command | `update_command` | Converts the requested action space into joint and gripper targets, then applies them. |
| Direct commands | `update_pose`, `update_joints`, `update_gripper` | Lower-level control paths; misuse can move hardware immediately. |
| State | `get_ee_pose`, `get_joint_positions`, `get_joint_velocities`, `get_robot_state` | Used for feedback, logging, and command validation. |
| Conversion | `create_action_dict` | Produces derived Cartesian, joint, and gripper command/state fields. |

`RobotEnv` supports `cartesian_position`, `joint_position`,
`cartesian_velocity`, and `joint_velocity` action spaces. Cartesian actions are
seven-dimensional: six arm values followed by one gripper value. Joint actions
are eight-dimensional: seven joints followed by the gripper. The upstream
environment declares a `15 Hz` control rate and checks velocity actions against
`[-1, 1]` before forwarding them.

The default high-level path is `cartesian_velocity`. `FrankaRobot` converts that
command into Cartesian and joint targets using its IK helper, then sends desired
joint positions through Polymetis and a normalized gripper target through the
gripper interface.

## Observations and cameras

The upstream `RobotEnv.get_observation()` combines two independently obtained
sources:

- robot state returned through `ServerInterface`; and
- camera frames, timestamps, intrinsics, and extrinsics read by a local
  `MultiCameraWrapper` on the laptop.

Robot state includes Cartesian pose, normalized gripper position, joint
positions and velocities, several torque fields, controller latency, and a
`prev_command_successful` flag. It also includes a robot timestamp split into
seconds and nanoseconds. The environment adds client-side read-start and
read-end timestamps. DroidAuto should retain all three times so communication
latency and observation skew can be analyzed rather than hidden.

A remote VLA policy server, if used, is a separate connection owned by the
DroidAuto client. It is not part of the upstream DROID control server protocol.

## Proposed DroidAuto responsibility split

| DROID server / NUC | DroidAuto client / workstation |
|---|---|
| Launch and connect Polymetis interfaces | Explicit server lifecycle and readiness checks |
| Execute arm and gripper commands | Acquire and synchronize camera observations |
| Return robot state and robot timestamps | Query local or remote VLA policy inference |
| Perform upstream action/IK conversion | Enforce rollout state, deadlines, and stop conditions |
| Maintain low-level robot communication | Run scene reset and verify the resulting configuration |
| Report command/controller status | Determine outcomes and write complete trial records |

This split is an integration proposal derived from the upstream architecture,
not an upstream DROID requirement.

## Integration hazards to resolve

The following issues are visible in upstream `main` and should be tested against
the exact installation:

1. **No secure network boundary in the entry point.** The server binds all
   interfaces on port `4242`, and the reviewed code does not configure
   authentication or encryption.
2. **Constructor side effects.** `ServerInterface(..., launch=True)` launches
   the controller and robot during initialization.
3. **No protocol/version handshake.** The reviewed RPC layer does not negotiate
   an API version or report capabilities. Pin and log the DROID commit or image.
4. **Client/server naming mismatch.** The wrapper calls
   `get_gripper_state()`, while the reviewed `FrankaRobot` defines
   `get_gripper_position()`. Avoid that wrapper method until verified or fixed.
5. **Configuration edge case.** Upstream parameters initialize `nuc_ip` to an
   empty string, while `RobotEnv` chooses local operation only when it is
   exactly `None`. Use a validated, explicit NUC address.
6. **Suppressed controller errors.** Some non-blocking joint-command paths catch
   `grpc.RpcError` without propagating it. Check `prev_command_successful`, robot
   state progression, and command deadlines instead of treating an RPC return
   as proof of execution.
7. **Heartbeat is not a motion watchdog.** The 20-second ZeroRPC heartbeat only
   helps detect connection loss. It is not a suitable real-time deadman or
   emergency-stop mechanism.
8. **Public RPC surface.** ZeroRPC wraps the `FrankaRobot` object directly. The
   effective remotely callable surface should be enumerated on the installed
   version and restricted by the client adapter and network.

## Recommended client adapter

DroidAuto should wrap the upstream interface behind a small project-owned
adapter with these properties:

- validated NUC address and fixed port configuration;
- explicit `connect`, `initialize`, `ready`, `stop`, and `close` lifecycle;
- no robot motion from a constructor;
- declared action convention, units, shapes, bounds, and control frequency;
- monotonic command sequence numbers and client timestamps in the trial log;
- observation age and RPC-latency checks before every action;
- a maximum rollout duration and maximum consecutive communication failures;
- post-command verification using returned robot state and
  `prev_command_successful`;
- structured exceptions that distinguish transport, server, controller,
  safety, perception, policy, reset, and evaluation failures;
- an external physical emergency stop and robot-side safety configuration that
  do not depend on the Python client or network.

The client should initially support read-only connection/state tests, then
low-risk bounded motion tests under supervision, before running a policy loop.

## Information still needed from the actual station

Before implementation, record:

- the DROID repository/fork URL and exact commit or container image;
- Franka model, gripper, firmware, and Polymetis versions;
- NUC and workstation operating systems, addresses, and firewall rules;
- whether host or Docker deployment is used;
- the actual callable RPC methods and their behavior;
- camera models, serials, calibration files, and workstation attachment;
- the selected scene-reset method and its own control interface;
- policy location and policy-client protocol;
- tested stop, recovery, workspace, collision, and emergency procedures.

## Inspection of the CIIRC control PC (2026-10-15)

I connected to `ros@192.168.88.13` (`cotta-pc`) and inspected the installed
code and configuration without issuing robot-control RPCs. This is the **control
PC**, not the NUC. Its checkout is
`/home/ros/drozdpat/repos/control_droid` on branch `numpy2-stack`, commit
`b1d96c2aadf7098950641d1566fcc58246c6cb9b`. The `packages/droid`
submodule is at `740f28b8108a8bbd4e3fd74a296bc89297a4042e`; it has local
edits to camera code and `droid/misc/parameters.py`. Other installed components
include `roboarena_evaluator` at `7b4a80e4ad04083a031e1608da6e48024ae0dbc9`
and `TeleopController4Droid` at `09092a7301cb7aebeac9e74fae1b07f58a551f17`.
The umbrella checkout also has local, untracked experiment scripts and data, so
these commit IDs alone do not fully describe its runtime behavior.

The installed environment is `control_droid/.conda` (Python 3.10.21, NumPy
2.2.6, Gymnasium 1.3.0, ZeroRPC 0.6.3). Both `droid` and `pyzed.sl` import in
that environment. Video devices exist on the control PC, but camera acquisition
and calibration were not exercised. The effective DROID parameters name NUC
`192.168.88.12`, robot `192.168.88.11`, and control PC `192.168.88.13`.
Configured camera serials are wrist `14301071`, left `37350985`, and right
`36264929`. Several policy scripts still default to older camera serials
`12990599`, `35188913`, and `30196176`; override those when using the current
cameras. TCP port 4242 on the NUC was reachable from the control PC. No robot
service status or RPC method behavior was verified on the NUC.

The installed `ServerInterface` wraps ZeroRPC at `tcp://<nuc_ip>:4242`.
Constructing it with the default `launch=True` calls `launch_controller()` and
`launch_robot()` remotely. Use `launch=False` for a connection that does not
initialize hardware:

```python
from droid.misc.server_interface import ServerInterface

robot = ServerInterface(ip_address="192.168.88.12", launch=False)
try:
    # This RPC requires a controller that another operator has already started.
    state, timestamp = robot.get_robot_state()
    print(state["joint_positions"], state["prev_command_successful"])
    print(timestamp)
finally:
    robot.server.close()
```

Run this in the installed `.conda` environment on the control PC. The call to
`get_robot_state()` reads state but can fail if the NUC has not initialized its
robot interface. `ServerInterface.get_gripper_state()` still calls a server
method absent from the inspected `FrankaRobot`; use the `gripper_position` field
in `get_robot_state()` instead.

For a full camera observation, the installed `RobotEnv` joins NUC robot state
with local `MultiCameraWrapper` frames, calibration, and timestamps. Its
constructor creates `ServerInterface` with the default `launch=True` and opens
the cameras; `do_reset=False` only skips the later arm reset. `env.reset()` opens
the gripper and moves the arm to the configured joint pose. `env.step(action)`
sends a command; it returns an action dictionary rather than the usual
Gymnasium five-tuple. The selected action space requires 7 values for
Cartesian control or 8 for joint control, each with the gripper as the last
value. Velocity actions are normalized and checked against `[-1, 1]` by
`RobotEnv.step`; the installed IK code maps a full-scale Cartesian component to
up to 0.075 m linear or 0.15 rad rotation per control step and gripper velocity
to up to 0.25 normalized width per step. The intended loop rate is 15 Hz.
Gripper position is normalized: 0 open, 1 closed. Direct
`ServerInterface.update_command()` bypasses `RobotEnv.step`'s input checks.

The existing `scripts/robot_control/pi0.py` and `run_policy.py` show the policy
integration path: acquire `env.get_observation()`, send selected camera and
robot-state fields to an `openpi_client` WebSocket policy, then pass the returned
action to `env.step()` and record rollout data. The camera wrapper runs on the
control PC; camera frames do not pass through the NUC ZeroRPC server. Their
defaults and reset behavior need review before adapting them for DroidAuto.

## Control-stack review (2026-10-22)

The installed checkout under `/home/ros/drozdpat/repos/control_droid` is still
`numpy2-stack` at `b1d96c2`, with DROID at `740f28b`. Its own environment imports
`droid` and `ServerInterface` from that checkout's `packages/droid` tree. Local
changes in that submodule concern camera reading and machine parameters, not
the robot command implementation. This inspection did not restart controllers
or send arm or gripper commands.

The most useful local sources, relative to that checkout, are:

| Source | What it establishes |
|---|---|
| `README.md`, `MIGRATION_HANDOFF.md` | Current stack installation and the unchanged ZeroRPC boundary to the older NUC server. Some handoff paths refer to a former `control_droid-new_stack` worktree. |
| `packages/droid/droid/misc/server_interface.py` | Robot-only client, startup sequence, state reads, and command forwarding. |
| `packages/droid/droid/robot_env.py` | Camera observations, default robot reset, action dimensions, and action-range checks. |
| `scripts/robot_control/test_server_no_cameras.py` | Existing robot-only example using `ServerInterface`; running it initializes controllers and subsequently moves the robot. |
| `scripts/robot_control/test_joint_sine.py` | Joint-position streaming, timing, and feedback measurements. Running it also resets and moves the robot. |
| `packages/droid/droid/franka/robot.py` | NUC-side target handling, controller selection, and action conversion. |
| `packages/droid/droid/robot_ik/robot_ik_solver.py` | Normalized action scaling and Cartesian IK constraints. |

The older `scripts/robot_control/README.md` recommends a nested TeleopController
environment. The umbrella README and migration handoff describe the newer
checkout-local environment. DroidAuto should retain its own environment when
integrating the client package; it should not depend on activating either of
Patrik's environments.

### Initialization and commands

`ServerInterface(ip_address=..., launch=True)` connects and calls
`launch_controller()` then `launch_robot()`. Its retry helper repeats both on a
remote error, up to five attempts. `launch_controller()` restarts the arm and
gripper controller processes; `launch_robot()` constructs their Polymetis
interfaces and the IK solver. `launch=False` skips those operations.

`RobotEnv` constructs that client, opens cameras, and by default calls
`reset()`, which opens the gripper and sends a home joint target.
`RobotEnv(do_reset=False)` still initializes controllers and cameras. For an
arm-only tool, the narrower integration point is `ServerInterface`.

| Operation | Installed client call and input |
|---|---|
| Read feedback | `get_robot_state()` returns state and robot timestamp. |
| One joint target | `update_joints(q, velocity=False, blocking=True)` takes seven joint angles in radians. |
| One gripper target | `update_gripper(g, velocity=False, blocking=True)` uses normalized position: 0 open, 1 closed. |
| Joint action stream | `update_command(action, action_space="joint_position", gripper_action_space="position", blocking=False)` takes seven angles plus one gripper position. |
| Cartesian action stream | `update_command(action, action_space="cartesian_velocity", gripper_action_space="position", blocking=False)` takes six normalized Cartesian action components plus one gripper position. |

The examples schedule streaming at 15 Hz in the client; `update_command` does
not provide that scheduling. DROID's normalized velocity actions represent
scaled increments, not raw SI velocity values. Blocking `update_pose` solves
endpoint IK and then sends a joint move; it does not promise a straight
end-effector path.

### Robot limits and missing metadata API

Read-only enumeration of the live NUC's public RPC methods found no robot
metadata, model, or joint-limit query. The control PC's selected model is
`panda`, and its model XML contains joint limits; this does not establish which
model/configuration the live NUC is using.

The inspected Cartesian IK path enables joint-position constraints using its
model. The direct joint-position path does not explicitly clamp target joints
against those limits before forwarding them to Polymetis. Using the official
client therefore does not itself establish server-side target validation for
every action space.

DroidAuto's jogger currently duplicates Panda limits. To replace that with
server-provided configuration, the NUC needs a metadata RPC backed by its
actual robot model, with joint names/order, units, position limits, and the
motion limits available from that model. Server-side target validation should
use the same source. A client should not silently substitute a copied Panda
table when metadata is missing.

The live state read during this review failed with
`AttributeError: 'FrankaRobot' object has no attribute '_robot'`. The RPC
service is reachable but lacks its arm interface. That establishes an
initialization problem at the time of inspection, not the cause of any
separate controller-launch failure.

## Primary sources

- [Official DROID platform repository](https://github.com/droid-dataset/droid)
- [DROID software architecture and Docker setup](https://droid-dataset.github.io/droid/software-setup/docker.html)
- [DROID host installation](https://droid-dataset.github.io/droid/software-setup/host-installation.html)
- [ZeroRPC server entry point](https://github.com/droid-dataset/droid/blob/main/scripts/server/run_server.py)
- [Upstream `ServerInterface`](https://github.com/droid-dataset/droid/blob/main/droid/misc/server_interface.py)
- [Upstream `RobotEnv`](https://github.com/droid-dataset/droid/blob/main/droid/robot_env.py)
- [Upstream `FrankaRobot`](https://github.com/droid-dataset/droid/blob/main/droid/franka/robot.py)
