# DROID integration

## System boundary

```text
DroidAuto client on the control PC
    ├── ZED camera acquisition
    ├── scene/reset framework
    ├── cuRobo IK on the local CUDA GPU
    └── ZeroRPC → DROID server on NUC → Polymetis → Franka and gripper
```

DroidAuto communicates directly with the existing ZeroRPC service. It does
not require ROS or Patrik's Python environment. Camera images are acquired
locally on the control PC, independently of robot-state RPCs.

The lab configuration uses these addresses:

| Component | Address |
| --- | --- |
| Control PC | `192.168.88.13` |
| NUC DROID service | `tcp://192.168.88.12:4242` |
| Franka controller | `192.168.88.11` |

The station example selects wrist camera serial `14301071`. Network/camera
values are station configuration, not properties of the reset algorithm.
This document does not describe the live availability of the robot or server.

## Robot backend

`src/auto_reset/hardware/droid.py` contains `DroidRobot` and `RobotState`.
The backend provides:

- context-manager connection and cleanup;
- explicit `initialize()` controller startup;
- `read_state()`, `require_stationary()` and `read_calibration_pose()`;
- `joint_target_for_pose()`, `command_pose()` and `command_joints()`;
- blocking `open_gripper()` / `close_gripper()`, `read_gripper_position()` and
  `wait_until_stopped()`.

Opening the RPC connection does not initialize controllers or move the robot.
`initialize()` uses existing RPCs to start controllers and attach interfaces.
Transient attachment errors are retried within a 30-second readiness window.
Persistent missing gripper metadata permits one recovery relaunch after five
seconds; recovery is handled inside the same initialization call. Unrelated
errors propagate. No server files or installed drivers are modified.
The RPC client uses a 60-second timeout and a 20-second heartbeat.

`RobotState` contains the measured tool transform, seven joint positions,
seven joint velocities, robot timestamp and previous-command status.
`read_state()` requires finite feedback, a recent timestamp and advancing
samples. Its default maximum state age is 0.5 seconds, so client and robot
clocks must agree. `read_calibration_pose()` reads one pose without these
manual-capture age/stationarity gates; the operator establishes stationarity.

## Command conventions

| Value | Convention |
| --- | --- |
| Joint positions | Seven angles in radians. |
| Cartesian poses | `[x, y, z, roll, pitch, yaw]`, metres and fixed-axis XYZ radians. |
| Transform matrices | `T_parent_child`, mapping child-frame coordinates into the parent frame. |
| DROID gripper position | Normalized position, 0 open and 1 closed; both reset backend methods are implemented. |

`command_joints()` sends one `update_joints(joints, False, blocking)` request.
The backend validates the list's shape/numeric values. Local IK additionally
uses the station's explicit `ik` joint bounds, intersected with its robot model.

Cartesian targets are clipped so the closed fingertips stay at base Z >= -0.025 m,
using the measured 178 mm flange offset. Clipping emits a warning and reset
logs record the applied endpoint. XY and orientation are preserved. This limits
endpoints; joint-space paths and arbitrary direct joint commands are not constrained.
The cube grasp center is separately configured 169 mm along flange +Z; it is
9 mm closer to the flange than the closed fingertips.

`joint_target_for_pose()` uses local cuRobo V2 on the station's NVIDIA GPU.
The target is `panda_link8`, the same flange frame DROID reports. The stock
Panda Hand geometry is excluded because the actual gripper is a Robotiq.
Both blocking and nonblocking `command_pose()` solve locally and send only
`update_joints`; no DROID Cartesian or Polymetis IK endpoint is called.

The station's `ik` configuration names the CUDA device, solve tolerances,
number of seeds and joint bounds. The example bounds reproduce the audited
DROID Panda hardware-client ranges; the 0.2 rad inward margin matches its
safety-controller activation zone. Limits are applied before optimization,
and only successful, finite solutions inside those bounds can be selected.
Among valid solutions, the one nearest the preceding joint configuration is
selected. There is no CPU or remote-IK fallback.
CUDA Graph capture is disabled by default (`ik.use_cuda_graph=false`); the
solver still runs on CUDA. This avoids graph-capture invalidation at startup.

Reset planning solves all endpoint joint targets before sending any arm or
gripper command. Each move retains both its Cartesian pose and seven joint
angles. `ik_solved` events record joint targets, pose errors and minimum
distance from the effective joint bounds. A failed solve aborts planning.

The underlying move is generated in joint space; the backend does not promise
a straight Cartesian path. No client interpolation or scene-reset policy is
implemented by this backend. `wait_until_stopped()` waits for sustained low
joint velocities and checks the server's previous-command status, without
comparing the tool pose to the requested endpoint. Gripper methods call
`update_gripper(position, False, True)` once, with 0 open and 1 closed.

## Current integration limitations

- IK checks endpoint feasibility and arm self-collision. There is no table,
  buffer or Robotiq collision model, and no continuous-path collision check.
- Some server motion paths suppress controller exceptions. A completed RPC
  alone is not an independently measured placement result.
- A blocking command may remain pending after an interrupted move. The client
  cannot recover an unresponsive RPC server through another RPC call, and it
  does not restart the NUC service automatically.
- The adapter has no hardware-client limit metadata query or robot-side
  cancellation API. Configured bounds must match the NUC deployment; motion
  commands are not automatically retried.
- A reachable TCP port does not establish that robot feedback is live.
- The camera-to-tool mounting frame must match the backend's reported tool
  frame; a transform measured against a different Franka end-effector frame
  requires frame conversion.

## Camera backend

`src/auto_reset/hardware/zed.py` provides `ZedWristCamera` and `CameraFrame`.
It captures rectified left/right images at HD720 (1280 x 720 per eye) and
15 FPS with matching intrinsics. The reset manager opens it for initial buffer
detection or an explicit rescan. Object poses are supplied directly. Stereo eyes
share one SDK grab and timestamp. The SDK rectified stereo transform supplies
the right optical frame relative to the left in IMAGE coordinates/metres.

Exposure/gain are automatic. Startup and metering-region changes consume
frames to let exposure settle. `camera.auto_exposure_roi` selects an optional
normalized metering rectangle; buffer capture can infer a region from detected
markers. A final image or synchronized pair supplies the pose observation.
Frame metadata includes exposure, gain, metering region and settling details.

## Upstream references

- [DROID platform repository](https://github.com/droid-dataset/droid)
- [DROID server entry point](https://github.com/droid-dataset/droid/blob/main/scripts/server/run_server.py)
- [DROID robot implementation](https://github.com/droid-dataset/droid/blob/main/droid/franka/robot.py)
- [DROID client wrapper](https://github.com/droid-dataset/droid/blob/main/droid/misc/server_interface.py)
- [DROID setup](https://droid-dataset.github.io/droid/software-setup/docker.html)
