# DROID integration

## System boundary

```text
DroidAuto client on the control PC
    ├── ZED camera acquisition
    ├── scene/reset framework
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
`initialize()` calls `launch_controller()` followed by `launch_robot()`, with
up to five full-pair retries on remote errors. Startup is an explicit action.
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
The backend validates the list's shape/numeric values and does not maintain a
copied table of robot joint limits.

For a blocking Cartesian endpoint, `command_pose()` uses the server's
`add_noise_to_joints(seed, delta)` endpoint-IK helper, then calls blocking
`update_joints`. Despite its name, this helper applies the supplied exact pose
offset and does not add random noise or command motion. IK is performed on
the server. This path works around the installed blocking `update_pose` branch
passing an IK `(joint_angles, success)` tuple into a tensor constructor.

The underlying move is generated in joint space; the backend does not promise
a straight Cartesian path. Nonblocking `command_pose()` forwards to DROID's
incremental Cartesian API. No client interpolation or scene-reset policy is
implemented by this backend. `wait_until_stopped()` waits for sustained low
joint velocities and checks the server's previous-command status, without
comparing the tool pose to the requested endpoint. Gripper methods call
`update_gripper(position, False, True)` once, with 0 open and 1 closed.

## Current integration limitations

- The IK helper can return its seed joints when IK fails; the adapter does not
  receive a separate IK-success flag from this helper.
- Some server motion paths suppress controller exceptions. A completed RPC
  alone is not an independently measured placement result.
- A blocking command may remain pending after an interrupted move. The client
  cannot recover an unresponsive RPC server through another RPC call, and it
  does not restart the NUC service automatically.
- The adapter has no model/limit metadata query, collision planner or robot-side
  cancellation API. It does not automatically retry motion commands.
- A reachable TCP port does not establish that robot feedback is live.
- The camera-to-tool mounting frame must match the backend's reported tool
  frame; a transform measured against a different Franka end-effector frame
  requires frame conversion.

## Camera backend

`src/auto_reset/hardware/zed.py` provides `ZedWristCamera` and `CameraFrame`.
It captures rectified left/right images at HD2K (2208 x 1242 per eye) and
15 FPS with matching intrinsics. The reset runner does not open this camera;
localization is deferred and observations are supplied directly. Stereo eyes
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
