# DroidAuto — automated real-world VLA evaluation on DROID

DroidAuto is the working repository for a semestral project that is intended to
continue into a bachelor's thesis. Its goal is to design, build, and validate an
automatic evaluation station for Vision-Language-Action (VLA) policies on the
[DROID robot platform](https://droid-dataset.github.io/droid/).

Mask R-CNN and YCB-Video experiments in this repository are supporting
perception work. They are not the main purpose of the project.

## Motivation

VLA models are increasingly evaluated on physical robots because simulation is
not always a reliable proxy for real-world performance. Real-world evaluation,
however, is slow and labour-intensive: after every rollout, a person normally
has to restore the scene, record the result, and prepare the next trial. This
limits the number of trials, introduces variability, and makes comparisons
between policies difficult to reproduce.

This project will build an evaluation station that executes repeated policy
rollouts with minimal human involvement. Between rollouts, it will use an
existing scene-reset method to restore a defined initial configuration, verify
the resulting scene, and record the trial and its outcome.

## Objectives

The project will:

1. Implement a DROID client that orchestrates policy rollouts on the real robot.
2. Integrate an existing automatic scene-reset method between rollouts.
3. Measure reset reliability and similarity to the intended initial state.
4. Log observations, actions, timing, reset measurements, outcomes, failures,
   and relevant configuration for every trial.
5. Quantify how reset errors affect measured policy performance.
6. Evaluate selected VLA models on a defined set of manipulation tasks.
7. Compare automatic evaluation with manually conducted evaluation.

The intended result is a system that enables larger and more reproducible
real-world VLA evaluations. The same infrastructure may later support work on
real-to-sim validation and real-world reinforcement-learning fine-tuning.

## Planned evaluation loop

```text
validate system state
        ↓
reset physical scene
        ↓
measure and verify initial configuration
        ↓
run one VLA policy rollout
        ↓
determine and record the outcome
        ↓
store a complete trial record
        ↓
repeat or stop safely
```

Trials must retain failures and reset deviations rather than silently dropping
them. Reproducibility requires distinguishing policy failures, reset failures,
perception failures, operator interventions, and infrastructure failures.

## System boundary

The evaluation station will act as a **client** of the DROID control server:

```text
VLA policy / policy server
            ↕
DroidAuto evaluation client ── ZeroRPC ── DROID server on the NUC
       ↕ cameras and logs                         ↕
 scene verification/reset                 Polymetis → Franka + gripper
```

The DROID server is the robot-control boundary; it is not itself a VLA policy
server. Camera acquisition, policy interaction, evaluation orchestration,
outcome measurement, and experiment logging belong on the client side. See
[DROID server and client architecture](docs/DROID_SERVER.md) for the researched
interface and integration notes.

## Current work areas

- DROID client and robot-control integration.
- Automatic rollout, reset, verification, and logging state machine.
- Scene-state and outcome perception.
- [Gridfinity buffer layout and ArUco localization](buffer/README.md).
- Gridfinity fixtures and generated ArUco markers under `openscad/`.
- Mask R-CNN, synthetic-data, and YCB-V perception experiments.

## Buffer calibration and marker-hover task

The reusable [`auto_reset` components](auto_reset/README.md) capture one
stationary wrist-camera image on the lab PC, call `Buffer.localize()`, save the
buffer pose in robot coordinates and reuse it for repeated marker visits.
The task hovers the TCP 20 cm above each marker for five seconds. Configuration,
offline target preview and lab-PC launch commands are documented in that
module's README. The geometric waypoint planner currently assumes a cleared
workspace; obstacle-aware planning remains future integration work.

## Single joint-offset move

From this PC or the control PC, using DroidAuto's own environment:

```bash
conda activate "$PWD/.conda"
python scripts/joint_offset_move.py
```

This script uses the existing DROID ZeroRPC interface at `192.168.88.12:4242`.
It initializes the arm and gripper controllers, reads the measured joint
positions, and previews a target of **+pi/8 radians (+22.5 degrees) on all seven
joints**. Press Enter to send the move or Ctrl+C to cancel. It reads fresh
feedback again after confirmation, then sends that relative offset using a
blocking `update_joints` call and reports the measured result. It does not
copy a joint-limit table into the client or clamp the requested target. This
does not establish that a particular target or path is collision-free or
within the robot's limits; the current DROID API has no metadata query.

The NUC must be reachable and the robot ready for control. No camera software
or installation of Patrik's full stack is needed on the computer running this
script. Use `--nuc-ip` to select a different server.

Both the offset script and the jogger use a 60-second RPC timeout and Patrik's
startup sequence: retry `launch_controller()` followed by `launch_robot()` up
to five times on a remote error, with 0.1 seconds between attempts. Feedback
must be no more than 0.5 seconds old. Motion commands are sent once; only
controller initialization is retried.

## Manual joint jog

`scripts/joint_jogger.py` is a small GUI for testing the DROID ZeroRPC server
on the NUC. It uses this repository's own environment and a direct ZeroRPC
client; it does not import the separate `control_droid` checkout. It displays
all seven joint angles, provides a slider and Jog button for each joint, and
has Open gripper and Close gripper buttons. It does not open cameras or
initialize the robot automatically. Relaunch controller explicitly restarts
the NUC arm and gripper controllers after a confirmation dialog.

Run it from this PC or any other machine that can reach the NUC on port 4242.
From the `droidauto` checkout:

```bash
conda env create -p ./.conda -f environment.yml  # first time only
conda activate ./.conda
python scripts/joint_jogger.py
```

The numbers at the left are measured joint angles. The sliders choose relative
steps between -1° and +1°; their center value of 0° is not the robot's current
position. Click a joint's Jog button to send that step. Slider movement alone
sends no robot command. Use Read state to refresh the displayed angles. The
gripper buttons send fully open (0) or fully closed (1) commands.

If the NUC responds with frozen robot feedback, click Relaunch controller only
when the robot workspace is clear and no other control session is active. The
button calls DROID's `launch_controller()` followed by `launch_robot()`, then
waits for fresh robot state before enabling motion. A failed relaunch leaves
motion disabled and reports the error in the GUI.

The script accepts steps of at most one degree, rejects a target within five
degrees of a configured Panda joint limit, and checks that the arm is still
before sending a command. It requires recent robot timestamps that advance
between two state reads, and disables motion buttons if feedback is stale,
unavailable, or a command outcome is uncertain. Read state must succeed before
motion is available again. These checks do not establish collision clearance
or make the tool suitable for unattended use;
inspect the robot's workspace and use it with an operator at the station.

## Repository layout

```text
buffer/                  Grid layout, items, and camera localization
auto_reset/              Capture, saved calibration, motion and composed tasks
configs/                 Tracked examples; machine-specific *.local.json ignored
docs/                    Design and integration notes
openscad/                Fixtures, fiducials, and fabrication automation
maskrcnn_experiments/    Dataset inspection and perception experiments
object_detector/         Separate object-detection checkout (local, ignored)
data/                    Local datasets (ignored)
```

## Environment

The tracked `environment.yml` defines the project dependencies. Create its
local Conda environment once from the repository root:

```bash
conda env create -p ./.conda -f environment.yml
conda activate ./.conda
```

The installed `.conda/` directory is ignored by Git. When `environment.yml`
changes, run `conda env update -p ./.conda -f environment.yml --prune`. The
environment includes the ZeroRPC client and Tk GUI for the joint jogger, and the NumPy,
OpenCV, Matplotlib, and Pillow libraries used by this repository's current
geometry and dataset inspection code. PyTorch and a policy inference runtime
are separate, optional additions for later experiments.

## Included evaluation data

`data/ycbv_test_bop19/test/` contains the BOP'19–24 test subset of YCB-Video:
900 annotated RGB-D images from 12 scenes, including full and visible instance
masks.

This is **held-out evaluation data**. It must not be used to train or tune a
model. Training data must be stored separately and documented before use. In
particular, the original YCB-V `data_syn` poses overlap the test images and must
not be used for a standard BOP comparison.

Dataset provenance, retrieval information, licensing, and an integrity
spot-check are recorded in
[the dataset source record](docs/YCBV_DATASET.md).

The separate `object_detector/` checkout comes from
<https://github.com/malenickymartin/object_detector> and is not part of this
repository. Generated ArUco SVGs and STLs, the local base STL, captured photos,
and downloaded datasets are also excluded from Git. To regenerate the marker
meshes on a new machine, supply a local Gridfinity base STL to
`openscad/gridfinity/aruko_square/build_aruco_stls.py` with `--base-stl`.

To inspect the labels:

```bash
conda run -n droidauto python maskrcnn_experiments/browse_ycbv.py
```

## Safety and project status

This repository is a research prototype. Code that can command the robot must
not be treated as safe for unattended use until the real platform has validated
workspace limits, command-rate and timeout handling, collision behaviour,
emergency-stop procedures, reset recovery, and operator supervision. A network
heartbeat is not a substitute for a physical emergency stop or a robot-side
safety mechanism.

## References

- [DROID developer documentation](https://droid-dataset.github.io/droid/)
- [Official DROID platform repository](https://github.com/droid-dataset/droid)
- [DROID paper](https://arxiv.org/abs/2403.12945)
- [YCB-Video / PoseCNN paper](https://arxiv.org/abs/1711.00199)
- [BOP benchmark](https://bop.felk.cvut.cz/)
