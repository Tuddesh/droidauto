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

## Repository layout

```text
buffer/                  Grid layout, items, and camera localization
docs/                    Design and integration notes
openscad/                Fixtures, fiducials, and fabrication automation
maskrcnn_experiments/    Dataset inspection and perception experiments
object_detector/         Separate object-detection checkout (local, ignored)
data/                    Local datasets (ignored)
```

## Environment

The Conda environment is named `droidauto`:

```bash
conda activate droidauto
python -c "import torch, torchvision; print(torch.__version__, torchvision.__version__)"
```

At setup time (2026-09-28), it contained Python 3.11.16, PyTorch
2.14.0+cpu, and TorchVision 0.29.0+cpu. CPU is sufficient for inspection and
smoke tests; training or low-latency VLA inference may require a suitable GPU or
a separate inference server.

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
