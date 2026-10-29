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
[DROID server and client architecture](docs/DROID_SERVER.md) for the current
interface and integration notes.

## Project layout

```text
droidauto/
├── pyproject.toml
├── environment.yml
├── src/
│   ├── auto_reset/
│   │   ├── manager.py
│   │   ├── planner.py
│   │   ├── target_configuration.py
│   │   ├── hardware/        DROID and ZED adapters
│   │   ├── calibration/     Buffer and hand-eye calibration
│   │   └── io/              Station configuration and event logs
│   ├── scene/              Shared physical and manipulation-object models
│   └── buffer/             Gridfinity layout, fixtures and localization
├── tests/                  Reserved; no project tests yet
├── scripts/                Reserved; no production entry points yet
├── experiments/            Independent perception utilities
├── configs/                Station examples; local settings ignored
├── docs/
└── openscad/
```

`src/` directly contains the three installable packages. `droidauto` is the
project/distribution name; there is no additional nested `droidauto/` package.
Only reset-manager, planner and target-configuration modules remain at the
root of `src/auto_reset/`.

## Framework status

The new reset operation is **not implemented**. Its workflow methods raise
`NotImplementedError`; constructing the data objects does not operate hardware.

- [`PhysicalObject` and scene models](src/scene/README.md) share identity,
  dimensions and observed pose/provenance across buffers, fixtures and objects.
- [`Buffer`](src/buffer/README.md) retains its existing layout/localization API.
- [`ResetManager`](src/auto_reset/README.md) owns reset orchestration and its
  internal outcome state, with a planner, target configuration, supplied
  localization adapter and concrete camera/robot collaborators.

No unused abstract driver layer is introduced. Localization-tool integration,
grasp selection, reset planning, execution and verification are future work.
The retained hardware adapters are explicit integrations rather than a reset
implementation.

## Installation

From this repository, create/activate its own Conda environment, then install
this local project in editable mode:

```bash
conda env create -p ./.conda -f environment.yml  # first installation only
conda activate ./.conda
python -m pip install --no-deps -e .
```

`environment.yml` supplies the environment dependencies; `pyproject.toml`
defines the Python distribution and discovers packages under `src/` only.
Editable installation makes imports independent of the shell's current folder.
The lab's ZED SDK/Python bindings remain a separately installed hardware
requirement. No application initializes hardware on import.

Update environment dependencies when needed:

```bash
conda env update -p ./.conda -f environment.yml --prune
```

## Configuration

```bash
cp configs/station.example.json configs/station.local.json
```

Use the station's actual marker layout, dimensions and measured camera mounting
transform. The example contains no reset method or task-object loader.
The local manual hand-eye capture experiment lives under the ignored
`experiments/calibration/` directory. `scripts/` is empty.
There is no executable reset CLI until the manager workflow is implemented.

## Experiments

Experiments are local, ignored material independent of the reset manager.
The [architecture document](docs/ARCHITECTURE.md) describes the current
source organization, object ownership and implementation status.

## Data and supporting work

Datasets, captures, model outputs, environments, and generated meshes are
ignored. The existing YCB-V subset is held-out evaluation data; see
[dataset provenance](docs/YCBV_DATASET.md). The separate `object_detector/`
checkout is unchanged.

From the repository root, the independent dataset browser is:

```bash
python -m experiments.perception.browse_ycbv
```

The exploratory buffer geometry/photo code lives under
`experiments/buffer_geometry/`. `tests/` is empty; there is no project test suite
yet. Fabrication sources remain under `openscad/gridfinity/aruko_square/`.

## References

- [DROID developer documentation](https://droid-dataset.github.io/droid/)
- [DROID integration notes](docs/DROID_SERVER.md)
- [Reset architecture](src/auto_reset/README.md)
