# DroidAuto — automatic scene resetting on DROID

DroidAuto restores a physical manipulation scene to a requested configuration
on the [DROID robot platform](https://droid-dataset.github.io/droid/).

## Scope

The reset operation accepts a target configuration, observes the scene, plans
and executes object transfers, verifies the resulting placements, and records
its outcome. An external caller decides when to request a reset.

The runtime boundary delegates robot control to the DROID server and image
localization to the supplied tools/service. Scene models, target definitions,
reset planning, sequencing, verification and logging belong to DroidAuto.
Application/container startup and localization-service wiring are not yet
implemented.

```text
reset request + target configuration
                ↓
        observe physical scene
                ↓
        plan and execute reset
                ↓
        observe and verify result
                ↓
        reset outcome + measurements
```

## Objectives

1. Restore objects to a defined physical configuration.
2. Integrate the supplied localization and existing robot-control stack.
3. Measure reset reliability and final placement deviations.
4. Preserve reset observations, actions, timing, outcomes and failures.

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
├── tests/                  Manual marker-pose camera diagnostic
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
`experiments/buffer_geometry/`. The [manual camera test](tests/README.md) uses
the current camera object to outline markers and display camera-relative poses.
There is no automated project test suite. Fabrication sources remain under
`openscad/gridfinity/aruko_square/`.

## References

- [DROID developer documentation](https://droid-dataset.github.io/droid/)
- [DROID integration notes](docs/DROID_SERVER.md)
- [Reset architecture](src/auto_reset/README.md)
