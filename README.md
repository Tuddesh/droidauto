# DroidAuto — automatic scene resetting on DROID

DroidAuto implements a reset operation: observe the scene and accept a target
configuration, plan object transfers, execute them through DROID, and verify
placements from separately supplied final measurements. VLA execution and
model evaluation orchestration are outside this project's implementation scope.

Cube poses come from camera ArUco detection or supplied robot-base coordinates.
The existing buffer detector captures the stationary buffer once and saves its
pose and source image; subsequent resets reuse that fixed pose.

## Current behavior

- Cube: **40 x 40 x 20 mm**, centered **38 mm** top marker,
  **DICT_ARUCO_ORIGINAL / ID 27**, aligned with the cube edges. Origin is the geometric center.
- Camera and supplied object observations use the **robot base frame**; the buffer is detected.
- Targets are object poses in **buffer coordinates**, or free buffer cells with
  an explicit support-plane height.
- Configured grasps are **T_object_tool**, using DROID's actual reported tool
  frame. A top-grasp helper accepts a measured tool-to-gripping-point offset.
- Planning creates lift/transit/pick/place/gripper endpoints. There is no dense
  interpolation or client-side positional-error gate. DROID owns IK and motion;
  the client waits for joint motion to settle before sending another endpoint.
- After execution, state is `awaiting_verification`, with `success: null`.
  Only supplied final measurements can establish success or failure.
- Events, commands, errors and final placement deviations are logged as JSONL.

This is a simple endpoint transfer planner, not collision-aware path planning.
The corridor must be clear. Joint-space moves between endpoints do not promise
straight Cartesian paths. No automatic retry of a motion command is performed.
Hardware integration limitations are described in [DROID integration](docs/DROID_SERVER.md).

## Layout

```text
droidauto/
├── src/
│   ├── auto_reset/
│   │   ├── manager.py, planner.py, target_configuration.py
│   │   ├── __main__.py       JSON request/container entry point
│   │   ├── hardware/         DROID and independent ZED adapters
│   │   ├── calibration/      Existing buffer and hand-eye utilities
│   │   └── io/               Configuration and event logging
│   ├── scene/                Shared physical objects and cube grasp geometry
│   └── buffer/               Grid geometry, fixtures and buffer localization
├── scripts/                  Interactive manual hardware tools
├── configs/                  Examples; local JSON ignored
├── docs/                     Current state and interfaces
├── Dockerfile
├── environment.yml
└── pyproject.toml
```

`src/` directly contains the three installable packages; there is no nested
`droidauto` Python package. `scripts/` contains manual utilities.
Existing local experiments, datasets, captures and environments are ignored.

## Installation

```bash
conda env create -p ./.conda -f environment.yml
conda activate ./.conda
python -m pip install --no-deps -e .
```

No hardware initializes on import. Buffer capture uses the lab PC's ZED SDK
bindings and measured camera mounting transform. Planning with a saved buffer
calibration does not open a camera. ROS and Patrik's environment are not required.

## Configure and run

```bash
cp configs/station.example.json configs/station.local.json
cp configs/reset.example.json configs/reset.local.json
```

The numeric example poses/transit settings are illustrative. The configured
`objects[].manipulation.tool_to_grasp_m` is **0.172 m**, measured from the
flange (`panda_link8`, DROID's configured tool frame) to the gripping point.
Supply actual base-frame observations, starting tool pose, buffer support
height and transit height before a physical transfer.
Buffer detection requires the measured wrist-camera mounting transform in the
local station config. `--scan-buffer` replaces the saved buffer calibration.

Plan without sending movement commands:

```bash
python -m auto_reset --config configs/station.local.json --request configs/reset.local.json
```

`python -m auto_reset --dummy` supplies only example object coordinates and
plans using the detected/cached buffer. Add `--scan-buffer` for a fresh capture
and `--execute` to perform the transfer.

`python -m auto_reset --detect-objects --review-buffer` instead detects the real
cube, shows frozen scan images and plans its transfer. Add `--execute` to move.

For persistent manager state, omit `--request` and send one JSON request per
stdin line. `--execute` enables the robot connection and `reset` requests:

```bash
python -m auto_reset --config configs/station.local.json --execute
```

Controller relaunch is separate and explicit: add `--initialize-controllers`
only when controller initialization is needed. Detailed request examples and
Python methods are in [the reset framework](src/auto_reset/README.md).

A one-shot reset process exits with verification pending. To subsequently
verify that same reset, keep the JSON-lines process running and submit `verify`.
No object motion or fresh observation is inferred from a successful RPC.

## Docker

```bash
docker build -t droidauto .
docker run --rm -i --network host \
  -v "$PWD/configs:/app/configs:ro" \
  -v "$PWD/runs:/app/runs" \
  droidauto --config configs/station.local.json
```

The default container can plan using supplied object poses and a saved buffer
calibration. Add `--execute` to enable transfers. Fresh buffer capture runs in
the lab environment with ZED bindings; the image contains no camera SDK.
Only source, packaging files and example configs enter the build context;
local configs, datasets, environments and recordings are excluded.

## Supporting material

- [Current architecture](docs/ARCHITECTURE.md)
- [Scene and cube methods](src/scene/README.md)
- [Buffer geometry/localization](src/buffer/README.md)
- [DROID integration](docs/DROID_SERVER.md)
- [Dataset provenance](docs/YCBV_DATASET.md)
