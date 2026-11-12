# DroidAuto — automatic scene resetting on DROID

DroidAuto implements a reset operation: observe the scene and accept a target
configuration, plan object transfers, execute them through DROID, and verify
placements from fresh observations after each manipulation. VLA execution and
model evaluation orchestration are outside this project's implementation scope.

Cube poses come from camera ArUco detection or supplied robot-base coordinates.
The existing buffer detector captures the stationary buffer once and saves its
pose and source image; subsequent resets reuse that fixed pose.

## Current behavior

- Cube: **40 x 40 x 20 mm**, centered **40 mm** top marker,
  **DICT_ARUCO_ORIGINAL / ID 5**, aligned with the cube edges. Origin is the geometric center.
- Camera and supplied object observations use the **robot base frame**; the buffer is detected.
- `planar_scene` fixes the table at base Z = -40 mm and the buffer marker/support
  surface 5 mm above it. Localization fits X/Y/yaw only, using both stereo eyes.
  Cube centers are at -30 mm on the table and -25 mm inside the buffer footprint;
  roll/pitch are zero. Targets use the same geometry, without an extra 5 mm offset.
- Targets are object poses in **buffer coordinates**, or free buffer cells with
  an explicit support-plane height.
- Configured grasps are **T_object_tool**, using DROID's actual reported tool
  frame. A top-grasp helper accepts a measured tool-to-gripping-point offset.
- Planning creates lift/transit/pick/place/gripper endpoints. There is no dense
  interpolation or client-side positional-error gate. cuRobo solves endpoint IK
  on the local CUDA GPU; DROID executes the resulting joint targets. The client
  waits for joint motion to settle before sending another endpoint.
- Reset execution repeats scene observation, target comparison and one
  pick-and-place action, returning to the observation pose after each action.
  It completes only when a fresh measured scene meets all requested targets.
  Off-target placements are replanned without a fixed retry limit; cancellation
  and hardware/capture failures stop the operation.
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
├── third_party/alignpose_project/  AlignPose submodule (custom-objects)
├── Dockerfile
├── environment.yml
└── pyproject.toml
```

`src/` directly contains the three installable packages; there is no nested
`droidauto` Python package. `scripts/` contains manual utilities.
Existing local experiments, datasets, captures and environments are ignored.

## Installation

Initialize the AlignPose source submodule after cloning:

```bash
git submodule update --init third_party/alignpose_project
```

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
`objects[].manipulation.tool_to_grasp_m` is **0.167 m** from the flange
(`panda_link8`, DROID's configured tool frame) to the grasp center. The grasp
center targets the cube center by default (`grasp_center_z_m=0`). Height clipping
separately uses the **0.178 m** closed-fingertip reach.
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
Camera-based execution first approaches a scan pose 250 mm from the top marker,
centered between the stereo cameras and looking down using the measured camera
mount. It captures/reviews a close-up, then replans the transfer from that pose
estimate. A plan preview ends at the rescan action; pickup coordinates follow
the new observation. `reset.scan_distance_m` sets the distance;
`reset.rescan_before_pick=false` disables this stage. Supplied/dummy poses skip it.
If the close-up cannot localize the cube, the transfer uses its coarse pose and
logs a warning.

To teach a target, arrange the scene and run `python scripts/teach_scene.py`.
Enter a name, capture and review the frozen stereo images, then save. All selected
objects must be detected; `--objects cube-1` selects a subset and `--scan-buffer`
refreshes a moved buffer. Targets are buffer-relative and stored under ignored
`runs/scenes/<name>/reset.json`, with capture diagnostics. The script prints the
reset command and sends no movement or controller-initialization commands.
After changing the height model, reteach the scene; the teacher replaces an
incompatible buffer calibration after a new reviewed capture. Old 3D scenes are
rejected when loading a planar target.

For persistent manager state, omit `--request` and send one JSON request per
stdin line. `--execute` enables the robot connection and `reset` requests:

```bash
python -m auto_reset --config configs/station.local.json --execute
```

Controller relaunch is separate and explicit: add `--initialize-controllers`
only when controller initialization is needed. Detailed request examples and
Python methods are in [the reset framework](src/auto_reset/README.md).

A one-shot reset process exits after measured completion, cancellation or an
error. The JSON-lines process retains the manager and camera between requests.
An explicit `verify` request can reassess a completed reset from supplied poses.

## Docker

```bash
docker build -t droidauto .
docker run --rm -i --gpus all --network host \
  -v "$PWD/configs:/app/configs:ro" \
  -v "$PWD/runs:/app/runs" \
  droidauto --config configs/station.local.json
```

The container includes pinned CUDA PyTorch and cuRobo dependencies. Execution
requires an NVIDIA GPU and the NVIDIA Container Toolkit (`--gpus all`); CPU IK
fallback is disabled. It can plan using supplied poses and a saved buffer
calibration. Feedback execution also requires an open camera with ZED bindings;
the current image contains no camera SDK, so run that flow in the lab environment.
Only source, packaging files and example configs enter the build context;
local configs, datasets, environments and recordings are excluded.

## Supporting material

- [Current architecture](docs/ARCHITECTURE.md)
- [Scene and cube methods](src/scene/README.md)
- [Buffer geometry/localization](src/buffer/README.md)
- [DROID integration](docs/DROID_SERVER.md)
- [Dataset provenance](docs/YCBV_DATASET.md)
