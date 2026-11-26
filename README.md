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
- `planar_scene` fixes the table at base Z = -40 mm. Buffer heights above the
  table are separate: bottom 40 mm, cube-support surface 48 mm, marker plane
  60 mm. Their settings are `buffer_bottom_height_m`, `buffer_surface_height_m`
  and `buffer_marker_height_m` in the station config. Localization fits X/Y/yaw
  using both eyes. Cube centers are at base Z = -30 mm on the table and +18 mm
  on the buffer; roll/pitch are zero. The buffer coordinate origin lies on its
  marker plane at base Z = +20 mm, so its support surface is at local Z = -12 mm.
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
  Off-target placements are replanned, at most `reset.max_transfers_per_cube`
  transfers per cube (default 3); a cube still off target after that ends the
  request unsuccessfully. If a target cube is not detected, the scene is observed
  again without moving, at most `reset.max_reobservations` times in a row (default 2);
  then the request ends unsuccessfully. Cancellation and hardware/capture failures
  stop the operation.
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

To collect one visible table cube into a free buffer cell, run:

```bash
python -m auto_reset --collect-one --execute --initialize-controllers --review-buffer
```

This first moves to `reset.home_joints_rad` and waits for motion to stop before
scanning. It reuses the saved buffer calibration, discovers marker IDs using the
configured cube template, and prefers available cells near the buffer center.
Installed fixtures and cells overlapped by other visible cubes are excluded.
The existing reset loop performs the transfer, returns to the home camera
pose and measures the result. The command exits after that reset. Omit
`--execute` to capture from the current pose and preview the next planning stage
without moving. With `--scan-buffer`, collection recaptures the buffer after homing.

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

## Placement experiments

Run the direct placement experiment from the lab environment:

```bash
python scripts/run_placement_experiment.py --execute
```

The runner scans all visible movable cubes at the saved DROID-home joints, picks
one, performs the established above-cube scan to refine its pickup pose, then
plans and executes one transfer to a clear grid cell. It returns to the same home
joints and measures signed X/Y error, XY distance and yaw error. Scene and
close-up scans fit five captures each by default. Incomplete scene scans get up
to three attempts, retaining fresh detections across those attempts. That scene
measurement also supplies the next pickup and obstacle map. There is no random
table staging, corrective transfer or repeated buffer calibration. Camera and
CUDA solver instances stay open.

`cube_dictionary_template` names a configured cube whose physical geometry,
marker mounting and grasp apply to every ID in its dictionary. Existing explicit
identities are preserved; other IDs receive `cube-DICT_ARUCO_ORIGINAL-<id>` names.
The example covers all 1024 original-dictionary IDs. The runner also enables this
catalog for older four-cube local configs. Absent catalog entries are candidates,
not missing physical objects; experiment scans use native-resolution detection
without upscaling to search for all 1024 IDs.

Targets leave one whole cell clear in every direction (including diagonals)
from other cubes' measured, rotated footprints and installed grid fixtures.
Previously placed cubes stay as obstacles; failed placements use their actual
footprints. Each visible cube gets a turn before previously used cubes are
selected again when feasible. When all cubes have been used, the runner can
relocate them to other clear cells until `--runs` attempts have been logged.
Failed detections, scans, planning and placement checks are recorded per trial;
the batch continues with other visible cubes. A blocked fixed target skips that
trial and allows the next target to be attempted. Missing cubes retain their
last known footprints as obstacles; an unobserved transferred cube also reserves
its destination until detected again. Motion failures require fresh stationary
feedback and an empty gripper before returning home and continuing; unavailable
feedback or an uncertain held cube prevents recovery. Cancellation still stops
the batch. Clearance checks cover destination footprints; the transit corridor
must remain clear for endpoint robot motion.

`--initialize-controllers` explicitly initializes controllers; `--review` enables
frozen scan review. `--runs`, `--seed` and `--object cube-1` override selection;
`--captures` and `--scan-attempts` override burst size and scene retry count.
`--object` still observes all other cubes as obstacles. `--scan-buffer` refreshes
the otherwise cached buffer calibration before starting. Fixed `target_cells`
are attempted in order (cycling) and obey the same clearance rules.
Home joints are in `configs/placement_experiment.example.json`. Results under
ignored `runs/placement_experiments/` include flushed trial CSV, JSONL events,
the station configuration and initial scene, and a summary. Ctrl+C retains
completed measurements. Normal feedback resets keep their existing behavior.

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
