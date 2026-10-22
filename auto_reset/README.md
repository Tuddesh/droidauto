# Auto-reset components: marker hover

## One-window launcher

From the repository with its environment active:

```bash
python scripts/marker_hover.py
```

The window loads the saved calibration if available. Use **Calibrate /
Recalibrate** to capture a new image and replace the active calibration,
**Start loop** to visit markers, and **Stop** to cancel with a best-effort hold.
**One cycle** limits execution to one pass. **Initialize controllers** explicitly
restarts DROID when needed. The analyzed image, marker IDs, RMS and any errors
appear in the same window; **Raw image** switches to the exact unmodified input.
The default config is `configs/marker_hover.local.json`, resolved from the
script's repository directory. Override with `--config PATH` if needed.

Recalibration requires no manual archiving. Only a successful replacement
updates the active JSON/PNG; timestamped input images and reports remain in the
ignored attempt history. A failed calibration disables Start until another
successful calibration in that session. Stop is cooperative between RPC calls;
an outstanding RPC may take up to its timeout to return. It is not an emergency
stop.

This is the first composed robot task for the reset stack: localize the buffer
from **one stationary wrist-camera stereo pair**, save the pose, then repeatedly hover
the robot TCP 20 cm above each configured marker for five seconds. No camera
images are acquired during execution. Buffer position is trusted until an
explicit new calibration; moving the buffer invalidates it.

## Ownership and modules

| Module | Responsibility |
| --- | --- |
| `buffer.Buffer` | Layout, marker detection and pose estimation in `localize()`; visible marker IDs and reprojection RMS in `last_localization`. |
| `camera.py` | One native-resolution rectified RGB image and matching intrinsics from the selected ZED camera. |
| `robot.py` | DROID ZeroRPC connection, optional explicit initialization, fresh state, Cartesian commands. |
| `calibration.py` | Stationary capture, coordinate-frame composition and saved calibration. Calls `Buffer.localize()`; contains no detector or PnP implementation. |
| `calibration_report.py` | Raw input, marker annotations, failure details and desktop preview for every captured calibration attempt. |
| `motion.py` | Buffer-relative hover targets, rise/traverse/descend waypoints, streamed execution and measured arrival/dwell checks. |
| `task.py` | Repeat visits, five-second holds, cancellation and JSONL event records. |
| `config.py` | Physical layout, hardware selection, task and motion settings. |
| `__main__.py` | Thin `calibrate`, `preview`, `run` commands. |

These classes are importable for future reset tasks. They do not initialize
hardware on import. This task exercises the shared components; it does not yet
implement object restoration or VLA evaluation.

## Frames and calibration

### Independent marker rotations and stereo

The station uses `buffer.marker_orientation: "independent"`. `Buffer.localize()`
jointly fits the buffer pose and an arbitrary in-plane yaw for each visible
marker. The printed codes need no common orientation. IDs, cell centers,
marker sizes and pitch must still match the physical layout; markers must be
centered, flat and coplanar. Standalone `Buffer.localize()` needs two markers;
joint `Buffer.localize_stereo()` needs two distinct known IDs across the pair.
Square-marker IPPE supplies candidate plane normals; the known
grid-center relationships establish buffer axes. All corners participate in
the final pixel-space fit, with per-marker yaw and RMS recorded.

`camera.view: "stereo"` captures LEFT and RIGHT after **one ZED grab**, with a
shared timestamp. Each eye uses its own rectified intrinsics. The SDK's
rectified `stereo_transform`, requested in IMAGE coordinates and metres,
expresses the right optical frame in the left optical frame. It supplies the
eye-to-eye offset; the DROID hand-eye file is not used to align the eyes.
Both images enter a joint pixel-space fit with a common buffer pose and one
shared yaw per physical marker. Only actually detected corners participate.
An eye may see zero or one marker when the combined observations contain
enough distinct IDs. `min_visible_markers` applies to distinct IDs across a
successful joint fit. Missing IDs are listed per eye and are never fabricated
as corner observations.

If joint fitting fails, usable standalone eye poses are considered. A weak,
collinear-marker pose that disagrees with an eye having a 2D spread of markers
is discarded in favor of that better-constrained eye, with the reason logged.
For standalone fallback, both poses are converted into the left optical frame,
checked against `stereo_max_position_error_m` and
`stereo_max_rotation_error_rad`, and averaged with marker-count/pixel-fit
weights. The weights are a heuristic, not calibrated uncertainty. The combined
pose must also meet the pixel-RMS threshold in **each** accepted eye.
Inconsistent image fits or ambiguous monocular solutions can reject a
pair even when individual pixel fits look good. Per-eye fits, estimated marker
rotations, rejection reasons and stereo disagreement are saved.

The launcher displays both results. Select **Left** or **Right** to enlarge an
eye, or **Both** for the pair. **Raw image** toggles unmodified input. Attempts
save `input_left.png`, `input_right.png`, individual annotations and combined
`input.png` / `annotated.png` previews. The active calibration PNG is the chosen
reference eye; the JSON retains both eye poses and the diagnostic pair's path.
Existing calibrations from the aligned-marker model need one new calibration.

SciPy is tracked in `environment.yml` for the joint pose/marker-yaw fit. The
legacy `Buffer` API still offers `marker_orientation="aligned"` for callers
with known common print orientation; the reset station defaults to independent.

`T_parent_child` maps points from the child frame into the parent frame:

```text
T_base_buffer = T_base_tool @ T_tool_camera @ T_camera_buffer
                    │              │                │
               measured pose  existing hand-eye  Buffer.localize(image)
```

The one capture establishes the **buffer pose relative to the camera**. It does
not calibrate wrist mounting geometry. The robot-base transform additionally
needs the left rectified optical frame's pose in the DROID tool frame. Supply
a sourced mounting pose as `camera.tool_left_camera_pose` with six values
`[x, y, z, roll, pitch, yaw]` in metres/radians (fixed-axis XYZ), or use a measured
left-eye DROID hand-eye entry. A valid right-eye entry can also be converted to
the left eye using the SDK stereo transform. A camera-body mount pose must
first be composed with the optical-frame offset; it is not interchangeable
with this setting. No numerical mounting pose is supplied by default.
All-zero identity entries and identical left/right camera-to-tool entries are
rejected as invalid for this wrist-camera setup. A JSON entry's presence alone
does not establish that hand-eye calibration has been performed. One buffer
image estimates buffer-to-camera geometry; it does not determine an unknown
camera-to-tool transform. Eye fitting/agreement and diagnostic images are
still produced when this mounting pose is missing; robot-frame calibration
then reports that specific missing input and leaves Start disabled.

An explicit `camera.tool_left_camera_pose: [0, 0, 0, 0, 0, 0]` selects the
operator-requested identity mounting approximation. The SDK stereo offset
remains active. This assumption is recorded in the calibration and shown in
the launcher; it is not a measured hand-eye transform.

Pressing Calibrate establishes the operator's assertion that the arm is
stationary. Manual capture reads the tool pose once and applies no timestamp,
clock-offset, frame-age, timestamp-advance, or velocity/stationarity gates.
Timestamps are retained as metadata. Capture still rejects missing hand-eye
calibration, insufficient visible markers and excessive reprojection error.
Detection and pose fitting remain inside
`Buffer.localize()`. One image can contain multiple markers; the example
requires two distinct IDs across a joint capture. Known cells allow targets for every configured marker even if
some are not visible. A low reprojection error cannot establish physical
calibration accuracy or eliminate all single-view planar pose ambiguity.

The JSON record stores the frame transforms, intrinsics, source image name,
visible marker IDs, reprojection RMS, source timestamps and layout identity.
The image is saved alongside it. Loading rejects a different layout or camera.
There is no automatic recalibration or expiry: the station trusts that the
buffer, robot base and wrist-camera mounting remain fixed.

The hover reference is the DROID **tool/TCP origin**, 0.20 m along buffer +Z
from a marker's center on the printed plane. It is not the camera optical
center or the gripper fingertips. The scan tool orientation is preserved.

## Lab PC setup

Use DroidAuto's own environment from the lab-PC repository directory:

```bash
conda env create -p ./.conda -f environment.yml  # first installation only
conda activate ./.conda
cp configs/marker_hover.example.json configs/marker_hover.local.json
```

Edit the local JSON to match **the actual marker IDs, cells, measured marker
sizes and pitch**. The five-marker arrangement in the example is a proposed
layout, not a measurement of the buffer. Check the wrist camera serial/view
and hand-eye file path. Paths in this JSON resolve relative to the JSON file.

The camera adapter additionally needs the lab PC's installed ZED SDK Python
bindings in this same `.conda` environment. The SDK installation provides a
`get_python_api.py` installer (normally `/usr/local/zed/get_python_api.py`):

```bash
python /usr/local/zed/get_python_api.py
```

Use the installer matching the installed SDK and this environment's Python.
Do not install the unrelated PyPI package named `pyzed`. The bindings are an
optional hardware dependency, so `environment.yml` does not fetch them.
The task does not import Patrik's `droid`, Polymetis, or ROS packages. It reads
only the existing hand-eye JSON. The camera is physically attached to the lab
PC, so capture runs there.

## Capture once, inspect, run

Place the buffer, aim the wrist camera so the configured markers are visible,
and leave the arm stationary. Initialize explicitly if controllers are not
already running (`--initialize` restarts both arm and gripper controllers):

```bash
python -m auto_reset calibrate --config configs/marker_hover.local.json --initialize
```

Omit `--initialize` to attach to the running controller. Calibration itself
sends no motion commands. It opens just the configured wrist camera, captures
one stereo pair (or one image in left/right mode) and closes it. Every captured attempt, including failed ones,
saves `input.png`, `annotated.png` and `report.json` in a timestamped folder under
`runs/marker_hover/calibration_attempts/`. The unmodified input is saved before
localization; annotation uses the detections from that same `Buffer.localize()`
call. Green outlines match configured IDs; red outlines identify detected IDs
absent from the layout. The report includes matched IDs, reprojection RMS and
the actual rejection reason and thresholds.

After hardware connections close, a desktop window shows that exact image and
the result. Press **R** for the raw image, **A** for annotations and **Q** or
Escape to close. On a headless SSH session, paths are printed instead. Use
`--no-display` to deliberately suppress the window; diagnostic files are still
saved. An accepted pose is additionally saved as the main calibration JSON/PNG
in `runs/marker_hover/`. A successful recalibration replaces the active JSON/PNG
automatically; failed attempts leave them intact.

Inspect targets offline; this command opens no camera or robot connection:

```bash
python -m auto_reset preview --config configs/marker_hover.local.json
```

Run one complete cycle, or omit `--cycles` for the continuous loop:

```bash
python -m auto_reset run --config configs/marker_hover.local.json --cycles 1
python -m auto_reset run --config configs/marker_hover.local.json
```

Run uses only the saved pose. The gripper is left untouched. The five-second
dwell begins after fresh measured feedback confirms arrival and a stationary
arm. Each hold monitors feedback; drift, failed commands, stale/frozen state
or a settling timeout aborts the task. PC and NUC clocks must be synchronized.
No failed motion is automatically retried and no failure triggers controller
relaunch. Ctrl+C requests a best-effort hold at the current measured pose.

Every run writes a separate JSONL event log with the calibration/configuration,
requested and measured visit poses, arrivals, completed visits, cycles and
failures. Images, calibration JSON and run logs belong under ignored `runs/`;
the sample config and implementation are tracked. Machine-specific
`configs/*.local.json` files are ignored as well.

## Current motion scope

The planner raises along buffer +Z, aligns the tool to its scan orientation,
traverses at the transit height (30 cm in the example) and descends to the
20 cm hover. It keeps a higher starting height rather than lowering before
the traverse. Cartesian references follow a smoothstep schedule at 15 Hz with
configurable translation/rotation speeds. These are reference speed bounds;
the server and robot determine actual motion.

The executor streams `update_pose(target, False, False)` through DROID's IK.
It avoids the blocking endpoint-IK joint interpolation path and verifies
feedback against its references and final targets. It includes no hardcoded
Franka joint-limit tables. The server still owns IK and robot model limits.

This planner assumes a cleared table workspace. It has **no obstacle model,
full-arm collision checking, reachability preflight or robot-side watchdog**.
TCP clearance alone does not prove arm or tool clearance. DROID may reject an
unreachable pose during execution. Communication failure stops this client's
command stream; it cannot guarantee a robot-side stop. Ctrl+C hold is not an
emergency stop. Adding a robot model/scene and a collision-aware planning
backend remains separate integration work before unattended reset operation.

No hardware motion or camera capture was performed while implementing these
modules; the physical configuration and execution still need lab validation.

The rectified-view intrinsics convention follows the
[ZED camera calibration documentation](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/camera-calibration).
