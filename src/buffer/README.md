# Gridfinity buffer model

`buffer/` models a rectangular Gridfinity buffer and estimates its pose from
ArUco tiles installed in known cells. This is the first draft of the buffer
side of the DroidAuto scene-reset station.

`Buffer` and `BufferItem` inherit the shared
[`PhysicalObject` dataclass](../scene/README.md). Identity, dimensions, observed
pose, reference-frame name and observation timestamp have one owner.
`SceneObject` is another subclass of the same parent for manipulation objects.
Localization methods retain their existing implementation. Shared observation
methods accept supplied poses and can mark an entity unobserved.

## Files

- `buffer.py` — grid layout, placement, and camera localization.
- `buffer_item.py` — base class for a physical object occupying one or more cells.
- `items/aruco_tile.py` — one-cell `BufferItem` carrying an ArUco marker.
- `items/` — place future concrete `BufferItem` subclasses here as physical
  items are defined.

The optional `object_id=` keyword on `Buffer`, `BufferItem`, and `ArucoTile`
allows configuration to supply persistent physical identities. Otherwise a
new UUID is generated. Buffer dimensions describe the grid reference plane
(width, depth, zero thickness), rather than the complete fixture volume.

## Coordinates

View the buffer from above, with the printed markers facing you. Cell indices
are `(column, row)`; `(0, 0)` is the lower-left cell, columns grow to the right,
and rows grow toward the far/top edge.

```text
       +Y / increasing row
        ↑
        │   (0,1)  (1,1)  ...
        │   (0,0)  (1,0)  ...
        └────────────────────→ +X / increasing column
       origin
```

The buffer frame is right-handed: **+X right, +Y toward the far edge, +Z out
of the marker faces**. `Z=0` is the plane of the marker top faces. The default
`origin=(0, 0)` is the lower-left corner of cell `(0, 0)` on that plane. If
`origin=(c, r)` is supplied, the coordinate origin moves to the lower-left
corner of cell `(c, r)` without changing cell indices.

The center of cell `(column, row)` is

```text
X = (column - origin_column + 0.5) × cell_size_mm
Y = (row    - origin_row    + 0.5) × cell_size_mm
Z = 0
```

`cell_size_mm` defaults to the nominal Gridfinity pitch of 42 mm. The pose
translation and `cell_center_m()` are in **metres**. The returned transform uses
the OpenCV camera frame: camera +X points right in the image, +Y points down,
and +Z points forward from the camera.

## Using it

Run from the `droidauto` directory in its Conda environment. Supply the camera
matrix and distortion coefficients for the same camera, resolution, and crop as
the RGB image. The example below assumes `camera_matrix` and
`distortion_coefficients` were obtained by camera calibration.

```python
import cv2
import numpy as np

from buffer import Buffer
from buffer.items import ArucoTile

grid = Buffer(
    columns=4,
    rows=3,
    cell_size_mm=42.0,
    origin=(0, 0),
    camera_matrix=camera_matrix,
    distortion_coefficients=distortion_coefficients,
    dictionary="DICT_4X4_50",
    marker_orientation="independent",
)

# Marker IDs and positions must match the physical buffer.
grid.place(ArucoTile(marker_id=0, marker_size_mm=32.0), column=0, row=0)
grid.place(ArucoTile(marker_id=1, marker_size_mm=32.0), column=3, row=2)

# cv2.imread returns BGR, so convert it before calling localize().
image_bgr = cv2.imread("buffer_photo.png")
image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

T_camera_buffer = grid.localize(image_rgb)
origin_in_camera_m = T_camera_buffer[:3, 3]

cell_center_buffer_m = grid.cell_center_m(1, 1)
cell_center_camera_m = (
    T_camera_buffer @ np.r_[cell_center_buffer_m, 1.0]
)[:3]
```

`marker_orientation="independent"` estimates each marker's printed rotation
alongside the buffer pose, using at least two known visible markers. Rotations
may be arbitrary in-plane angles; IDs still identify known cell centers.
The legacy default `"aligned"` supports one visible marker when every printed
top edge points along buffer +X. The reset station uses independent mode.

`left_buffer.localize_stereo(left_rgb, right_buffer, right_rgb,
T_left_camera_right_camera)` fits one pose from both calibrated views, sharing
each marker's yaw between eyes. Two distinct known IDs across the pair are
required; each eye may detect fewer. Missing detections contribute no corners.
Each buffer retains its own fit diagnostics; `last_joint_localization` on the
left buffer describes the combined fit. Detection and all pose fitting remain
inside `Buffer`.

`T_camera_buffer` is a 4×4 homogeneous transform. Its upper-left 3×3 block is
rotation and its upper-right three values are translation. It maps a point in
buffer coordinates to the camera frame. `localize()` uses all visible markers
whose IDs match placed `ArucoTile` items. It raises `LocalizationError` if it
cannot see a placed marker. It does not establish the robot-base frame; that
requires a separate camera-to-robot calibration.

`localize()` also sets `last_localization` to the matched `visible_marker_ids`
and `reprojection_rms_px`, plus per-marker yaw and RMS in independent mode.
It clears this metadata at the start of each call,
so a failed localization cannot leave a previous successful result visible.
`last_detection` also retains detected IDs, corners and the number of rejected
candidates, even when pose estimation fails. `draw_localization(image_rgb)`
draws those stored detections on the same input image without detecting again.
`marker_cells()` returns the marker-ID-to-cell mapping and rejects ambiguous
duplicate IDs or mismatched dictionaries. Placement still permits distinct
physical items with the same ID, but such a layout cannot be localized.

The [`auto_reset` components](../auto_reset/README.md) call this method during
a single stationary wrist-camera capture, compose its result with the measured
robot pose and existing hand-eye calibration, and persist the resulting buffer
pose. They do not implement a second localizer.

`place(item, column, row)` updates the buffer's **layout model**. It reserves
the cells occupied by an item's footprint and rejects overlapping or out of
bounds placements. Every `BufferItem` has an inherited string `object_id`.
The read-only `item_id` property is a compatibility alias for that same ID.
`place()` uses `object_id` to reject placing the
same item twice in one buffer; distinct items are allowed even if they carry
the same ArUco marker ID. Repeated marker IDs are indistinguishable in an
image, so use distinct marker IDs for tiles used to localize the buffer.
`place()` does not physically move an object. `item_at()` returns the item
assigned to a cell or `None`.

## Physical assumptions

- Markers are flat, coplanar, and centered in their cells. The model assumes
  all marker top faces lie at buffer `Z=0`.
- All installed markers use the buffer's configured ArUco dictionary. The
  default matches the generated `DICT_4X4_50` Gridfinity tiles under
  `openscad/gridfinity/aruko_square/`.
- In aligned mode the printed top edge must point along buffer +X. Independent
  mode estimates each print's rotation instead.
- `marker_size_mm` measures the outer black square including its border; it
  excludes the white quiet zone. Measure the final print if exact scale matters.
- Camera intrinsics and distortion must match the captured image. An unknown
  focal length or marker size prevents reliable metric translation from one
  RGB image.
- A single visible marker can produce a pose in aligned mode; independent mode
  needs at least two. A wider marker spread gives
  a better constrained buffer pose. Reprojection RMS measures image fit, not
  absolute accuracy; it does not resolve inconsistent physical marker placements.

The geometry/photo exercises are experiments under
`experiments/buffer_geometry/`, not a project test suite. To run the geometry
experiment explicitly:

```bash
python -m experiments.buffer_geometry.geometry_experiment
```

## Checking phone photos

`experiments/buffer_geometry/photo_cases.json` contains five proposed 6×6 layouts using subsets of
generated marker IDs 0–4: `outer_corners_center` (five markers),
`inset_corners` (four), `three_edge_midpoints` (three), `staggered_spread`
(three), and `three_corners` (three). These are not claims about the actual
physical arrangement: use a layout only if its marker positions match the photo, or
edit it first. JSON does not support comments, so each layout has a
`description` and each marker has a `placement` note in plain language.

Put each photo in the folder for the layout it shows. Keep the phone's
original filenames; any number of angles can go in the same folder:

```text
experiments/buffer_geometry/photos/
  outer_corners_center/
  inset_corners/
  three_edge_midpoints/
  staggered_spread/
  three_corners/
```

The checker automatically finds JPEG, PNG, and WebP files in these folders and
uses the folder name to select the layout. HEIC/HEIF files need conversion to
JPEG or PNG first. Cell `(0, 0)` remains the lower-left cell when viewed from
above; the four central cells are `(2, 2)`, `(3, 2)`, `(2, 3)`, and `(3, 3)`.

No JSON edit is needed to add a photo. If a particular image needs extra
settings later, add them under `photo_overrides` in `photo_cases.json`, keyed
by its path relative to `experiments/buffer_geometry/`, for example:

```json
"photo_overrides": {
  "photos/outer_corners_center/IMG_1234.JPG": {
    "expected_visible_ids": [0, 1, 2]
  }
}
```

List only markers actually visible in that image under `expected_visible_ids`.
With at least two visible markers, the checker also reports a planar
reprojection error that tests whether their positions fit the declared grid
layout. Without a calibrated camera matrix, it **cannot verify a metric pose**.
Once the Pixel 6a camera is calibrated, supply its 3×3
`camera_matrix`, `distortion_coefficients`, and
`calibration_image_size_px: [width, height]` for the same image resolution and
capture settings in the relevant photo override. Optionally set
`max_planar_reprojection_error_px` and
`max_reprojection_error_px` to fail on poor geometric agreement. The checker
reports the 4×4 `T_camera_buffer` and corner
reprojection RMS in pixels; a low RMS alone does not prove absolute pose
accuracy. Avoid repeated marker IDs in a layout used for localization.

Run the photo report from the repository root with:

```bash
python -m experiments.buffer_geometry.check_photos
```

The experimental scripts use a unittest-based harness,
but are outside `tests/` and are not an established project test suite. No new
tests have been introduced.

The pose convention and planar point ordering follow
[OpenCV's solvePnP documentation](https://docs.opencv.org/doc/doxygen/html/d5/d1f/calib3d_solvePnP.html).
