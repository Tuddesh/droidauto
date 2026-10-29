# Manual camera test

With the project installed in its active environment, from the repository root:

```bash
python tests/marker_pose_preview.py
```

The test uses `ZedWristCamera` and defaults to the **left wrist camera**. It
loads `configs/station.local.json` when present, otherwise the station example.
It opens no DROID connection and does not initialize or move the robot. FCI is
not required for this camera-only test. Close other camera viewers first.

The preview outlines 5x5 markers and shows IDs, camera-relative XYZ in mm,
marker XYZ roll/pitch/yaw in degrees, and the converted cube center. Camera
axes are +X right, +Y down, +Z forward. All marker poses assume the cube's
38 mm marker size. Other marker sizes will produce an incorrect metric scale.
The cube center is 10 mm inward along the marker normal; its edge-aligned
orientation is not inferred from an unknown mounting yaw.

Auto mode tries the four standard `DICT_5X5_*` dictionaries plus
`DICT_ARUCO_ORIGINAL` (also a 5x5 bit pattern), and merges matching
ID/corner detections. An individual code may belong to several of these
dictionaries; the diagnostic reports the matches rather than claiming to know
which size dictionary was originally used to print it. Once the cube ID is
known, restrict the display with `--marker-id ID`.

The project's observed cube marker is **`DICT_ARUCO_ORIGINAL`, ID 13**.
To select it explicitly:

```bash
python tests/marker_pose_preview.py --dictionary DICT_ARUCO_ORIGINAL --marker-id 13
```

Keys: **Q/Esc** quit, **P** print values, **S** save images/JSON, **R** select an
automatic-exposure metering region, **A** restore full-image metering. Exposure
and gain remain automatic. Captures are saved under ignored
`runs/marker_pose_preview/`.

For a finite capture without a desktop window:

```bash
python tests/marker_pose_preview.py --no-display --frames 1
```

This is a local visual diagnostic, not the production localization service or
an automated project test suite.
