"""Validate layout examples and check photos discovered in layout folders."""

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import cv2

from buffer.tests.check_photos import (
    CONFIG_PATH,
    check_photo,
    discover_photo_cases,
    load_cases,
)
from buffer.tests.test_buffer import CAMERA_MATRIX, rendered_marker_image


def render_image_for_layout(layout):
    """Render a few known markers to exercise the same path as phone photos."""
    from buffer import Buffer

    grid = Buffer(
        layout["columns"], layout["rows"], CAMERA_MATRIX,
        cell_size_mm=layout["cell_size_mm"], origin=tuple(layout["origin"]),
    )
    locations = [
        (marker["id"], grid.cell_center_m(marker["column"], marker["row"]))
        for marker in layout["markers"]
        if marker["id"] in (0, 1, 2)
    ]
    return rendered_marker_image(locations)


class PhotoCaseTests(unittest.TestCase):
    def test_example_layouts_use_generated_marker_ids(self):
        layouts = load_cases()["layouts"]
        expected_counts = {
            "outer_corners_center": 5,
            "inset_corners": 4,
            "three_edge_midpoints": 3,
            "staggered_spread": 3,
            "three_corners": 3,
        }
        self.assertEqual(set(layouts), set(expected_counts))
        for name, layout in layouts.items():
            with self.subTest(layout=name):
                self.assertEqual((layout["columns"], layout["rows"]), (6, 6))
                self.assertTrue(layout["description"])
                self.assertEqual(layout["dictionary"], "DICT_4X4_50")
                ids = [marker["id"] for marker in layout["markers"]]
                self.assertEqual(len(ids), expected_counts[name])
                self.assertEqual(len(set(ids)), len(ids))
                self.assertTrue(set(ids) <= set(range(5)))
                self.assertTrue(all(marker["placement"] for marker in layout["markers"]))
                cells = [(marker["column"], marker["row"]) for marker in layout["markers"]]
                self.assertEqual(len(cells), len(set(cells)))
                self.assertTrue(all(
                    0 <= column < layout["columns"] and 0 <= row < layout["rows"]
                    for column, row in cells
                ))

    def test_photo_checker_on_rendered_image(self):
        layouts = load_cases()["layouts"]
        with TemporaryDirectory() as temp_dir:
            image_path = Path(temp_dir) / "rendered.png"
            image_rgb = render_image_for_layout(layouts["outer_corners_center"])
            self.assertTrue(cv2.imwrite(str(image_path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)))
            case = {
                "image": image_path.name,
                "layout": "outer_corners_center",
                "expected_visible_ids": [0, 1, 2],
                "camera_matrix": CAMERA_MATRIX.tolist(),
                "calibration_image_size_px": [1280, 960],
                "distortion_coefficients": [0, 0, 0, 0, 0],
                "max_reprojection_error_px": 3.0,
                "max_planar_reprojection_error_px": 3.0,
            }
            report = check_photo(case, layouts, Path(temp_dir))
            detection_only = check_photo(
                {**case, "camera_matrix": None}, layouts, Path(temp_dir)
            )
        self.assertFalse(report["issues"], report)
        self.assertEqual(report["mode"], "pose")
        self.assertLess(report["reprojection_rms_px"], 3.0)
        self.assertLess(report["planar_reprojection_rms_px"], 3.0)
        self.assertFalse(detection_only["issues"], detection_only)
        self.assertEqual(detection_only["mode"], "detection_only_no_camera_calibration")
        self.assertIsNone(detection_only["pose_camera_buffer"])
        self.assertLess(detection_only["planar_reprojection_rms_px"], 3.0)

    def test_discovery_accepts_arbitrary_names_and_multiple_angles(self):
        config = load_cases()
        with TemporaryDirectory() as temp_dir:
            folder = Path(temp_dir) / "photos" / "outer_corners_center"
            folder.mkdir(parents=True)
            (folder / "IMG_20261012_123456.JPG").touch()
            (folder / "side-view.png").touch()
            cases = discover_photo_cases(config, Path(temp_dir))
        self.assertEqual(len(cases), 2)
        self.assertTrue(all(case["layout"] == "outer_corners_center" for case in cases))

    def test_discovered_photos(self):
        config = load_cases()
        cases = discover_photo_cases(config, CONFIG_PATH.parent)
        if not cases:
            self.skipTest("No photos in the layout folders yet")
        for case in cases:
            with self.subTest(image=case["image"]):
                report = check_photo(case, config["layouts"], CONFIG_PATH.parent)
                self.assertFalse(report["issues"], report)


if __name__ == "__main__":
    unittest.main()
