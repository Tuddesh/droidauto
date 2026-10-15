"""Geometry checks using rendered OpenCV markers and a known camera pose."""

import unittest

import cv2
import numpy as np

from buffer import Buffer, BufferItem, LocalizationError
from buffer.items import ArucoTile


CAMERA_MATRIX = np.array(
    [[1000.0, 0.0, 640.0], [0.0, 1000.0, 480.0], [0.0, 0.0, 1.0]]
)
ROTATION_VECTOR = np.array([3.03, 0.12, 0.08])
TRANSLATION = np.array([-0.055, 0.035, 0.45])


def rendered_marker_image(marker_locations):
    """Render placed markers into a white RGB image at a known buffer pose."""
    image = np.full((960, 1280), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_size_m = 0.032
    half_size = marker_size_m / 2
    unit_square = np.array(
        [[0, 0], [399, 0], [399, 399], [0, 399]], dtype=np.float32
    )

    for marker_id, center in marker_locations:
        object_corners = center + np.array(
            [
                [-half_size, half_size, 0],
                [half_size, half_size, 0],
                [half_size, -half_size, 0],
                [-half_size, -half_size, 0],
            ]
        )
        image_corners = cv2.projectPoints(
            object_corners,
            ROTATION_VECTOR,
            TRANSLATION,
            CAMERA_MATRIX,
            np.zeros(5),
        )[0].reshape(4, 2).astype(np.float32)
        marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, 400)
        homography = cv2.getPerspectiveTransform(unit_square, image_corners)
        warped = cv2.warpPerspective(
            marker_image,
            homography,
            (image.shape[1], image.shape[0]),
            flags=cv2.INTER_NEAREST,
            borderValue=255,
        )
        image = np.minimum(image, warped)

    return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)


class BufferTests(unittest.TestCase):
    def test_pose_from_two_markers_with_shifted_origin(self):
        buffer = Buffer(4, 3, CAMERA_MATRIX, origin=(1, 1))
        buffer.place(ArucoTile(0), 0, 0)
        buffer.place(ArucoTile(1), 3, 2)
        locations = [
            (item.marker_id, buffer.cell_center_m(col, row))
            for (col, row), item in [((0, 0), buffer.item_at(0, 0)), ((3, 2), buffer.item_at(3, 2))]
        ]

        pose = buffer.localize(rendered_marker_image(locations))

        np.testing.assert_allclose(pose[:3, 3], TRANSLATION, atol=0.006)
        expected_rotation = cv2.Rodrigues(ROTATION_VECTOR)[0]
        np.testing.assert_allclose(pose[:3, :3], expected_rotation, atol=0.05)
        np.testing.assert_array_equal(pose[3], [0, 0, 0, 1])

    def test_single_marker_and_layout_errors(self):
        buffer = Buffer(2, 2, CAMERA_MATRIX)
        tile = ArucoTile(4)
        buffer.place(tile, 1, 0)
        center = buffer.cell_center_m(1, 0)
        pose = buffer.localize(rendered_marker_image([(4, center)]))
        np.testing.assert_allclose(pose[:3, 3], TRANSLATION, atol=0.01)

        with self.assertRaises(ValueError):
            buffer.place(BufferItem(), 1, 0)
        same_marker_id = ArucoTile(4)
        self.assertNotEqual(tile.item_id, same_marker_id.item_id)
        buffer.place(same_marker_id, 0, 0)
        self.assertIs(buffer.item_at(0, 0), same_marker_id)
        with self.assertRaisesRegex(ValueError, "already placed"):
            buffer.place(tile, 0, 1)
        wide_item = BufferItem(width_cells=2)
        buffer.place(wide_item, 0, 1)
        self.assertIs(buffer.item_at(1, 1), wide_item)
        with self.assertRaises(LocalizationError):
            buffer.localize(np.full((960, 1280, 3), 255, dtype=np.uint8))

    def test_item_ids_are_unique_across_subclasses(self):
        first = BufferItem()
        second = BufferItem()
        marker = ArucoTile(4)
        self.assertEqual(len({first.item_id, second.item_id, marker.item_id}), 3)
        with self.assertRaises(AttributeError):
            first.item_id = second.item_id


if __name__ == "__main__":
    unittest.main()
