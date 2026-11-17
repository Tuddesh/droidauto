"""Hardware-free regression checks for the direct placement experiment."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

from auto_reset.localization import localize_cubes

from auto_reset.experiments import PlacementExperiment
from auto_reset.io import StationConfig
from auto_reset.manager import ResetManager
from scene.transforms import pose_to_transform

ROOT = Path(__file__).resolve().parents[1]


class SimulatedManager(ResetManager):
    def observe(self, observations=None, **kwargs):
        if kwargs.get('detect_objects'):
            self.captures += 1
            assert kwargs.get('retry_missing') is False
            if self.drop_after_transfer and self.transfers:
                self.objects[self.transfers[-1]].mark_unobserved()
        return super().observe()

    def _execute(self, actions):
        key = next(iter(self.target_configuration.poses))
        self.transfers.append(key)
        self.objects[key].update_pose(self.buffer.pose @ self.target_configuration.pose_for(key), 'base', 1.)


def make_manager():
    config = StationConfig(ROOT / 'configs/station.example.json')
    robot = SimpleNamespace(
        read_state=lambda: SimpleNamespace(joints=np.zeros(7), tool_pose=np.eye(4)),
        command_joints=lambda *a, **k: None,
        wait_until_stopped=lambda **k: None,
        joint_target_for_pose=lambda *a, **k: [0.] * 7,
        last_ik={},
    )
    manager = SimulatedManager(buffer=config.make_buffer(), objects=config.make_objects(),
                               station_config=config, robot=robot, camera=object(), planner=config.make_planner())
    manager.buffer.update_pose(np.eye(4), 'base', 1.)
    manager.captures = 0
    manager.transfers = []
    manager.drop_after_transfer = False
    return manager


class PlacementTests(unittest.TestCase):
    def test_catalog_covers_dictionary_and_preserves_existing(self):
        objects = make_manager().objects
        self.assertEqual(len(objects), 1024)
        self.assertEqual({o.marker_id for o in objects.values()}, set(range(1024)))
        self.assertEqual(objects['cube-1'].marker_id, 5)
        self.assertEqual(objects['cube-4'].marker_id, 27)
        self.assertEqual(objects['cube-DICT_ARUCO_ORIGINAL-1023'].manipulation, objects['cube-1'].manipulation)
        objects['cube-DICT_ARUCO_ORIGINAL-1023'].manipulation['tool_to_grasp_m'] = 1
        self.assertEqual(objects['cube-1'].manipulation['tool_to_grasp_m'], .167)

    def test_actual_detector_localizes_new_dictionary_id(self):
        manager = make_manager()
        config = manager.station_config
        gray = np.full((600, 600), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        gray[220:380, 220:380] = cv2.aruco.generateImageMarker(dictionary, 1023, 160)
        frame = SimpleNamespace(image_rgb=cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB),
                                view='left', serial='test', timestamp_s=1., left_camera_eye=None,
                                camera_matrix=np.array([[1836., 0, 300], [0, 1836., 300], [0, 0, 1]]),
                                distortion=np.zeros(5))
        base_camera = pose_to_transform([.105, .105, .4, np.pi, 0, 0])
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(output):
            poses, report = localize_cubes([frame], config.aruco_detector, config.aruco_dictionary_names,
                                           manager.objects, base_camera, Path(directory),
                                           planar_scene=config.data['planar_scene'], buffer=manager.buffer,
                                           retry_missing=False)
        self.assertNotIn('not detected', output.getvalue())
        self.assertIn('cube-DICT_ARUCO_ORIGINAL-1023', poses)
        self.assertEqual(len(poses), 1)
        self.assertEqual(len(report['eyes']['left']['detection_passes']), 1)

    def test_clearance_includes_diagonals_actual_footprint_and_source_exclusion(self):
        manager = make_manager()
        experiment = PlacementExperiment(manager, [0.]*7, [], Path('/tmp/unused'))
        manager.objects['cube-2'].update_pose(pose_to_transform([.105, .105, .01, 0, 0, 0]), 'base', 1.)
        free = experiment._free_cells('cube-1')
        self.assertNotIn((3, 3), free)  # diagonal neighbor
        self.assertIn((4, 2), free)  # one complete intervening cell
        # A missed placement intrudes into the reserved empty cell.
        manager.objects['cube-2'].update_pose(pose_to_transform([.125, .105, .01, 0, 0, 0]), 'base', 1.)
        self.assertNotIn((4, 2), experiment._free_cells('cube-1'))
        self.assertIn((4, 2), experiment._free_cells('cube-2'))

    def test_loop_measures_once_and_rotates_cubes(self):
        manager = make_manager()
        for i, key in enumerate(['cube-1', 'cube-2', 'cube-DICT_ARUCO_ORIGINAL-1023']):
            manager.objects[key].update_pose(pose_to_transform([-.2, i*.1, .01, 0, 0, 0]), 'base', 1.)
        with tempfile.TemporaryDirectory() as directory:
            summary = PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=3, seed=7).run()
            self.assertEqual(summary['completed_trials'], 3)
            self.assertEqual(summary['placement_successes'], 3)
            self.assertEqual(manager.captures, 4)  # initial + exactly one measurement per transfer
            self.assertEqual(len(set(manager.transfers)), 3)
            self.assertEqual(len((Path(directory)/'trials.csv').read_text().splitlines()), 4)
            self.assertFalse(manager._run_lock.locked())

    def test_missing_cube_records_trial_and_stops(self):
        manager = make_manager()
        manager.objects['cube-1'].update_pose(pose_to_transform([-.2, 0, .01, 0, 0, 0]), 'base', 1.)
        manager.drop_after_transfer = True
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, 'Cubes missing'):
                PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=3, seed=7).run()
            summary = json.loads((Path(directory)/'summary.json').read_text())
            self.assertEqual(summary['completed_trials'], 1)
            self.assertEqual(summary['placement_successes'], 0)
            self.assertEqual(len(manager.transfers), 1)
            self.assertFalse(manager._run_lock.locked())

    def test_blocked_fixed_target_does_not_move(self):
        manager = make_manager()
        manager.objects['cube-1'].update_pose(pose_to_transform([-.2, 0, .01, 0, 0, 0]), 'base', 1.)
        manager.objects['cube-2'].update_pose(pose_to_transform([.105, .105, .01, 0, 0, 0]), 'base', 1.)
        with tempfile.TemporaryDirectory() as directory:
            summary = PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=3,
                                          object_id='cube-1', target_cells=[[3, 3]]).run()
            self.assertEqual(summary['status'], 'capacity_reached')
            self.assertEqual(manager.transfers, [])


if __name__ == '__main__':
    unittest.main()
