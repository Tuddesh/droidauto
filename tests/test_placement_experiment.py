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

from auto_reset.localization import localize_objects

from auto_reset.experiments import PlacementExperiment
from auto_reset.io import StationConfig
from auto_reset.manager import ResetManager
from scene.transforms import pose_to_transform

ROOT = Path(__file__).resolve().parents[1]


class SimulatedManager(ResetManager):
    def observe(self, observations=None, **kwargs):
        if kwargs.get('detect_objects'):
            self.captures += 1
            self.capture_counts.append(kwargs['capture_count'])
            selected = self.objects if kwargs.get('object_ids') is None else {
                key: self.objects[key] for key in kwargs['object_ids']}
            is_closeup = kwargs.get('object_ids') is not None
            if not is_closeup:
                assert kwargs.get('retry_missing') is False
            if self.scan_errors:
                self.scan_errors -= 1
                raise RuntimeError('camera fitting failed')
            allowed = set(self.physical)
            if not is_closeup and self.visibility_schedule:
                allowed = self.visibility_schedule.pop(0)
            for key, obj in selected.items():
                obj.mark_unobserved()
                if key in allowed and key in self.physical and not (self.drop_after_transfer and key in self.transfers):
                    obj.update_pose(self.physical[key].copy(), 'base', 1.)
            self._state['last_observation'] = {'tool_left_camera': np.eye(4).tolist(), 'eyes': {}}
        return super().observe()

    def plan(self, target_configuration, **kwargs):
        actions = super().plan(target_configuration, **kwargs)
        self.plans.append((kwargs.get('rescan_objects'), actions))
        return actions

    def _execute(self, actions):
        if any(action['type'] == 'rescan_object' for action in actions):
            self.scans.append(next(iter(self.target_configuration.poses)))
            if self.fail_closeup and len(self.scans) == 1:
                raise RuntimeError('closeup failed')
            key = next(iter(self.target_configuration.poses))
            self.physical[key][0, 3] += self.closeup_shift
            return super()._execute(actions)
        key = next(iter(self.target_configuration.poses))
        self.transfers.append(key)
        if self.fail_transfer and len(self.transfers) == 1:
            raise RuntimeError('move failed')
        self.physical[key] = self.buffer.pose @ self.target_configuration.pose_for(key)
        self.physical[key][0, 3] += self.placement_error


def seed_scene(manager):
    manager.physical = {key: obj.pose.copy() for key, obj in manager.objects.items() if obj.pose is not None}


def make_manager():
    config = StationConfig(ROOT / 'tests/fixtures/station.cubes.json')
    robot = SimpleNamespace(
        read_state=lambda: SimpleNamespace(joints=np.zeros(7), tool_pose=np.eye(4)),
        command_joints=lambda *a, **k: None,
        wait_until_stopped=lambda **k: None,
        joint_target_for_pose=lambda *a, **k: [0.] * 7,
        last_ik={},
        open_gripper=lambda: None,
        require_stationary=lambda: None,
        read_gripper_position=lambda: 0.,
    )
    manager = SimulatedManager(buffer=config.make_buffer(), objects=config.make_objects(),
                               station_config=config, robot=robot, camera=SimpleNamespace(), planner=config.make_planner())
    manager.buffer.update_pose(np.eye(4), 'base', 1.)
    manager.captures = 0
    manager.transfers = []
    manager.drop_after_transfer = False
    manager.physical = {}
    manager.capture_counts = []
    manager.scans = []
    manager.scan_errors = 0
    manager.fail_closeup = False
    manager.fail_transfer = False
    manager.placement_error = 0.
    manager.closeup_shift = 0.
    manager.plans = []
    manager.visibility_schedule = []
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
                                view='left', serial='test', timestamp_s=1., left_camera_eye=None, image_settings=None,
                                camera_matrix=np.array([[1836., 0, 300], [0, 1836., 300], [0, 0, 1]]),
                                distortion=np.zeros(5))
        base_camera = pose_to_transform([.105, .105, .4, np.pi, 0, 0])
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(output):
            poses, report = localize_objects([frame], config.aruco_detector, config.aruco_dictionary_names,
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

    def test_loop_restores_closeup_and_uses_bursts(self):
        manager = make_manager()
        for i, key in enumerate(['cube-1', 'cube-2', 'cube-DICT_ARUCO_ORIGINAL-1023']):
            manager.objects[key].update_pose(pose_to_transform([-.2, i*.1, .01, 0, 0, 0]), 'base', 1.)
        seed_scene(manager)
        with tempfile.TemporaryDirectory() as directory:
            summary = PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=3, seed=7).run()
            self.assertEqual(summary['completed_trials'], 3)
            self.assertEqual(summary['placement_successes'], 3)
            self.assertEqual(manager.captures, 7)  # initial + closeup + measurement per trial
            self.assertEqual(manager.scans, manager.transfers)
            self.assertEqual(set(manager.capture_counts), {5})
            self.assertEqual(len(set(manager.transfers)), 3)
            self.assertEqual(len((Path(directory)/'trials.csv').read_text().splitlines()), 4)
            self.assertFalse(manager._run_lock.locked())

    def test_missing_cube_records_failure_and_other_cubes_continue(self):
        manager = make_manager()
        for i, key in enumerate(['cube-1', 'cube-2', 'cube-3']):
            manager.objects[key].update_pose(pose_to_transform([-.2, i*.1, .01, 0, 0, 0]), 'base', 1.)
        seed_scene(manager)
        manager.drop_after_transfer = True
        with tempfile.TemporaryDirectory() as directory:
            summary = PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=3, seed=7).run()
            self.assertEqual(summary['completed_trials'], 3)
            self.assertEqual(summary['placement_successes'], 0)
            self.assertEqual(len(manager.transfers), 3)
            self.assertEqual(len(set(manager.transfers)), 3)
            self.assertFalse(manager._run_lock.locked())
            self.assertTrue(manager._state['object_placement_errors'])

    def test_blocked_fixed_target_skips_trial_then_uses_next_target(self):
        manager = make_manager()
        manager.objects['cube-1'].update_pose(pose_to_transform([-.2, 0, .01, 0, 0, 0]), 'base', 1.)
        manager.objects['cube-2'].update_pose(pose_to_transform([.105, .105, .01, 0, 0, 0]), 'base', 1.)
        seed_scene(manager)
        with tempfile.TemporaryDirectory() as directory:
            summary = PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=2,
                                          object_id='cube-1', target_cells=[[3, 3], [4, 2]]).run()
            self.assertEqual(summary['status'], 'completed')
            self.assertEqual(summary['completed_trials'], 2)
            self.assertEqual(summary['placement_successes'], 1)
            self.assertEqual(manager.transfers, ['cube-1'])

    def test_closeup_failure_recovers_and_continues(self):
        manager = make_manager()
        for i, key in enumerate(['cube-1', 'cube-2']):
            manager.objects[key].update_pose(pose_to_transform([-.2, i*.1, .01, 0, 0, 0]), 'base', 1.)
        seed_scene(manager)
        manager.fail_closeup = True
        with tempfile.TemporaryDirectory() as directory:
            summary = PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=2, seed=7).run()
            self.assertEqual(summary['failed_trials'], 1)
            self.assertEqual(summary['placement_successes'], 1)
            self.assertEqual(len(set(manager.scans)), 2)
            self.assertEqual(len(manager.transfers), 1)

    def test_scan_exception_retries_and_out_of_tolerance_does_not_stop(self):
        manager = make_manager()
        manager.objects['cube-1'].update_pose(pose_to_transform([-.2, 0, .01, 0, 0, 0]), 'base', 1.)
        seed_scene(manager)
        manager.scan_errors = 1
        manager.placement_error = .015
        with tempfile.TemporaryDirectory() as directory:
            summary = PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=2, seed=7).run()
            self.assertEqual(summary['completed_trials'], 2)
            self.assertEqual(len(manager.transfers), 2)
            self.assertEqual(summary['failed_trials'], 2)

    def test_motion_failure_continues_after_feedback_recovery(self):
        manager = make_manager()
        for i, key in enumerate(['cube-1', 'cube-2']):
            manager.objects[key].update_pose(pose_to_transform([-.2, i*.1, .01, 0, 0, 0]), 'base', 1.)
        seed_scene(manager)
        manager.fail_transfer = True
        with tempfile.TemporaryDirectory() as directory:
            summary = PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=2, seed=7).run()
            self.assertEqual(summary['failed_trials'], 1)
            self.assertEqual(summary['placement_successes'], 1)


    def test_closeup_pose_is_used_for_transfer(self):
        manager = make_manager()
        manager.objects['cube-1'].update_pose(pose_to_transform([-.2, 0, .01, 0, 0, 0]), 'base', 1.)
        seed_scene(manager)
        manager.closeup_shift = .007
        with tempfile.TemporaryDirectory() as directory:
            PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=1, seed=7).run()
        transfer = next(actions for scan, actions in manager.plans if scan is False)
        pick = next(action for action in transfer if action['name'] == 'pick')
        self.assertAlmostEqual(pick['pose'][0][3], -.193)

    def test_scan_retries_merge_fresh_detections(self):
        from auto_reset.io import EventLog
        manager = make_manager()
        for i, key in enumerate(['cube-1', 'cube-2']):
            manager.objects[key].update_pose(pose_to_transform([-.2, i*.1, .01, 0, 0, 0]), 'base', 1.)
        seed_scene(manager)
        experiment = PlacementExperiment(manager, [0.]*7, [], Path('/tmp/unused'))
        experiment._obstacles = {key: pose.copy() for key, pose in manager.physical.items()}
        manager.visibility_schedule = [{'cube-1'}, {'cube-2'}]
        with tempfile.TemporaryDirectory() as directory:
            events = EventLog(Path(directory)/'scans.jsonl', echo=False)
            try:
                visible = experiment._scan_scene(events, 1)
            finally:
                events.close()
        self.assertEqual(visible, {'cube-1', 'cube-2'})
        self.assertEqual(manager.captures, 2)
        self.assertTrue(all(manager.objects[key].pose is not None for key in visible))

    def test_missing_cube_keeps_source_and_destination_blocked(self):
        manager = make_manager()
        experiment = PlacementExperiment(manager, [0.]*7, [], Path('/tmp/unused'))
        experiment._obstacles['cube-2'] = pose_to_transform([.105, .105, .01, 0, 0, 0])
        experiment._reservations['cube-2'] = pose_to_transform([.189, .105, .01, 0, 0, 0])
        free = experiment._free_cells('cube-1')
        self.assertNotIn((2, 2), free)
        self.assertNotIn((4, 2), free)

    def test_unavailable_robot_feedback_records_failure_before_stopping(self):
        manager = make_manager()
        manager.objects['cube-1'].update_pose(pose_to_transform([-.2, 0, .01, 0, 0, 0]), 'base', 1.)
        seed_scene(manager)
        manager.fail_transfer = True
        def unavailable():
            raise RuntimeError('feedback unavailable')
        manager.robot.require_stationary = unavailable
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, 'Robot recovery failed'):
                PlacementExperiment(manager, [0.]*7, [], Path(directory), runs=2, seed=7).run()
            summary = json.loads((Path(directory)/'summary.json').read_text())
            self.assertEqual(summary['completed_trials'], 1)
            self.assertEqual(summary['status'], 'failed')
            self.assertFalse(manager._run_lock.locked())
            self.assertEqual(len(manager.transfers), 1)


if __name__ == '__main__':
    unittest.main()
