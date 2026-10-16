"""No hardware: original algorithm parity, ROI and Ubuntu serial compatibility."""
import ast
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
from scipy.interpolate import CubicSpline

ROOT = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    module.cv2, module.np, module.CubicSpline = cv2, np, CubicSpline
    module.SRC_MAT_BASE = np.float32(module.SRC_MAT_BASE)
    return module


class PortTests(unittest.TestCase):
    def setUp(self):
        self.old = load('original_0707_test', ROOT/'original/0707float.py')
        self.new = load('ubuntu_0707_test', ROOT/'drive_0707_ubuntu.py')

    def test_algorithm_function_bodies_are_unchanged(self):
        def functions(path):
            return {node.name: ast.dump(node, include_attributes=False)
                    for node in ast.parse(path.read_text(encoding='utf-8')).body
                    if isinstance(node, ast.FunctionDef)}
        old = functions(ROOT/'original/0707float.py')
        new = functions(ROOT/'drive_0707_ubuntu.py')
        adapted = {'main', 'parse_args', 'serial_message', 'src_mat_with_offset'}
        for name in old.keys() - adapted:
            with self.subTest(function=name):
                self.assertEqual(new[name], old[name])

    def test_roi_matches_current_packaged_track(self):
        import yaml
        cfg = ROOT.parent/'src/vehicle_bringup_pkg/config/track_tuning.yaml'
        values = yaml.safe_load(cfg.read_text(encoding='utf-8'))['track_controller_node']['ros__parameters']
        expected = np.float32([[values['bev_src_'+corner+'_x'], values['bev_src_'+corner+'_y']]
                              for corner in ('tl', 'tr', 'br', 'bl')])
        expected[:2, 1] += values['bev_top_shift']
        np.testing.assert_array_equal(self.new.src_mat_with_offset(0), expected)
        self.assertEqual(self.new.CUTTING_IDX, values['roi_cut'])
        self.assertEqual(self.new.STEERING_GAIN, self.old.STEERING_GAIN)
        self.assertEqual(self.new.CAR_CENTER_POINT, self.old.CAR_CENTER_POINT)
        self.assertEqual(self.new.LANE_WIDTH, self.old.LANE_WIDTH)

    def test_synthetic_lane_paths_and_float_steering_match_original_with_same_roi(self):
        self.old.SRC_MAT_BASE = self.new.src_mat_with_offset(0)
        for shift in (-30, 0, 30):
            edge = np.zeros((480, 640), np.uint8)
            cv2.line(edge, (250+shift, 310), (200+shift, 479), 255, 2)
            cv2.line(edge, (390+shift, 310), (460+shift, 479), 255, 2)
            a = self.old.process_lane_edge(edge)
            b = self.new.process_lane_edge(edge)
            np.testing.assert_array_equal(a[1], b[1])
            self.assertEqual([vars(p) for p in a[3]], [vars(p) for p in b[3]])
            old_path, new_path = self.old.plan_path(a[3]), self.new.plan_path(b[3])
            np.testing.assert_allclose(new_path, old_path)
            self.assertEqual(self.new.calculate_steering_detail_from_path(*new_path),
                             self.old.calculate_steering_detail_from_path(*old_path))

    def test_linux_arguments_and_original_float_baud(self):
        with patch.object(sys, 'argv', ['drive']):
            args = self.new.parse_args()
        self.assertEqual(args.camera, '/dev/video2')
        self.assertEqual(args.port, '/dev/arduino')
        self.assertEqual(args.baudrate, 115200)
        self.assertTrue(args.wait_start)
        self.assertEqual(args.launch_burst, 0)
        self.assertFalse(args.enable_lane_change)
        self.assertTrue(Path(args.model).is_file())
        with patch.object(sys, 'argv', ['drive', '--camera', '2', '--serial-format', 'float']):
            args = self.new.parse_args()
        self.assertEqual(args.camera, 2)
        self.assertEqual(args.baudrate, 9600)

    def test_float_protocol_is_original_and_integer_protocol_is_strict(self):
        cmd = self.new.MotionCommand(2.35, 80, 90)
        self.assertEqual(self.new.serial_message(cmd), 's2l80r90\n')
        self.assertEqual(self.new.serial_message(self.new.MotionCommand(4, 0, 0)), 'X\n')
        self.new.SERIAL_FORMAT = 'float'
        self.assertEqual(self.new.serial_message(cmd), self.old.serial_message(cmd))

    def test_fake_serial_opens_stopped_and_sends_stop_before_close(self):
        fake = Mock()
        with patch.object(self.new, 'serial', Mock(Serial=Mock(return_value=fake))), \
             patch.object(self.new.time, 'sleep'):
            connection = self.new.ArduinoSerial('/dev/fake', 115200)
            fake.write.assert_called_with(b'X\n')
            connection.send(self.new.MotionCommand(2.35, 80, 80), force=True)
            fake.write.assert_called_with(b's2l80r80\n')
            connection.stop()
            connection.close()
        fake.write.assert_called_with(b'X\n')
        fake.close.assert_called_once()

    def test_linux_camera_path_and_numeric_index(self):
        for camera in ('/dev/video2', 2):
            fake_cap = Mock()
            with patch.object(sys, 'platform', 'linux'), \
                 patch.object(self.new, 'cv2', Mock(VideoCapture=Mock(return_value=fake_cap))) as fake:
                args = Mock(video=None, camera=camera)
                self.new.open_capture(args)
                fake.VideoCapture.assert_called_once_with(camera)

    def test_sigterm_requests_normal_cleanup(self):
        with self.assertRaises(KeyboardInterrupt):
            self.new.request_stop(15, None)

    def fake_runtime(self):
        cap = Mock()
        cap.read.side_effect = [(True, np.zeros((480, 640, 3), np.uint8)), KeyboardInterrupt()]
        result = Mock(masks=None, boxes=None)
        result.cpu.return_value = result
        model = Mock()
        model.predict.return_value = [result]
        ultralytics = Mock(YOLO=Mock(return_value=model))
        return cap, model, ultralytics

    def test_main_runs_a_frame_and_cleans_up_on_ctrl_c(self):
        cap, model, dependency = self.fake_runtime()
        connection = Mock()
        with patch.dict(sys.modules, {'ultralytics': dependency}), \
             patch.object(sys, 'argv', ['drive', '--no-serial', '--no-wait-start']), \
             patch.object(self.new, 'open_capture', return_value=cap), \
             patch.object(self.new, 'ArduinoSerial', return_value=connection):
            self.new.main()
        model.predict.assert_called_once()
        connection.send.assert_called_once()
        connection.stop.assert_called_once()
        connection.close.assert_called_once()
        cap.release.assert_called_once()

    def test_inference_failure_still_stops_and_closes(self):
        cap, model, dependency = self.fake_runtime()
        model.predict.side_effect = RuntimeError('fake inference failure')
        connection = Mock()
        with patch.dict(sys.modules, {'ultralytics': dependency}), \
             patch.object(sys, 'argv', ['drive', '--no-serial', '--no-wait-start']), \
             patch.object(self.new, 'open_capture', return_value=cap), \
             patch.object(self.new, 'ArduinoSerial', return_value=connection):
            with self.assertRaisesRegex(RuntimeError, 'fake inference'):
                self.new.main()
        connection.stop.assert_called_once()
        connection.close.assert_called_once()
        cap.release.assert_called_once()

    def test_serial_open_failure_releases_camera(self):
        cap, model, dependency = self.fake_runtime()
        with patch.dict(sys.modules, {'ultralytics': dependency}), \
             patch.object(sys, 'argv', ['drive', '--no-wait-start']), \
             patch.object(self.new, 'open_capture', return_value=cap), \
             patch.object(self.new, 'ArduinoSerial', side_effect=OSError('fake serial failure')):
            with self.assertRaisesRegex(OSError, 'fake serial'):
                self.new.main()
        cap.release.assert_called_once()


if __name__ == '__main__':
    unittest.main()
