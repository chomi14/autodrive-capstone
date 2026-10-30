"""Manual capture integration with the real bridge callbacks and fake serial."""

import copy
import csv
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
from interfaces_pkg.msg import MotionCommand
from std_msgs.msg import Bool, String

import test_drive_safety as bridge_tests
from manual_drive_pkg.manual_drive_capture_node import ManualDriveCaptureNode


class ManualCaptureSafetyTests(unittest.TestCase):
    def setUp(self):
        self.bridge_case = bridge_tests.DriveSafetyTests()
        self.bridge_case.setUp()
        self.addCleanup(self.bridge_case.doCleanups)
        self.sender = self.bridge_case.node
        self.manual = object.__new__(ManualDriveCaptureNode)
        node = self.manual
        node.dry_run = False
        node.show_window = False
        node.should_quit = False
        node.calibration_ready = True
        node.armed = False
        node.arm_challenge = self.sender.challenge
        node._last_ui_heartbeat = 0.0
        node.speed_cmd = 0
        node.steering_cmd = 0
        node.speed_step = 20
        node.steering_step = 1
        node.max_speed = 255
        node.max_steering = 7
        node.left_sign = -1
        node.cmd = MotionCommand()
        node.last_key = 'x'
        node.auto_capture = False
        node.auto_interval = 0.2
        node.last_capture_time = 0.0
        node.draw_window = Mock()
        node.capture_frame = Mock(return_value=True)
        node.get_logger = Mock(return_value=Mock())
        node.pub = Mock()
        node.pub.publish.side_effect = lambda msg: self.sender.on_cmd(copy.deepcopy(msg))
        node.arm_pub = Mock()
        node.arm_pub.publish.side_effect = self.sender.on_arm
        node.request_pub = Mock()
        node.request_pub.publish.side_effect = self.sender.on_arm_request
        node.heartbeat_pub = Mock()
        node.heartbeat_pub.publish.side_effect = self.sender.on_ui_heartbeat
        self.sender.challenge_pub.publish.side_effect = node.on_arm_challenge
        self.sender.state_pub.publish.side_effect = node.on_drive_state

    def tick(self, seconds=0.05):
        self.bridge_case.clock += seconds
        self.manual.on_timer()
        self.sender.enforce_safe_state()

    def test_first_w_requests_current_token_and_accepted_motion_continues(self):
        self.manual.handle_key('W')
        self.assertTrue(self.sender.armed)
        self.assertTrue(self.manual.armed)
        self.assertEqual(self.manual.request_pub.publish.call_args.args[0].data, self.sender.challenge)
        self.manual.arm_pub.publish.assert_not_called()  # Bool True cannot authorize motion.
        for _ in range(40):
            self.tick()
            self.assertTrue(self.sender.armed)
            self.assertEqual(self.bridge_case.port.events[-1], b's0l20r20\n')

    def test_motion_waits_for_actual_bridge_acceptance_and_does_not_capture(self):
        self.manual.request_pub.publish.side_effect = None
        self.manual.handle_key('r')
        self.manual.handle_key('w')
        self.tick()
        self.assertFalse(self.manual.armed)
        self.assertFalse(self.sender.armed)
        self.manual.capture_frame.assert_not_called()
        self.manual.on_drive_state(Bool(data=True))
        self.tick(0.2)
        self.manual.capture_frame.assert_called_once_with('auto')

    def test_x_and_space_hold_stop_until_a_new_drive_key(self):
        for key in ('x', ' '):
            self.manual.handle_key('w')
            self.tick()
            self.manual.handle_key(key)
            self.assertFalse(self.sender.armed)
            self.assertEqual(self.manual.speed_cmd, 0)
            for _ in range(4):
                self.tick()
                self.assertEqual(self.bridge_case.port.events[-1], b'X\n')
            self.manual.handle_key('w')
            self.tick()
            self.assertTrue(self.sender.armed)
            self.assertEqual(self.bridge_case.port.events[-1], b's0l20r20\n')
            self.manual.handle_key('x')

    def test_s_keeps_signed_speed_and_reverse_controls(self):
        self.manual.handle_key('w')
        self.manual.handle_key('w')
        self.manual.handle_key('s')
        self.assertEqual(self.manual.speed_cmd, 20)
        self.manual.handle_key('s')
        self.assertEqual(self.manual.speed_cmd, 0)
        self.manual.handle_key('s')
        self.tick()
        self.assertEqual(self.bridge_case.port.events[-1], b's0l-20r-20\n')

    def test_ui_timeout_clears_manual_command_and_requires_a_new_key(self):
        self.manual.handle_key('w')
        token = self.sender.challenge
        self.bridge_case.clock += 0.8
        self.sender.enforce_safe_state()
        self.assertFalse(self.sender.armed)
        self.assertFalse(self.manual.armed)
        self.assertEqual(self.manual.speed_cmd, 0)
        self.assertNotEqual(token, self.manual.arm_challenge)
        self.sender.on_arm_request(String(data=token))
        self.tick()
        self.assertFalse(self.sender.armed)
        self.manual.handle_key('w')
        self.tick()
        self.assertTrue(self.sender.armed)

    def test_not_ready_or_missing_bridge_token_cannot_start(self):
        self.manual.calibration_ready = False
        self.manual.handle_key('w')
        self.manual.calibration_ready = True
        self.manual.arm_challenge = ''
        self.manual.handle_key('s')
        self.assertEqual(self.manual.speed_cmd, 0)
        self.manual.request_pub.publish.assert_not_called()

    def test_window_close_disarms_and_stops_heartbeat(self):
        self.manual.handle_key('w')
        self.manual.show_window = True
        count = self.manual.heartbeat_pub.publish.call_count
        with patch('manual_drive_pkg.manual_drive_capture_node.cv2.getWindowProperty', return_value=0):
            self.manual.on_timer()
        self.assertTrue(self.manual.should_quit)
        self.assertFalse(self.sender.armed)
        self.assertEqual(self.manual.heartbeat_pub.publish.call_count, count)
        self.assertEqual(self.bridge_case.port.events[-1], b'X\n')

    def test_dry_run_does_not_publish_vehicle_authorization_or_heartbeat(self):
        self.manual.dry_run = True
        self.manual.handle_key('w')
        self.manual.publish_ui_heartbeat(force=True)
        self.manual.request_pub.publish.assert_not_called()
        self.manual.heartbeat_pub.publish.assert_not_called()

    def test_r_toggles_auto_capture_only_for_accepted_nonzero_motion(self):
        self.manual.handle_key('r')
        self.tick()
        self.manual.capture_frame.assert_not_called()
        self.manual.handle_key('w')
        self.tick(0.2)
        self.manual.capture_frame.assert_called_once_with('auto')
        self.manual.handle_key('r')
        self.tick(0.2)
        self.assertEqual(self.manual.capture_frame.call_count, 1)

    def test_image_and_command_csv_are_saved_under_selected_directory(self):
        with tempfile.TemporaryDirectory(prefix='manual_capture_test_') as temporary:
            node = self.manual
            node.images_dir = Path(temporary) / 'images'
            node.images_dir.mkdir()
            node.latest_frame = np.zeros((24, 32, 3), dtype=np.uint8)
            node.frame_index = 0
            node.jpeg_quality = 95
            with (Path(temporary) / 'labels.csv').open('w+', newline='') as csv_file:
                node.csv_file = csv_file
                node.csv_writer = csv.writer(csv_file)
                self.assertTrue(ManualDriveCaptureNode.capture_frame(node, 'manual'))
                csv_file.seek(0)
                row = next(csv.reader(csv_file))
                self.assertTrue((node.images_dir / row[1]).is_file())
                self.assertEqual(row[5], 'manual')


if __name__ == '__main__':
    unittest.main()
