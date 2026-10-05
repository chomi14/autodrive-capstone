"""Motor safety regression tests using fake serial and publishers only.

These tests invoke the production callbacks and cleanup code. They never open a
serial device, construct a DDS participant, display a window, or move hardware.
"""
import json
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from std_msgs.msg import Bool, String
from interfaces_pkg.msg import MotionCommand
from vehicle_io_pkg.drive_arm_node import DriveArmNode
from vehicle_io_pkg.serial_sender_node import SerialSenderNode
from skku_track_drive_pkg.track_tuner_node import TrackTunerNode


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.in_waiting = 0
        self.events = []
        self.fail = False

    def write(self, payload):
        if self.fail:
            raise OSError('USB disconnected')
        self.events.append(payload)
        return len(payload)

    def close(self):
        self.events.append('close')
        self.is_open = False


class DriveSafetyTests(unittest.TestCase):
    def setUp(self):
        self.clock = 10.0
        self.time_patch = patch('vehicle_io_pkg.serial_sender_node.time.monotonic', side_effect=lambda: self.clock)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)
        self.context_patch = patch('vehicle_io_pkg.serial_sender_node.rclpy.ok', return_value=True)
        self.context_patch.start()
        self.addCleanup(self.context_patch.stop)
        self.node = self.construct_sender()
        self.command = MotionCommand(steering=3, left_speed=80, right_speed=80)

    def construct_sender(self, fail_on_open=False, command_timeout=None):
        # Execute the real constructor while replacing Node infrastructure.
        # This verifies defaults, the initial X frame, QoS, and subscriptions.
        parameters = {}
        def declare(node, name, value, **_kwargs):
            parameters[name] = command_timeout if name == 'command_timeout' and command_timeout is not None else value
        def get(node, name):
            return SimpleNamespace(value=parameters[name])
        self.port = FakeSerial()
        self.port.fail = fail_on_open
        self.logger = Mock()
        with patch('vehicle_io_pkg.serial_sender_node.Node.__init__', lambda n, *a: setattr(n, '_context', Mock())), \
             patch.object(SerialSenderNode, 'declare_parameter', declare), \
             patch.object(SerialSenderNode, 'get_parameter', get), \
             patch.object(SerialSenderNode, 'get_logger', return_value=self.logger), \
             patch.object(SerialSenderNode, 'create_publisher', side_effect=lambda *a: Mock()), \
             patch.object(SerialSenderNode, 'create_subscription', return_value=Mock()), \
             patch.object(SerialSenderNode, 'create_timer', return_value=Mock()), \
             patch('vehicle_io_pkg.serial_sender_node.serial.Serial', return_value=self.port), \
             patch('vehicle_io_pkg.serial_sender_node.time.sleep'):
            node = SerialSenderNode()
        # Keep logger available after the constructor's patches expire.
        node.get_logger = Mock(return_value=self.logger)
        return node

    def feed_ui(self):
        self.node.on_ui_heartbeat(String(data=self.node.challenge))
        self.node.on_tuner_heartbeat(String(data=self.node.challenge))

    def feed_controller(self, active=True, instance='controller-a', token=None):
        self.node.on_controller_heartbeat(String(data=json.dumps({
            'challenge': token or self.node.challenge, 'instance': instance, 'active': active})))

    def canonical_sender(self, timeout=0.0):
        self.node = self.construct_sender(command_timeout=timeout)
        self.node.require_controller_heartbeat = True
        self.node.require_tuner_heartbeat = True
        self.feed_controller()
        self.arm()
        self.node.enforce_safe_state()

    def test_independent_health_allows_long_inference_delay_at_20hz(self):
        for timeout in (0.0, .75, .25, .5):
            self.canonical_sender(timeout)
            received = self.node.last_command_time
            token = self.node.challenge
            for _ in range(200):  # 10 seconds without another inference
                self.clock += .05
                self.feed_controller()
                self.feed_ui()
                count = len(self.port.events)
                self.node.enforce_safe_state()
                self.assertEqual(self.port.events[count:], [b's3l80r80\n'])
                self.assertTrue(self.node.armed)
            self.assertEqual(self.node.last_command_time, received)
            self.assertEqual(self.node.challenge, token)

    def test_canonical_nonzero_commands_wait_for_serial_timer(self):
        self.canonical_sender()
        count = len(self.port.events)
        self.node.on_cmd(MotionCommand(steering=-2, left_speed=37, right_speed=37))
        self.assertEqual(len(self.port.events), count)
        self.node.enforce_safe_state()
        self.assertEqual(self.port.events[-1], b's-2l37r37\n')

    def test_controller_loss_uses_existing_timeout_and_requires_fresh_w(self):
        for timeout in (0.0, .75, .25, .5):
            self.canonical_sender(timeout)
            deadline = self.clock + self.node.controller_timeout
            token = self.node.challenge
            while self.clock + .05 < deadline:
                self.clock += .05
                self.feed_ui()
                self.node.enforce_safe_state()
                self.assertTrue(self.node.armed)
            self.clock = deadline + .001
            self.feed_ui()
            self.node.enforce_safe_state()
            self.assertFalse(self.node.armed)
            self.assertEqual(self.port.events[-1], b'X\n')
            self.feed_controller(token=token)  # queued pre-expiry heartbeat
            self.assertIsNone(self.node.last_controller_time)
            self.feed_controller()
            self.feed_ui()
            self.node.on_cmd(self.command)
            self.node.on_arm_request(String(data=token))
            self.node.enforce_safe_state()
            self.assertFalse(self.node.armed)
            self.node.on_arm_request(String(data=self.node.challenge))
            self.node.enforce_safe_state()
            self.assertTrue(self.node.armed)

    def test_intentional_stop_is_not_replaced_by_delay_replay(self):
        self.canonical_sender()
        self.node.on_cmd(MotionCommand(steering=3, left_speed=0, right_speed=0))
        self.assertEqual(self.port.events[-1], b's3l0r0\n')
        for _ in range(40):
            self.clock += .05
            self.feed_ui()
            self.feed_controller()
            self.node.enforce_safe_state()
            self.assertEqual(self.port.events[-1], b's3l0r0\n')
            self.assertTrue(self.node.armed)  # mission stop is not operator S

    def test_s_during_inference_delay_is_immediate_and_recovery_needs_w(self):
        self.canonical_sender()
        token = self.node.challenge
        self.node.on_arm(Bool(data=False))
        self.assertEqual(self.port.events[-1], b'X\n')
        self.feed_controller(token=token)
        self.feed_ui()
        self.node.on_arm_request(String(data=self.node.challenge))
        self.assertFalse(self.node.armed)  # no post-S command/health
        self.feed_controller()
        self.node.on_cmd(self.command)
        self.node.enforce_safe_state()
        self.assertFalse(self.node.armed)
        self.node.on_arm_request(String(data=token))
        self.assertFalse(self.node.armed)
        self.node.on_arm_request(String(data=self.node.challenge))
        self.node.enforce_safe_state()
        self.assertTrue(self.node.armed)

    def test_graceful_controller_shutdown_and_restart_invalidate_w(self):
        self.canonical_sender()
        self.feed_controller(active=False)
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')
        self.feed_controller()
        self.feed_ui()
        self.node.on_cmd(self.command)
        self.node.on_arm_request(String(data=self.node.challenge))
        self.node.enforce_safe_state()
        self.assertTrue(self.node.armed)
        self.feed_controller(instance='controller-b')
        self.assertFalse(self.node.armed)
        self.assertIsNone(self.node.last_command_time)
        self.assertEqual(self.port.events[-1], b'X\n')

    def test_canonical_bridge_gap_does_not_rearm_on_new_health(self):
        self.canonical_sender()
        token = self.node.challenge
        self.clock += .51  # bridge, not just YOLO, was suspended
        self.feed_controller(token=token)
        self.assertFalse(self.node.armed)
        self.assertIsNone(self.node.last_controller_time)
        self.assertEqual(self.port.events[-1], b'X\n')

    def arm(self):
        self.feed_ui()
        self.node.on_cmd(self.command)
        self.node.on_arm_request(String(data=self.node.challenge))
        self.assertTrue(self.node.armed)
        self.node.on_cmd(self.command)  # motion needs a fresh post-W arrival

    def gate(self):
        gate = object.__new__(DriveArmNode)
        gate.ready = True
        gate.armed = False
        gate.challenge = self.node.challenge
        gate.get_logger = Mock(return_value=Mock())
        gate.heartbeat_pub = Mock()
        gate.heartbeat_pub.publish.side_effect = self.node.on_ui_heartbeat
        gate.arm_pub = Mock()
        gate.arm_pub.publish.side_effect = self.node.on_arm
        gate.request_pub = Mock()
        gate.request_pub.publish.side_effect = self.node.on_arm_request
        self.node.challenge_pub.publish.side_effect = gate.on_challenge
        self.node.state_pub.publish.side_effect = gate.on_drive_state
        return gate

    def test_initial_disarm_and_legacy_true_cannot_start(self):
        self.assertFalse(self.node.armed)
        self.assertTrue(self.node.calibration_ready)
        self.assertEqual(self.port.events[0], b'X\n')
        self.feed_ui()
        self.node.on_arm(Bool(data=True))
        self.node.on_cmd(self.command)
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')

    def test_tuned_pwm_is_encoded_without_bridge_override(self):
        self.feed_ui()
        command = MotionCommand(steering=-2, left_speed=37, right_speed=37)
        self.node.on_cmd(command)
        self.node.on_arm_request(String(data=self.node.challenge))
        self.node.on_cmd(command)
        self.assertEqual(self.port.events[-1], b's-2l37r37\n')

    def test_initial_stop_failure_cannot_publish_ready(self):
        node = self.construct_sender(fail_on_open=True)
        self.assertFalse(node.armed)
        self.assertFalse(node.calibration_ready)
        self.assertTrue(node.serial_fault)
        self.assertEqual(node.calibration_state, 'FAILED')

    def test_disabled_timeout_never_replays_cached_motion_when_controller_stops(self):
        self.node = self.construct_sender(command_timeout=0.0)
        self.node.require_tuner_heartbeat = True
        self.arm()
        token = self.node.challenge
        received_at = self.node.last_command_time
        # Both UIs remain alive. No cached frame may feed the board watchdog.
        for _ in range(9):
            self.clock += 0.05
            self.feed_ui()
            before = len(self.port.events)
            self.node.enforce_safe_state()
            self.assertTrue(self.node.armed)
            self.assertEqual(self.port.events[before:], [])
        self.clock += .06
        self.feed_ui()
        self.node.enforce_safe_state()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')
        self.assertNotEqual(self.node.challenge, token)
        self.assertIsNone(self.node.last_command_time)

    def test_disabled_timeout_s_stops_repeats_and_requires_new_command_and_w(self):
        self.node = self.construct_sender(command_timeout=0.0)
        self.arm()
        stale = self.node.challenge
        self.node.on_arm(Bool(data=False))
        self.assertIsNone(self.node.last_command_time)
        for _ in range(20):
            self.clock += 0.1
            self.feed_ui()
            self.node.enforce_safe_state()
            self.assertEqual(self.port.events[-1], b'X\n')
        self.node.on_arm_request(String(data=stale))
        self.node.on_arm_request(String(data=self.node.challenge))
        self.assertFalse(self.node.armed)  # no command received since S
        self.node.on_cmd(self.command)
        self.node.enforce_safe_state()
        self.assertFalse(self.node.armed)  # command recovery is not a new W
        self.node.on_arm_request(String(data=self.node.challenge))
        self.node.on_cmd(self.command)
        self.node.enforce_safe_state()
        self.assertEqual(self.port.events[-1], b's3l80r80\n')

    def test_disabled_timeout_still_requires_first_command_before_w(self):
        self.node = self.construct_sender(command_timeout=0.0)
        self.feed_ui()
        self.node.on_arm_request(String(data=self.node.challenge))
        self.node.enforce_safe_state()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')

    def test_disabled_timeout_ui_loss_stops_repeats(self):
        self.node = self.construct_sender(command_timeout=0.0)
        self.node.require_tuner_heartbeat = True
        for missing in ('gate', 'tuner'):
            self.arm()
            stale = self.node.challenge
            for _ in range(4):
                self.clock += 0.2
                callback = self.node.on_tuner_heartbeat if missing == 'gate' else self.node.on_ui_heartbeat
                callback(String(data=self.node.challenge))
                self.node.enforce_safe_state()
            self.assertFalse(self.node.armed)
            self.assertEqual(self.port.events[-1], b'X\n')
            self.feed_ui()
            self.node.on_cmd(self.command)
            self.node.on_arm_request(String(data=stale))
            self.node.enforce_safe_state()
            self.assertFalse(self.node.armed)
            self.assertEqual(self.port.events[-1], b'X\n')

    def test_disabled_timeout_fresh_command_write_failure_disarms(self):
        self.node = self.construct_sender(command_timeout=0.0)
        self.arm()
        self.port.fail = True
        self.node.on_cmd(self.command)
        self.node.enforce_safe_state()
        self.assertTrue(self.node.serial_fault)
        self.assertFalse(self.node.armed)
        self.port.fail = False
        self.feed_ui()
        self.node.on_cmd(self.command)
        self.node.on_arm_request(String(data=self.node.challenge))
        self.node.on_cmd(self.command)
        self.node.enforce_safe_state()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')

    def test_disabled_timeout_invalid_input_does_not_replace_last_valid_command(self):
        self.node = self.construct_sender(command_timeout=0.0)
        self.arm()
        self.node.on_cmd(MotionCommand(steering=8, left_speed=255, right_speed=255))
        self.node.enforce_safe_state()
        self.assertTrue(self.node.armed)
        self.assertEqual(self.port.events[-1], b's3l80r80\n')

    def test_disabled_timeout_shutdown_stops_before_close(self):
        self.node = self.construct_sender(command_timeout=0.0)
        self.arm()
        self.node.enforce_safe_state()
        with patch('vehicle_io_pkg.serial_sender_node.rclpy.ok', return_value=False):
            self.node.close_serial()
        self.assertEqual(self.port.events[-2:], [b'X\n', 'close'])

    def test_disabled_timeout_bridge_suspension_cannot_resume_after_board_watchdog(self):
        self.node = self.construct_sender(command_timeout=0.0)
        self.arm()
        self.node.enforce_safe_state()
        stale = self.node.challenge
        # No bridge timer runs for >500 ms. Even if UI heartbeat delivery
        # resumes first (and is still below 750 ms), do not replay the old drive
        # frame to a board which has already stopped via its own watchdog.
        self.clock += 0.51
        self.node.on_ui_heartbeat(String(data=stale))
        self.node.enforce_safe_state()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')
        self.feed_ui()
        self.node.on_cmd(self.command)
        self.node.enforce_safe_state()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')
        self.node.on_arm_request(String(data=self.node.challenge))
        self.node.on_cmd(self.command)
        self.node.enforce_safe_state()
        self.assertEqual(self.port.events[-1], b's3l80r80\n')

    def test_w_drive_s_continuing_commands_then_new_w(self):
        gate = self.gate()
        self.node.on_cmd(self.command)
        gate.handle_key('W')
        self.node.on_cmd(self.command)
        self.assertEqual(self.port.events[-1], b's3l80r80\n')
        stale_w = self.node.challenge
        gate.handle_key('S')
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')
        for _ in range(10):
            self.clock += 0.1
            self.feed_ui()
            self.node.on_cmd(self.command)
            self.node.enforce_safe_state()
            self.assertEqual(self.port.events[-1], b'X\n')
        self.node.on_arm(Bool(data=True))
        self.node.on_arm_request(String(data=stale_w))
        gate.on_forwarded_key(String(data=f'w:{stale_w}'))
        self.assertFalse(self.node.armed)
        gate.handle_key('w')
        self.node.on_cmd(self.command)
        self.assertEqual(self.port.events[-1], b's3l80r80\n')

    def test_command_timeout_recovery_requires_new_w(self):
        self.arm()
        stale = self.node.challenge
        self.clock += 0.5
        self.node.enforce_safe_state()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')
        self.feed_ui()
        self.node.on_cmd(self.command)
        self.node.on_arm_request(String(data=stale))
        self.assertFalse(self.node.armed)
        self.arm()

    def test_repeated_subthreshold_delays_keep_motion_authorized(self):
        self.node.require_tuner_heartbeat = True
        self.arm()
        token = self.node.challenge
        # Almost half a second between commands is much slower than nominal
        # operation. Keep both UIs alive while the controller is delayed, and
        # exercise the actual timer repeatedly between arrivals.
        for _ in range(20):
            for delta in (0.1, 0.1, 0.1, 0.1, 0.09):
                self.clock += delta
                self.feed_ui()
                self.node.enforce_safe_state()
                self.assertTrue(self.node.armed)
            self.node.on_cmd(self.command)
            self.assertEqual(self.port.events[-1], b's3l80r80\n')
        self.assertEqual(self.node.challenge, token)

    def test_arrival_after_timeout_cannot_hide_expiry(self):
        self.arm()
        self.clock += 0.51
        self.node.on_cmd(self.command)  # timer did not get a turn before this arrival
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')

    def test_gate_and_tuner_heartbeat_timeout(self):
        self.node.require_tuner_heartbeat = True
        for missing in ('gate', 'tuner'):
            self.arm()
            stale = self.node.challenge
            for _ in range(4):
                self.clock += 0.2
                callback = self.node.on_tuner_heartbeat if missing == 'gate' else self.node.on_ui_heartbeat
                callback(String(data=self.node.challenge))
                self.node.on_cmd(self.command)
            self.assertFalse(self.node.armed)
            self.assertEqual(self.port.events[-1], b'X\n')
            self.node.on_ui_heartbeat(String(data=stale))
            self.node.on_arm_request(String(data=stale))
            self.assertFalse(self.node.armed)

    def test_late_heartbeat_does_not_revive_arm(self):
        self.arm()
        self.clock += 0.76
        self.node.last_command_time = self.clock
        self.node.on_ui_heartbeat(String(data=self.node.challenge))
        self.assertFalse(self.node.armed)

    def test_invalid_commands_do_not_renew_watchdog(self):
        self.arm()
        last = self.node.last_command_time
        self.clock += 0.3
        self.node.on_cmd(MotionCommand(steering=8, left_speed=80, right_speed=80))
        self.assertEqual(self.node.last_command_time, last)
        self.clock += 0.21
        self.node.enforce_safe_state()
        self.assertFalse(self.node.armed)

    def test_not_calibrated_cannot_arm(self):
        self.node.calibration_ready = False
        self.feed_ui()
        self.node.on_cmd(self.command)
        self.node.on_arm_request(String(data=self.node.challenge))
        self.assertFalse(self.node.armed)

    def test_write_failure_locks_even_after_connection_recovers(self):
        self.arm()
        stale = self.node.challenge
        self.port.fail = True
        self.node.on_cmd(self.command)
        self.assertTrue(self.node.serial_fault)
        self.assertFalse(self.node.armed)
        self.port.fail = False
        self.feed_ui()
        self.node.on_cmd(self.command)
        self.node.on_arm_request(String(data=stale))
        self.node.on_arm_request(String(data=self.node.challenge))
        self.assertFalse(self.node.armed)

    def test_short_serial_write_disarms(self):
        self.arm()
        with patch.object(self.port, 'write', return_value=1):
            self.node.on_cmd(self.command)
        self.assertTrue(self.node.serial_fault)
        self.assertFalse(self.node.armed)

    def test_ui_close_requests_disarm_and_ceases_heartbeat(self):
        gate = self.gate()
        self.node.on_cmd(self.command)
        gate.handle_key('w')
        tuner = object.__new__(TrackTunerNode)
        tuner._arm_challenge = self.node.challenge
        tuner.get_logger = Mock(return_value=Mock())
        tuner.drive_key_pub = Mock()
        tuner.drive_key_pub.publish.side_effect = gate.on_forwarded_key
        tuner._debug_window_open = False
        tuner._ui_ready = True
        tuner._window_closed = False
        with patch('skku_track_drive_pkg.track_tuner_node.cv2.destroyWindow'):
            tuner._close_gui_windows()
        self.assertTrue(tuner._window_closed)
        self.assertFalse(self.node.armed)
        self.assertEqual(self.port.events[-1], b'X\n')

    def test_tuner_waitkey_dispatches_both_case_w_s(self):
        tuner = object.__new__(TrackTunerNode)
        tuner._window_closed = False
        tuner._arm_challenge = ''
        tuner._get_future = Mock()
        tuner._ui_ready = True
        tuner._send_pending_parameters = Mock()
        tuner._draw_status = Mock()
        tuner._draw_debug = Mock()
        tuner.forward_drive_key = Mock()
        with patch('skku_track_drive_pkg.track_tuner_node.cv2.getWindowProperty', return_value=1), \
             patch('skku_track_drive_pkg.track_tuner_node.cv2.waitKey', side_effect=list(map(ord, 'WwSs'))):
            for _ in range(4):
                tuner.on_timer()
        self.assertEqual([c.args[0] for c in tuner.forward_drive_key.call_args_list], ['w', 'w', 's', 's'])

    def test_cleanup_after_ros_shutdown_stops_before_close(self):
        self.arm()
        with patch('vehicle_io_pkg.serial_sender_node.rclpy.ok', return_value=False):
            self.node.close_serial()
            self.node.close_serial()
        self.assertEqual(self.port.events[-2:], [b'X\n', 'close'])
        self.assertEqual(self.port.events.count('close'), 1)

    def test_s_aborts_calibration_with_x_not_center_command(self):
        self.node.calibration_state = 'WAIT_MEASURE'
        self.node.calibration_ready = False
        self.node.send_stop()
        self.assertEqual(self.port.events[-1], b's0l0r0\n')
        self.node.on_arm(Bool(data=False))
        self.assertEqual(self.port.events[-1], b'X\n')
        self.assertEqual(self.node.calibration_state, 'FAILED')

    def test_tuner_key_forwarding_and_stale_key_filter(self):
        gate = self.gate()
        tuner = object.__new__(TrackTunerNode)
        tuner._arm_challenge = self.node.challenge
        tuner.get_logger = Mock(return_value=Mock())
        tuner.drive_key_pub = Mock()
        tuner.drive_key_pub.publish.side_effect = gate.on_forwarded_key
        self.feed_ui()
        self.node.on_cmd(self.command)
        tuner.forward_drive_key('w')
        self.assertTrue(self.node.armed)
        tuner.forward_drive_key('s')
        self.assertFalse(self.node.armed)
        tuner.forward_drive_key('w')  # challenge was not updated at this tuner yet
        self.assertFalse(self.node.armed)

    def test_main_exception_stops_port_and_handles_partial_init(self):
        from vehicle_io_pkg import serial_sender_node as module
        port = FakeSerial()
        def failing_init(node):
            node.ser = port
            raise RuntimeError('failure after port opened')
        with patch.object(module.rclpy, 'init'), patch.object(module.rclpy, 'shutdown'), \
             patch.object(module.SerialSenderNode, '__init__', failing_init):
            module.main()
        self.assertEqual(port.events, [b'X\n', 'close'])

    def test_real_sigint_and_sigterm_cleanup_with_fake_port(self):
        # Execute main() in an isolated child; deliver actual OS signals. Serial
        # is file-backed so we can prove write precedes close after child exit.
        child = '''
import sys, time
from pathlib import Path
from unittest.mock import patch
from vehicle_io_pkg import serial_sender_node as m
p = Path(sys.argv[1])
class Port:
    is_open = True
    def write(self, payload):
        with p.open('a') as f: f.write(repr(payload) + '\\n')
        return len(payload)
    def close(self):
        with p.open('a') as f: f.write('close\\n')
def init(node): node.ser = Port()
def spin(node):
    with p.open('a') as f: f.write('ready\\n')
    while True: time.sleep(0.02)
with patch.object(m.rclpy, 'init'), patch.object(m.rclpy, 'ok', return_value=False), patch.object(m.rclpy, 'spin', spin), patch.object(m.SerialSenderNode, '__init__', init):
    m.main()
'''
        for sig in (signal.SIGINT, signal.SIGTERM):
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / 'events'
                proc = subprocess.Popen([sys.executable, '-c', child, str(path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                try:
                    deadline = time.perf_counter() + 5
                    while not path.exists() and time.perf_counter() < deadline:
                        time.sleep(0.02)
                    self.assertTrue(path.exists(), 'signal-test child did not start')
                    proc.send_signal(sig)
                    out, err = proc.communicate(timeout=5)
                    self.assertEqual(proc.returncode, 0, (out, err))
                    self.assertEqual(path.read_text().splitlines(), ['ready', "b'X\\n'", 'close'])
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.wait()


if __name__ == '__main__':
    unittest.main()
