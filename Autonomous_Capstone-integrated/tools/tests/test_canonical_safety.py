"""Offline contract tests; do not construct ROS nodes or open hardware."""
import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from launch import LaunchContext
from vehicle_bringup_pkg.configuration import VehicleDefault
from vehicle_io_pkg.serial_sender_node import SerialSenderNode
from interfaces_pkg.msg import MotionCommand
from skku_track_drive_pkg.messages import DetectionArray, PathPlanningResult
from skku_track_drive_pkg.motion_planner import MotionPlanner

ROOT = Path(__file__).resolve().parents[2]


class CanonicalSafetyTests(unittest.TestCase):
    def test_profile_and_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            share = Path(tmp)
            (share / 'config').mkdir()
            (share / 'config/vehicle.yaml').write_text('arduino:\n  port: /dev/arduino\n')
            profile = share / 'profile.yaml'
            profile.write_text('arduino:\n  port: /dev/test\n')
            context = LaunchContext()
            context.launch_configurations['vehicle_config'] = str(profile)
            with patch('vehicle_bringup_pkg.configuration.get_package_share_directory', return_value=tmp):
                self.assertEqual(VehicleDefault('arduino.port', 'fallback').perform(context), '/dev/test')
                self.assertEqual(VehicleDefault('missing.key', 42).perform(context), '42')
                profile.write_text('[]')
                with self.assertRaises(ValueError):
                    VehicleDefault('arduino.port', '').perform(context)

    def test_all_launch_arguments_without_starting_nodes(self):
        paths = list((ROOT / 'src/vehicle_bringup_pkg/launch').glob('*.py'))
        paths.append(ROOT / 'src/launch_pkg/launch/mission_launch.py')
        from launch.actions import DeclareLaunchArgument
        for path in paths:
            spec = importlib.util.spec_from_file_location('test_launch', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            context = LaunchContext()
            context.launch_configurations['camera_device'] = '/dev/override'
            for action in module.generate_launch_description().entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            self.assertEqual(context.launch_configurations['camera_device'], '/dev/override')
            if 'auto_calibrate' in context.launch_configurations:
                self.assertEqual(context.launch_configurations['auto_calibrate'], 'false')

    def test_gate_and_calibration_heartbeat(self):
        node = object.__new__(SerialSenderNode)
        node.ser = Mock(is_open=True)
        node.calibration_state = 'READY'
        node.calibration_ready = True
        node.armed = False
        command = MotionCommand(steering=3, left_speed=80, right_speed=80)
        node.write_gated_command(command)
        node.ser.write.assert_called_with(b'X\n')
        node.armed = True
        node.write_gated_command(command)
        node.ser.write.assert_called_with(b's3l80r80\n')
        node.calibration_ready = False
        node.calibration_state = 'WAIT_MEASURE'
        node.write_gated_command(command)
        node.ser.write.assert_called_with(b's0l0r0\n')
        node.calibration_state = 'FAILED'
        node.write_gated_command(command)
        node.ser.write.assert_called_with(b'X\n')

    def test_baseline_ready_without_automatic_motion(self):
        node = object.__new__(SerialSenderNode)
        node.calibration_state = 'WAIT_CONFIG'
        node.auto_calibrate = False
        node.min_cal_span = 80
        node.publish_status = Mock()
        node.publish_ready = Mock()
        node.send_ascii = Mock()
        node.fail_calibration = Mock()
        node.handle_serial_line('CONFIG,left=600,right=445,center=522,max_step=7')
        node.publish_ready.assert_called_once_with(True)
        node.send_ascii.assert_not_called()
        node.calibration_state = 'WAIT_CONFIG'
        node.handle_serial_line('CONFIG,left=600,right=445,center=999,max_step=7')
        node.fail_calibration.assert_called_once()

    def test_manual_baseline_mode_does_not_wait_for_config(self):
        node = object.__new__(SerialSenderNode)
        node.auto_calibrate = False
        node.calibration_state = 'STARTING'
        node.calibration_deadline = 123.0
        node.publish_status = Mock()
        node.publish_ready = Mock()
        node.send_ascii = Mock()
        node.start_calibration_sequence()
        self.assertEqual(node.calibration_state, 'READY')
        self.assertEqual(node.calibration_deadline, 0.0)
        node.publish_ready.assert_called_once_with(True)
        node.publish_status.assert_called_once()
        node.send_ascii.assert_not_called()

    def test_auto_calibration_still_requires_config(self):
        node = object.__new__(SerialSenderNode)
        node.auto_calibrate = True
        node.cal_timeout = 22.0
        node.calibration_state = 'STARTING'
        node.publish_status = Mock()
        node.publish_ready = Mock()
        node.send_ascii = Mock()
        with patch('vehicle_io_pkg.serial_sender_node.time.monotonic', return_value=10.0):
            node.start_calibration_sequence()
        self.assertEqual(node.calibration_state, 'WAIT_CONFIG')
        self.assertEqual(node.calibration_deadline, 32.0)
        self.assertEqual(node.last_query_time, 10.0)
        node.publish_ready.assert_called_once_with(False)
        node.send_ascii.assert_called_once_with('?')

    def test_duplicate_config_does_not_restart_calibration(self):
        node = object.__new__(SerialSenderNode)
        node.calibration_state = 'READY'
        node.send_ascii = Mock()
        node.handle_serial_line('CONFIG,left=600,right=445,center=522,max_step=7')
        node.send_ascii.assert_not_called()
        self.assertEqual(node.calibration_state, 'READY')

    def test_missing_path_holds_last_valid_steering_at_configured_speed(self):
        planner = MotionPlanner(
            default_left_speed=255,
            default_right_speed=255,
            stop_on_red=False,
        )
        planner.last_target_steer = -5
        command = planner.plan(DetectionArray(), PathPlanningResult())
        self.assertEqual(command.steering, -5)
        self.assertEqual(command.left_speed, 255)
        self.assertEqual(command.right_speed, 255)


if __name__ == '__main__':
    unittest.main()
