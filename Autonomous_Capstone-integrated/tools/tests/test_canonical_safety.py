"""Offline contract tests; do not construct ROS nodes or open hardware."""
import importlib.util
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
from rclpy.parameter import Parameter
from sensor_bringup_pkg.camera_publisher_node import CameraPublisherNode
from vehicle_bringup_pkg.configuration import VehicleDefault
from vehicle_io_pkg.serial_sender_node import SerialSenderNode
from interfaces_pkg.msg import MotionCommand
from skku_track_drive_pkg.messages import DetectionArray, PathPlanningResult
from skku_track_drive_pkg.motion_planner import MotionPlanner
from skku_track_drive_pkg.track_controller_node import TrackControllerNode

ROOT = Path(__file__).resolve().parents[2]


class CanonicalSafetyTests(unittest.TestCase):
    def test_track_tuning_uses_reliable_latest_frame_transport(self):
        path = ROOT / 'src/vehicle_bringup_pkg/launch/track_drive_tuning.launch.py'
        spec = importlib.util.spec_from_file_location('track_tuning_launch_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update({
            'camera_device': '/dev/video-test',
            'lidar_port': '/dev/lidar-test',
            'lidar_rotation': '180',
            'use_lidar': 'false',
            'arduino_port': '/dev/arduino-test',
            'device': 'cpu',
            'steering_sign': '1',
            'calibration_tolerance': '35',
            'enable_visualization': 'false',
            'show_bev': 'false',
            'profile': 'false',
            'debug_log': 'false',
            'arduino_baud': '115200',
            'auto_calibrate': 'false',
        })
        tuning = ({'speed': 80}, Path('/tmp/in'), Path('/tmp/out'), 'test')
        with patch.object(module, 'resolve_tuning', return_value=tuning):
            nodes = module._launch_nodes(context)
        parameters = {
            getattr(node, '_Node__node_name', ''): evaluate_parameters(
                context, getattr(node, '_Node__parameters')
            )[0]
            for node in nodes
            if hasattr(node, '_Node__parameters')
        }
        camera = parameters['camera_publisher_node']
        controller = parameters['track_controller_node']
        self.assertEqual(camera['fourcc'], 'MJPG')
        self.assertEqual(camera['reliability'], 'reliable')
        self.assertEqual(controller['image_reliability'], 'reliable')
        self.assertFalse(controller['publish_debug'])

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

    def test_fixed_track_speed_rejects_runtime_changes(self):
        node = object.__new__(TrackControllerNode)
        node.allow_speed_tuning = False
        result = TrackControllerNode.on_tuning_parameters(
            node, [Parameter('speed', value=80)]
        )
        self.assertFalse(result.successful)
        self.assertEqual(result.reason, 'speed is fixed for this launch')

    def test_perception_failure_holds_steering_and_speed(self):
        node = object.__new__(TrackControllerNode)
        node.enabled = True
        node.speed = 250
        node.motion = SimpleNamespace(last_target_steer=-5)
        node.cmd_pub = Mock()
        TrackControllerNode.publish_perception_fallback(node)
        command = node.cmd_pub.publish.call_args.args[0]
        self.assertEqual(command.steering, -5)
        self.assertEqual(command.left_speed, 250)
        self.assertEqual(command.right_speed, 250)

    def test_camera_requests_configured_transport_and_single_buffer(self):
        capture = Mock()
        capture.isOpened.return_value = True
        capture.get.side_effect = lambda prop: {
            cv2.CAP_PROP_FRAME_WIDTH: 640.0,
            cv2.CAP_PROP_FRAME_HEIGHT: 480.0,
            cv2.CAP_PROP_FPS: 30.0,
            cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'YUYV'),
        }.get(prop, 0.0)
        node = SimpleNamespace(
            cap=None,
            device='/dev/video-test',
            fourcc='YUYV',
            width=640,
            height=480,
            fps=30.0,
            buffer_size=1,
            consecutive_failures=4,
            last_frame_time=123.0,
            fourcc_text=CameraPublisherNode.fourcc_text,
            apply_v4l2_controls=Mock(),
            get_logger=Mock(return_value=Mock()),
        )
        with patch(
            'sensor_bringup_pkg.camera_publisher_node.cv2.VideoCapture',
            return_value=capture,
        ):
            self.assertTrue(CameraPublisherNode.open_camera(node))
        capture.set.assert_any_call(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV')
        )
        capture.set.assert_any_call(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.assertEqual(node.consecutive_failures, 0)
        self.assertIsNone(node.last_frame_time)

    def test_track_controller_accepts_legacy_8uc3_images(self):
        expected = np.zeros((4, 6, 3), dtype=np.uint8)
        node = SimpleNamespace(
            bridge=Mock(),
            _warned_generic_bgr_encoding=False,
            get_logger=Mock(return_value=Mock()),
        )
        node.bridge.imgmsg_to_cv2.return_value = expected
        message = SimpleNamespace(encoding='8UC3')
        converted = TrackControllerNode._image_to_bgr(node, message)
        node.bridge.imgmsg_to_cv2.assert_called_once_with(
            message, desired_encoding='passthrough'
        )
        self.assertEqual(converted.shape, (4, 6, 3))
        self.assertTrue(node._warned_generic_bgr_encoding)


if __name__ == '__main__':
    unittest.main()
