"""Raw capture callbacks and launch topology, without DDS or vehicle hardware."""
import csv
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
import pytest
from cv_bridge import CvBridge
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from manual_drive_pkg.autonomous_capture_node import AutonomousCaptureNode
from skku_track_drive_pkg.track_tuner_node import TrackTunerNode


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def capture(tmp_path, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('manual_drive_pkg.autonomous_capture_node.time.monotonic', lambda: clock[0])
    parameters = {}

    def declare(node, name, value):
        parameters[name] = str(tmp_path) if name == 'output_dir' else value

    with patch('manual_drive_pkg.autonomous_capture_node.Node.__init__'), \
         patch.object(AutonomousCaptureNode, 'declare_parameter', declare), \
         patch.object(AutonomousCaptureNode, 'get_parameter', lambda node, name: SimpleNamespace(value=parameters[name])), \
         patch.object(AutonomousCaptureNode, 'create_publisher', return_value=Mock()) as publishers, \
         patch.object(AutonomousCaptureNode, 'create_subscription', return_value=Mock()) as subscriptions, \
         patch.object(AutonomousCaptureNode, 'create_service', return_value=Mock()), \
         patch.object(AutonomousCaptureNode, 'create_timer', return_value=Mock()), \
         patch.object(AutonomousCaptureNode, 'get_logger', return_value=Mock()):
        node = AutonomousCaptureNode()
    node.get_logger = Mock(return_value=Mock())
    node.test_clock = clock
    node.test_publishers = publishers
    node.test_subscriptions = subscriptions
    yield node
    if node.csv_file is not None:
        node.csv_file.close()


def image(node, value=120):
    msg = CvBridge().cv2_to_imgmsg(np.full((48, 64, 3), value, dtype=np.uint8), encoding='bgr8')
    msg.header.stamp.sec, msg.header.stamp.nanosec = 123, 456
    msg.header.frame_id = 'front_camera'
    node.on_image(msg)


def command(node, armed=True, left=80, right=75):
    node.on_command_status(String(data=json.dumps({
        'armed': armed, 'steering': -3, 'left_pwm': left, 'right_pwm': right,
    })))


def start(node):
    image(node)
    node.on_drive_state(Bool(data=True))
    command(node)
    node.on_key(String(data='R'))


def test_passive_default_off_and_no_files_until_capture(capture):
    node = capture
    image(node)
    node.on_drive_state(Bool(data=True))
    command(node)
    node.on_timer()
    assert not node.enabled and not node.root.exists()
    assert [c.args[1] for c in node.test_publishers.call_args_list] == ['capture/status']
    image_subscription = node.test_subscriptions.call_args_list[0]
    assert image_subscription.args[1] == '/track/image_raw'
    from rclpy.qos import ReliabilityPolicy
    assert image_subscription.args[3].reliability == ReliabilityPolicy.BEST_EFFORT


def test_r_saves_raw_jpeg_with_applied_command_and_source_stamp(capture):
    start(capture)
    original = CvBridge().imgmsg_to_cv2(capture.latest_image, desired_encoding='bgr8').copy()
    with patch('manual_drive_pkg.autonomous_capture_node.cv2.imwrite', wraps=cv2.imwrite) as write:
        capture.on_timer()
        np.testing.assert_array_equal(write.call_args.args[1], original)
    rows = list(csv.DictReader((capture.root / 'labels.csv').open()))
    assert len(rows) == 1 and capture.count == 1
    row = rows[0]
    assert (row['steering'], row['left_speed'], row['right_speed']) == ('-3', '80', '75')
    assert row['capture_mode'] == 'auto'
    assert (row['image_stamp_sec'], row['image_stamp_nanosec'], row['frame_id']) == ('123', '456', 'front_camera')
    saved = cv2.imread(str(capture.images_dir / row['filename']))
    np.testing.assert_array_equal(saved, original)  # Constant field survives JPEG exactly.
    capture.on_key(String(data='r'))
    capture.test_clock[0] += 0.3
    image(capture, 180)
    capture.on_timer()
    assert capture.count == 1 and not capture.enabled


def test_interval_duplicate_and_camera_freshness(capture):
    start(capture)
    capture.on_timer()
    capture.test_clock[0] += 0.1
    image(capture)
    capture.on_timer()
    assert capture.count == 1
    capture.test_clock[0] += 0.11
    capture.on_timer()
    assert capture.count == 2
    capture.test_clock[0] += 0.21
    capture.on_timer()
    assert capture.count == 2  # The latest raw frame is never saved twice automatically.
    image(capture)
    capture.test_clock[0] += 0.8
    command(capture)
    capture.on_timer()
    assert capture.count == 2  # Old camera image, even with fresh motion feedback.


@pytest.mark.parametrize('state', ['locked', 'disarmed', 'stopped', 'status_stale'])
def test_auto_requires_fresh_accepted_nonzero_motion(capture, state):
    start(capture)
    if state == 'locked':
        capture.armed = False  # Requested commands alone never authorize capture.
    elif state == 'disarmed':
        capture.on_drive_state(Bool(data=False))
    elif state == 'stopped':
        command(capture, left=0, right=0)
    else:
        capture.test_clock[0] += 0.8
        image(capture)
    capture.on_timer()
    assert capture.count == 0 and not capture.root.exists()


def test_pause_and_resume_when_vehicle_stops_and_starts(capture):
    start(capture)
    capture.on_timer()
    capture.test_clock[0] += 0.3
    image(capture)
    capture.on_drive_state(Bool(data=False))
    capture.on_timer()
    assert capture.count == 1 and capture.enabled
    capture.on_drive_state(Bool(data=True))
    capture.on_timer()
    assert capture.count == 1  # Re-arm still needs fresh accepted command feedback.
    command(capture)
    capture.on_timer()
    assert capture.count == 2


def test_service_switch_and_single_capture_while_stopped(capture):
    response = capture.set_enabled(SetBool.Request(data=True), SetBool.Response())
    assert response.success and capture.enabled
    capture.set_enabled(SetBool.Request(data=False), SetBool.Response())
    image(capture)
    capture.on_key(String(data='C'))
    assert not capture.enabled and capture.count == 1
    row = next(csv.DictReader((capture.root / 'labels.csv').open()))
    assert row['capture_mode'] == 'manual' and row['left_speed'] == '0'


def test_failed_jpeg_disables_auto_without_a_label_row(capture):
    start(capture)
    with patch('manual_drive_pkg.autonomous_capture_node.cv2.imwrite', return_value=False):
        capture.on_timer()
    assert not capture.enabled and capture.error and capture.count == 0
    assert list(csv.DictReader((capture.root / 'labels.csv').open())) == []
    assert json.loads(capture.status_pub.publish.call_args.args[0].data)['error']


@pytest.mark.parametrize('keys', ['rRcC', 'WwSs'])
def test_tuner_key_dispatch_preserves_drive_controls(keys):
    node = object.__new__(TrackTunerNode)
    node._window_closed = False
    node._arm_challenge = ''
    node._get_future = Mock()
    node._ui_ready = True
    node._send_pending_parameters = node._draw_status = node._draw_debug = Mock()
    node.forward_drive_key = Mock()
    node.forward_capture_key = Mock()
    with patch('skku_track_drive_pkg.track_tuner_node.cv2.getWindowProperty', return_value=1), \
         patch('skku_track_drive_pkg.track_tuner_node.cv2.waitKey', side_effect=list(map(ord, keys))):
        for _ in keys:
            node.on_timer()
    target = node.forward_capture_key if keys == 'rRcC' else node.forward_drive_key
    assert [c.args[0] for c in target.call_args_list] == list(keys.lower())


def test_capture_keys_are_not_sent_to_a_missing_or_dead_capture_node():
    node = object.__new__(TrackTunerNode)
    node.capture_status_time = float('-inf')
    node.capture_key_pub = Mock()
    node.get_logger = Mock(return_value=Mock())
    node.forward_capture_key('r')
    node.capture_key_pub.publish.assert_not_called()
    node.on_capture_status(String(data='{"enabled": false}'))
    node.forward_capture_key('r')
    assert node.capture_key_pub.publish.call_args.args[0].data == 'r'


@pytest.mark.parametrize('filename,topic', [
    ('track_drive.launch.py', '/image_raw'),
    ('track_drive_tuning.launch.py', '/track/image_raw'),
    ('mission_drive_tuning.launch.py', '/mission/image_raw'),
    ('mission_bbox_only.launch.py', '/mission/image_raw'),
    ('mission_lidar_camera.launch.py', '/mission/image_raw'),
])
def test_camera_launch_capture_uses_original_input_and_one_controller(filename, topic):
    spec = importlib.util.spec_from_file_location('capture_launch_test', ROOT / 'src/vehicle_bringup_pkg/launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(capture='true', capture_interval='0.4', capture_start_enabled='true')
    actions = []
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            actions.extend(action.execute(context))
        else:
            actions.append(action)
    nodes = [n for n in actions if isinstance(n, Node) and (n.condition is None or n.condition.evaluate(context))]
    assert sum('controller_node' in n.node_executable for n in nodes) == 1
    captures = [n for n in nodes if n.node_executable == 'autonomous_capture_node']
    assert len(captures) == 1
    settings = evaluate_parameters(context, captures[0]._Node__parameters)[0]
    assert settings['image_topic'] == topic
    assert settings['auto_capture_interval'] == 0.4 and settings['start_enabled'] is True
