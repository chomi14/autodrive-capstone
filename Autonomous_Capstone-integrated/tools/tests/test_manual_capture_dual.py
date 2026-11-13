"""Paired manual capture and launch wiring; no devices, windows, or DDS."""
import csv
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
import pytest
from cv_bridge import CvBridge
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[2]
for package in ('manual_drive_pkg', 'vehicle_bringup_pkg'):
    sys.path.insert(0, str(ROOT / 'src' / package))

from manual_drive_pkg.manual_drive_capture_node import ManualDriveCaptureNode
from vehicle_bringup_pkg.development import manual_capture as launch


@pytest.fixture
def capture(tmp_path, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('manual_drive_pkg.manual_drive_capture_node.time.monotonic', lambda: clock[0])
    parameters = {}
    overrides = dict(output_dir=str(tmp_path), use_rear_camera=True, auto_capture=True,
                     show_window=False, dry_run=True)

    def declare(node, name, value):
        parameters[name] = overrides.get(name, value)

    with patch('manual_drive_pkg.manual_drive_capture_node.Node.__init__'), \
         patch.object(ManualDriveCaptureNode, 'declare_parameter', declare), \
         patch.object(ManualDriveCaptureNode, 'get_parameter', lambda n, key: SimpleNamespace(value=parameters[key])), \
         patch.object(ManualDriveCaptureNode, 'create_subscription', return_value=Mock()) as subs, \
         patch.object(ManualDriveCaptureNode, 'create_publisher', return_value=Mock()), \
         patch.object(ManualDriveCaptureNode, 'create_timer', return_value=Mock()):
        node = ManualDriveCaptureNode()
    node.get_logger = Mock(return_value=Mock())
    node.test_clock = clock
    node.test_subs = subs
    yield node
    node.csv_file.close()


def image(node, role, stamp=123.0, value=80):
    msg = CvBridge().cv2_to_imgmsg(np.full((24, 32, 3), value, dtype=np.uint8), encoding='bgr8')
    stamp_ns = round(stamp * 1_000_000_000)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    (node.on_image if role == 'front' else node.on_rear_image)(msg)


def pair(node):
    image(node, 'front', value=80)
    image(node, 'rear', stamp=123.04, value=160)


def rows(node):
    with node.csv_path.open() as stream:
        return list(csv.DictReader(stream))


def test_pair_saves_two_raw_images_and_matching_command_record(capture):
    node = capture
    pair(node)
    node.handle_key('s')
    node.handle_key('a')
    node.on_timer()  # Starts enabled, captures during reverse as well as forward.
    row, = rows(node)
    assert (row['steering'], row['left_speed'], row['right_speed']) == ('-1', '-20', '-20')
    assert row['capture_mode'] == 'auto'
    assert float(row['pair_delta_ms']) == 40.0
    assert int(row['rear_stamp_ns']) - int(row['front_stamp_ns']) == 40_000_000
    for role, value in [('front', 80), ('rear', 160)]:
        path = node.images_dir / row[role + '_filename']
        assert path.parent.name == role
        assert path.name.endswith('_' + role + '.jpg')
        np.testing.assert_array_equal(cv2.imread(str(path)), np.full((24, 32, 3), value, dtype=np.uint8))
    assert len({Path(row[role + '_filename']).stem.rsplit('_', 1)[0] for role in ('front', 'rear')}) == 1
    assert {'image_raw', 'manual/rear/image_raw'} <= {c.args[1] for c in node.test_subs.call_args_list}


@pytest.mark.parametrize('case', ['missing', 'stale', 'skew'])
def test_pair_rejects_incomplete_old_or_mismatched_images(capture, case):
    image(capture, 'front')
    if case != 'missing':
        image(capture, 'rear', stamp=124.0 if case == 'skew' else 123.0)
    if case == 'stale':
        capture.test_clock[0] += 0.6
        image(capture, 'front')
    assert not capture.capture_frame('manual')
    assert rows(capture) == []
    assert list(capture.images_dir.rglob('*.jpg')) == []


def test_both_images_must_advance_before_next_auto_pair(capture):
    pair(capture)
    assert capture.capture_frame('auto')
    assert not capture.capture_frame('auto')
    image(capture, 'front')
    assert not capture.capture_frame('auto')
    image(capture, 'rear')
    assert capture.capture_frame('auto')
    assert len(rows(capture)) == 2


@pytest.mark.parametrize('failure', ['return_false', 'exception'])
def test_failed_second_write_removes_incomplete_pair(capture, failure):
    pair(capture)
    original = cv2.imwrite
    def write(path, frame, params):
        if '/rear/' in path:
            if failure == 'exception':
                raise cv2.error('test disk failure')
            return False
        return original(path, frame, params)
    with patch('manual_drive_pkg.manual_drive_capture_node.cv2.imwrite', side_effect=write):
        assert not capture.capture_frame('manual')
    assert capture.frame_index == 0
    assert rows(capture) == []
    assert list(capture.images_dir.rglob('*.jpg')) == []


def test_stop_pauses_auto_capture_and_c_captures_while_stopped(capture):
    pair(capture)
    capture.on_timer()
    assert rows(capture) == []
    capture.handle_key('w')
    capture.on_timer()
    assert len(rows(capture)) == 1
    capture.handle_key('x')
    capture.test_clock[0] += 0.3
    pair(capture)
    capture.on_timer()
    assert len(rows(capture)) == 1
    capture.handle_key('c')
    assert rows(capture)[-1]['capture_mode'] == 'manual'
    assert rows(capture)[-1]['left_speed'] == '0'


def test_preview_handles_missing_camera_and_never_changes_raw_images(capture):
    capture.show_window = True
    for ready in (False, True):
        if ready:
            pair(capture)
        with patch('manual_drive_pkg.manual_drive_capture_node.cv2.imshow') as show, \
             patch('manual_drive_pkg.manual_drive_capture_node.cv2.waitKeyEx', return_value=ord('x')):
            capture.draw_window()
        assert show.call_args.args[1].shape == (480, 1280, 3)
    assert np.all(capture.latest_frame == 80)
    assert np.all(capture.latest_rear_frame == 160)


def test_launch_dual_settings_and_single_camera_compatibility(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setattr('vehicle_bringup_pkg.configuration.get_package_share_directory',
                        lambda name: str(ROOT / 'src' / name))
    description = launch.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    nodes = [a for a in description.entities if isinstance(a, Node)]
    rear = next(n for n in nodes if n.node_executable == 'camera_publisher_node' and n.condition is not None)
    manual = next(n for n in nodes if n.node_executable == 'manual_drive_capture_node')
    assert not rear.condition.evaluate(context)
    assert context.launch_configurations['use_rear_camera'] == 'false'
    assert context.launch_configurations['auto_capture'] == 'false'
    context.launch_configurations.update(use_rear_camera='true', camera_device='/dev/video2',
                                         rear_camera_device='/dev/video0', auto_capture='true')
    launch.validate_cameras(context)
    assert rear.condition.evaluate(context)
    settings = evaluate_parameters(context, manual._Node__parameters)[0]
    assert settings['use_rear_camera'] and settings['auto_capture']
    assert settings['rear_image_topic'] == 'manual/rear/image_raw'
    assert settings['max_pair_skew'] == 0.1
    assert sum(n.node_executable == 'serial_sender_node_v2' for n in nodes) == 1
    alias = tmp_path / 'rear_alias'
    alias.symlink_to('/dev/video2')
    context.launch_configurations['rear_camera_device'] = str(alias)
    with pytest.raises(ValueError, match='different devices'):
        launch.validate_cameras(context)
