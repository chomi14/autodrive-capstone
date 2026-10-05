"""Hardware-free checks of parameter flow, shared paths and command ownership."""
import importlib.util
from pathlib import Path
from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import yaml
from rclpy.parameter import Parameter
from launch_ros.utilities import evaluate_parameters

from skku_track_drive_pkg.command_lease import CommandLease
from skku_track_drive_pkg.calibration_controller_node import CalibrationControllerNode
from skku_track_drive_pkg.lane_processing import LaneInfoExtractor
from skku_track_drive_pkg.path_planner import PathPlanner
from skku_track_drive_pkg.motion_planner import MotionPlanner
from skku_track_drive_pkg.mission_core import MissionCore
from skku_track_drive_pkg.mission_controller_node import MissionControllerNode
from skku_track_drive_pkg.track_controller_node import TrackControllerNode
from skku_track_drive_pkg.messages import DetectionArray, Detection, Mask
from vehicle_bringup_pkg.mode_launch import preview_remappings
from test_four_mode_launches import topology, ROOT


@pytest.mark.parametrize('mode', ['track', 'mission', 'perpendicular', 'parallel'])
def test_sensor_preview_has_sensors_without_any_vehicle_io(mode):
    context, nodes = topology(mode, {'sensors_only': 'true'})
    names = [node.node_executable for node in nodes]
    assert 'serial_sender_node_v2' not in names and 'drive_arm_node' not in names
    assert names.count('camera_publisher_node') == (mode in ('track', 'mission'))
    assert names.count('lidar_publisher_node_v2') == (mode in ('perpendicular', 'parallel'))
    controller = next(n for n in nodes if 'controller_node' in n.node_executable)
    assert evaluate_parameters(context, controller._Node__parameters)[0]['cmd_topic'] == '/sensors_only/topic_control_signal'
    assert ('vehicle/drive_key', '/sensors_only/vehicle/drive_key') in preview_remappings(context)
    context, nodes = topology(mode, {'sensors_only': 'true', 'dry_run': 'true'})
    assert not any('publisher_node' in n.node_executable for n in nodes)
    assert ('vehicle/drive_key', '/dry_run/vehicle/drive_key') in preview_remappings(context)


def test_track_explicit_speed_reaches_controller_and_gui_is_tunable():
    context, nodes = topology('track', {'speed': '37'})
    controller = next(n for n in nodes if n.node_executable == 'track_controller_node')
    gui = next(n for n in nodes if n.node_executable == 'track_tuner_node')
    settings = evaluate_parameters(context, controller._Node__parameters)[0]
    assert settings['speed'] == 37 and settings['allow_speed_tuning'] is True
    assert evaluate_parameters(context, gui._Node__parameters)[0]['allow_speed_tuning'] is True


def test_manual_launch_has_only_calibration_producer_and_private_io():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location('manual_launch', ROOT/'src/vehicle_bringup_pkg/launch/parking_calibration.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for dry in ('false', 'true'):
        context = LaunchContext()
        context.launch_configurations['dry_run'] = dry
        actions = []
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
            elif isinstance(action, OpaqueFunction):
                actions.extend(action.execute(context))
        nodes = [action for action in actions if isinstance(action, Node)]
        names = [n.node_executable for n in nodes]
        assert sum('controller_node' in name for name in names) == 1
        assert 'parking_controller_node' not in names
        controller = next(n for n in nodes if n.node_executable == 'parking_calibration_controller_node')
        settings = evaluate_parameters(context, controller._Node__parameters)[0]
        assert settings['pwm'] == settings['steering_step'] == 0
        if dry == 'false':
            sender = next(n for n in nodes if n.node_executable == 'serial_sender_node_v2')
            assert evaluate_parameters(context, sender._Node__parameters)[0]['topic'] == settings['cmd_topic'] == '/parking_calibration/command'
        else:
            assert len(nodes) == 2
            assert settings['cmd_topic'] == '/dry_run/topic_control_signal'


def bare_controller(cls, speed):
    node = object.__new__(cls)
    node.profile = node.publish_debug = node.debug_log = False
    node.enabled = node.allow_speed_tuning = True
    node.speed = speed
    node._parameter_lock = RLock()
    node._image_to_bgr = lambda frame: frame
    node.lane = LaneInfoExtractor(show_image=False, bev_top_shift=-15, look_shift=0, slope_ema_alpha=.6, virtual_lane_width=290)
    node.path_planner = PathPlanner(car_center_point=(325, 179))
    node.motion = MotionPlanner(stanley_gain=.027, heading_gain=.55, lookahead_steps=30,
        max_steering=7., max_steering_angle_deg=50., heading_step=3,
        car_center_x=325., car_center_y=179., default_left_speed=speed, default_right_speed=speed, stop_on_red=False)
    node.cmd_pub = Mock()
    node.get_logger = Mock(return_value=Mock())
    node.frame_counter, node.last_t = 0, None
    if cls is MissionControllerNode:
        node.mission = MissionCore()
        node.status_pub = Mock()
        node._processed_lane = 2
    return node


def detection_for_bev(extractor, offset):
    src = np.float32(extractor._src_mat())
    dst = np.float32([[192, 0], [448, 0], [448, 480], [192, 480]])
    inverse = cv2.getPerspectiveTransform(dst, src)
    bird = np.zeros((480, 640), np.uint8)
    for y in range(480):
        center = int(320 + offset + .001 * (y-400)**2)
        bird[y, max(0, center-45):min(640, center+45)] = 255
    bitmap = cv2.warpPerspective(bird, inverse, (640, 480))
    return DetectionArray([Detection(class_name='lane2', mask=Mask(bitmap=bitmap, width=640, height=480))])


def test_same_detections_and_shared_tuning_produce_identical_paths_and_steering():
    track, mission = bare_controller(TrackControllerNode, 250), bare_controller(MissionControllerNode, 80)
    frame = np.zeros((480, 640, 3), np.uint8)
    for offset in (-25, 0, 25, 15, -15):
        detections = detection_for_bev(track.lane, offset)
        for node in (track, mission):
            node.yolo = SimpleNamespace(detect=lambda f: detections)
            node.on_image(frame)
            assert node.lane.missed_frames == 0
        assert len(track.motion.path_data) >= 2
        assert track.motion.path_data == pytest.approx(mission.motion.path_data)
        tc, mc = track.cmd_pub.publish.call_args.args[0], mission.cmd_pub.publish.call_args.args[0]
        assert tc.steering == mc.steering
        assert track.motion.last_cte == pytest.approx(mission.motion.last_cte)
        assert track.motion.last_heading_deg == pytest.approx(mission.motion.last_heading_deg)
        assert (tc.left_speed, mc.left_speed) == (250, 80)
        assert mission.mission.status['state'] == 'NORMAL'
    result = track.on_tuning_parameters([Parameter('speed', value=37)])
    assert result.successful
    track.on_image(frame)
    command = track.cmd_pub.publish.call_args.args[0]
    assert track.speed == track.motion.default_left_speed == command.left_speed == command.right_speed == 37


def test_live_command_lease_blocks_parallel_and_manual_publishers(tmp_path):
    path = tmp_path/'lease'
    first = CommandLease('topic_control_signal', 'parallel_parking_controller_node', path)
    with pytest.raises(RuntimeError, match='parallel_parking_controller_node'):
        CommandLease('/parking_calibration/command', 'calibration', path)
    preview = CommandLease('/sensors_only/topic_control_signal', 'preview', path)
    preview.close()
    first.close()
    second = CommandLease('/parking_calibration/command', 'calibration', path)
    second.close()


@pytest.mark.parametrize('pwm,steer', [(60, 0), (-60, 0), (0, -7), (0, 7)])
def test_calibration_command_requires_actual_arm_and_preserves_sign(pwm, steer):
    node = object.__new__(CalibrationControllerNode)
    node.settings = {'pwm': pwm, 'steering_step': steer}
    node.armed = False
    assert node.command().left_speed == node.command().steering == 0
    node.armed = True
    assert (node.command().left_speed, node.command().right_speed, node.command().steering) == (pwm, pwm, steer)
    assert not node.on_parameters([Parameter('pwm', value=30)]).successful
    node.armed = False
    assert node.on_parameters([Parameter('pwm', value=30)]).successful


def test_mission_bootstrap_is_independent_and_preserves_existing_file(tmp_path):
    spec = importlib.util.spec_from_file_location('bootstrap', ROOT/'tools/bootstrap_mission_tuning.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    track_default, mission_default, track_saved, mission_saved = [tmp_path/name for name in ('td', 'md', 'ts', 'ms')]
    def write(path, root, settings):
        path.write_text(yaml.safe_dump({root: {'ros__parameters': settings}}))
    write(track_default, 'track_controller_node', {'speed': 250, 'heading_gain': .6})
    write(track_saved, 'track_controller_node', {'speed': 37, 'heading_gain': .55})
    write(mission_default, 'mission_controller_node', {'speed': 80, 'heading_gain': .6, 'yellow_speed': 60})
    report = module.bootstrap(track_default, mission_default, track_saved, mission_saved)
    assert report['created']
    saved = yaml.safe_load(mission_saved.read_text())['mission_controller_node']['ros__parameters']
    assert saved == {'speed': 80, 'heading_gain': .55, 'yellow_speed': 60}
    before = mission_saved.read_bytes()
    write(track_saved, 'track_controller_node', {'heading_gain': .9})
    report = module.bootstrap(track_default, mission_default, track_saved, mission_saved)
    assert not report['created'] and report['previous_differences']['heading_gain'] == {'track': .9, 'mission': .55}
    assert mission_saved.read_bytes() == before
