"""Inspect launch topology and tuning persistence without starting hardware."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

from vehicle_bringup_pkg.tuning_configuration import resolve_tuning
from skku_track_drive_pkg.track_tuner_node import TrackTunerNode


ROOT = Path(__file__).resolve().parents[2]
FILES = {'track': 'track_drive_tuning.launch.py', 'mission': 'mission_drive_tuning.launch.py',
         'perpendicular': 'perpendicular_parking.launch.py', 'parallel': 'parallel_parking.launch.py'}

def topology(mode, overrides=None):
    path = ROOT / 'src/vehicle_bringup_pkg/launch' / FILES[mode]
    spec = importlib.util.spec_from_file_location('four_mode_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    actions = []
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            actions.extend(action.execute(context))
    nodes = [n for n in actions if isinstance(n, Node) and (n.condition is None or n.condition.evaluate(context))]
    return context, nodes


@pytest.mark.parametrize('mode', FILES)
def test_mode_has_one_controller_and_one_gated_serial_path(mode):
    context, nodes = topology(mode)
    names = [n.node_executable for n in nodes]
    assert sum('controller_node' in n for n in names) == 1
    assert names.count('serial_sender_node_v2') == 1
    assert names.count('drive_arm_node') == 1
    assert not any(n in names for n in ('motion_planner_node', 'mission_manager_node', 'serial_sender_node'))
    assert names.count('camera_publisher_node') == (1 if mode in ('track', 'mission') else 0)
    assert names.count('lidar_publisher_node_v2') == (1 if mode in ('perpendicular', 'parallel') else 0)
    assert context.launch_configurations['auto_calibrate'] == 'false'
    sender = next(n for n in nodes if n.node_executable == 'serial_sender_node_v2')
    assert evaluate_parameters(context, sender._Node__parameters)[0]['require_controller_heartbeat'] is True
    settings = evaluate_parameters(context, sender._Node__parameters)[0]
    assert settings['require_perception_progress'] is (mode in ('track', 'mission'))
    if mode in ('track', 'mission'):
        assert settings['perception_stop_s'] == 6.0
        assert 'perception_hold_s' not in settings and 'perception_ramp_s' not in settings


@pytest.mark.parametrize('mode', FILES)
def test_dry_run_omits_all_hardware_and_isolates_commands(mode):
    context, nodes = topology(mode, {'dry_run': 'true', 'gui': 'false'})
    assert len(nodes) == 1
    settings = evaluate_parameters(context, nodes[0]._Node__parameters)[0]
    assert settings['cmd_topic'] == '/dry_run/topic_control_signal'


def test_only_mission_uses_mission_model_and_track_preserves_default_with_tunable_speed():
    context, nodes = topology('mission')
    controller = next(n for n in nodes if n.node_executable == 'mission_controller_node')
    settings = evaluate_parameters(context, controller._Node__parameters)[0]
    assert settings['model_path'].endswith('/models/mission/best.pt')
    context, nodes = topology('track')
    controller = next(n for n in nodes if n.node_executable == 'track_controller_node')
    settings = evaluate_parameters(context, controller._Node__parameters)[0]
    assert settings['speed'] == 250 and settings['allow_speed_tuning'] is True
    assert 'model_path' not in settings


def test_each_mode_has_a_unique_saved_config_path():
    paths = [topology(mode)[0].launch_configurations['saved_tuning_path'] for mode in FILES]
    assert len(set(paths)) == 4


def test_another_mode_file_cannot_be_loaded_or_overwritten(tmp_path):
    track_file = tmp_path / 'track.yaml'
    TrackTunerNode.write_yaml_atomic(track_file, {'confidence': .5})
    context = LaunchContext()
    context.launch_configurations.update({'saved_tuning_path': str(track_file), 'tuning_config': ''})
    with pytest.raises(ValueError, match='another mode'):
        resolve_tuning(context, 'mission')


@pytest.mark.parametrize('root', ['mission_controller_node', 'perpendicular_parking_controller_node', 'parallel_parking_controller_node'])
def test_mode_yaml_round_trip_preserves_root_and_types(tmp_path, root):
    path = tmp_path / 'settings.yaml'
    values = {'forward_pwm': 80, 'lidar_x_m': .35}
    TrackTunerNode.write_yaml_atomic(path, values, root=root)
    assert yaml.safe_load(path.read_text()) == {root: {'ros__parameters': values}}


def test_gui_saves_accepted_values_and_refuses_other_mode_file(tmp_path):
    tuner = object.__new__(TrackTunerNode)
    tuner._set_future = None
    tuner.trackbars = {'confidence': None}
    tuner.managed_parameters = ('confidence',)
    tuner.current_values = tuner.desired_values = {'confidence': .5}
    tuner.loaded_config_path = ''
    tuner.parameter_root = 'mission_controller_node'
    tuner.save_path = tmp_path / 'track.yaml'
    tuner.get_logger = Mock(return_value=Mock())
    TrackTunerNode.write_yaml_atomic(tuner.save_path, {'confidence': .7})
    with pytest.raises(ValueError, match='another mode'):
        tuner.save()
    assert yaml.safe_load(tuner.save_path.read_text())['track_controller_node']['ros__parameters']['confidence'] == .7


def test_rejected_or_pending_gui_values_are_not_saved(tmp_path):
    tuner = object.__new__(TrackTunerNode)
    tuner._set_future = Mock()
    tuner.trackbars = {'confidence': None}
    tuner.current_values = {'confidence': .5}
    tuner.desired_values = {'confidence': .7}
    tuner.save_path = tmp_path / 'settings.yaml'
    tuner.get_logger = Mock(return_value=Mock())
    tuner.save()
    assert not tuner.save_path.exists()
