"""Real ROS launch assembly checks; explicitly skipped when ROS is absent."""
import copy
from pathlib import Path

import pytest
import yaml

pytest.importorskip('launch')
pytest.importorskip('launch_ros')

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from vehicle_bringup_pkg.modes.perpendicular_camera.launch import assemble
from skku_track_drive_pkg.modes.perpendicular_camera.configuration import load_config

ROOT = Path(__file__).resolve().parents[3]


def topology(tmp_path, preview, **options):
    data = copy.deepcopy(load_config(ROOT/'src/vehicle_bringup_pkg/config/perpendicular_camera.yaml'))
    data['control']['calibration_confirmed'] = options.pop('calibrated', True)
    for role in ('front', 'rear'):
        weight = tmp_path/(role+'.pt')
        weight.touch()  # path checks only; launch nodes are NEVER started
        data[role]['model_path'] = str(weight)
    if options.pop('same_camera', False):
        data['rear']['device'] = data['front']['device']
    config = tmp_path/'config.yaml'
    config.write_text(yaml.safe_dump(data))
    settings = dict(config_file=str(config), source='camera', gui='true', use_sim_time='false',
                    command_timeout='0.5', ui_timeout='0.75', arduino_port='/dev/arduino',
                    arduino_baud='115200', auto_calibrate='false', calibration_tolerance='35',
                    front_model='', rear_model='', front_camera_device='', rear_camera_device='')
    settings.update(options)
    context = LaunchContext()
    context.launch_configurations.update(settings)
    return context, [node for node in assemble(context, preview) if isinstance(node, Node)]


def test_preview_has_no_serial_and_one_isolated_command_producer(tmp_path):
    context, nodes = topology(tmp_path, True)
    executables = [node.node_executable for node in nodes]
    assert executables.count('camera_publisher_node') == 2
    assert executables.count('camera_parking_perception_node') == 2
    assert executables.count('camera_parking_controller_node') == 1
    assert 'serial_sender_node_v2' not in executables and 'drive_arm_node' not in executables
    controller = next(node for node in nodes if node.node_executable == 'camera_parking_controller_node')
    assert evaluate_parameters(context, controller._Node__parameters)[0]['cmd_topic'].startswith('/sensors_only/')


def test_live_uses_existing_single_serial_and_arm_gate(tmp_path):
    _, nodes = topology(tmp_path, False)
    executables = [node.node_executable for node in nodes]
    assert executables.count('serial_sender_node_v2') == 1
    assert executables.count('drive_arm_node') == 1


@pytest.mark.parametrize('settings', [dict(calibrated=False), dict(source='replay'),
    dict(gui='false'), dict(use_sim_time='true'), dict(same_camera=True)])
def test_live_invalid_conditions_fail_before_starting_processes(tmp_path, settings):
    with pytest.raises(ValueError):
        topology(tmp_path, False, **settings)


def test_replay_preview_does_not_open_cameras(tmp_path):
    _, nodes = topology(tmp_path, True, source='replay')
    assert not any(node.node_executable == 'camera_publisher_node' for node in nodes)
