"""Run the actual launch assembly function with inert action doubles.

Checks selection/isolation before process creation; real launch_ros tests are
in test_ros_launch.py and require a ROS installation.
"""
import ast
import copy
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from skku_track_drive_pkg.modes.perpendicular_camera.configuration import load_config
from skku_track_drive_pkg.modes.perpendicular_camera.core import ControlConfig
from skku_track_drive_pkg.modes.perpendicular_camera.model import model_path

ROOT = Path(__file__).resolve().parents[3]
FILE = ROOT/'src/vehicle_bringup_pkg/vehicle_bringup_pkg/modes/perpendicular_camera/launch.py'


class Action:
    def __init__(self, *args, **kwargs):
        self.kwargs = kwargs


class FakeLaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


def plan(tmp_path, preview=True, **options):
    definitions = [node for node in ast.parse(FILE.read_text()).body
                   if isinstance(node, ast.FunctionDef) and node.name in ('_bool', 'assemble')]
    ns = dict(Path=Path, os=os, json=json, LaunchConfiguration=FakeLaunchConfiguration,
              load_config=load_config, ControlConfig=ControlConfig, model_path=model_path,
              Node=Action, EmitEvent=Action, Shutdown=Action, RegisterEventHandler=Action, OnProcessExit=Action,
              vehicle_remappings=lambda prefix: [('vehicle/drive_key', prefix+'/vehicle/drive_key')],
              vehicle_io_nodes=lambda: [Action(executable='serial_sender_node_v2'), Action(executable='drive_arm_node')])
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(FILE), 'exec'), ns)
    data = copy.deepcopy(load_config(ROOT/'src/vehicle_bringup_pkg/config/perpendicular_camera.yaml'))
    data['control']['calibration_confirmed'] = options.pop('calibrated', True)
    for role in ('front', 'rear'):
        weights = tmp_path/(role+'.pt')
        weights.touch()
        data[role]['model_path'] = str(weights)
    if options.pop('same_camera', False):
        data['rear']['device'] = data['front']['device']
    config = tmp_path/'settings.yaml'
    config.write_text(yaml.safe_dump(data))
    context = dict(config_file=str(config), source='camera', gui='true', use_sim_time='false',
                   front_model='', rear_model='', front_camera_device='', rear_camera_device='')
    context.update(options)
    actions = ns['assemble'](context, preview)
    return [a.kwargs for a in actions if 'executable' in a.kwargs]


def test_preview_isolates_commands_observations_services_and_hardware(tmp_path):
    nodes = plan(tmp_path)
    names = [node['executable'] for node in nodes]
    assert names.count('camera_parking_controller_node') == 1
    assert 'serial_sender_node_v2' not in names and 'drive_arm_node' not in names
    for node in nodes:
        assert node['namespace'] == 'parking_camera_preview'
        mappings = dict(node['remappings'])
        assert mappings['/parking_camera/front/observation'] == '/parking_camera_preview/front/observation'
        assert mappings['/perpendicular_camera_controller/reset'].startswith('/parking_camera_preview/')
        assert mappings['/parking_camera/front/image_raw'] == '/parking_camera_preview/front/image_raw'
    controller = next(n for n in nodes if n['executable'] == 'camera_parking_controller_node')
    assert controller['parameters'][0]['cmd_topic'] == '/sensors_only/topic_control_signal'


def test_replay_opens_no_camera_or_serial_but_reads_configured_bag_topics(tmp_path):
    nodes = plan(tmp_path, source='replay', use_sim_time='true')
    assert not any(n['executable'] in ('camera_publisher_node', 'serial_sender_node_v2') for n in nodes)
    assert all('/parking_camera/front/image_raw' not in dict(n['remappings']) for n in nodes)


@pytest.mark.parametrize('settings', [dict(source='replay'), dict(use_sim_time='true'),
    dict(gui='false'), dict(calibrated=False), dict(same_camera=True)])
def test_live_bad_conditions_fail_before_any_process(tmp_path, settings):
    with pytest.raises(ValueError):
        plan(tmp_path, preview=False, **settings)


def test_live_reuses_single_serial_arm_and_saves_effective_cli_overrides(tmp_path):
    nodes = plan(tmp_path, preview=False, front_camera_device='/dev/custom-front')
    assert [n['executable'] for n in nodes].count('serial_sender_node_v2') == 1
    assert [n['executable'] for n in nodes].count('drive_arm_node') == 1
    controller = next(n for n in nodes if n['executable'] == 'camera_parking_controller_node')
    effective = json.loads(controller['parameters'][0]['session_overrides_json'])
    assert effective['front']['device'] == '/dev/custom-front'


def test_all_camera_python_files_compile_even_without_ros():
    paths = list((ROOT/'src/skku_track_drive_pkg/skku_track_drive_pkg/modes/perpendicular_camera').glob('*.py'))
    paths += list((ROOT/'src/vehicle_bringup_pkg/vehicle_bringup_pkg/modes/perpendicular_camera').glob('*.py'))
    paths += list((ROOT/'src/vehicle_bringup_pkg/launch').glob('perpendicular_camera*.launch.py'))
    for path in paths:
        compile(path.read_text(), str(path), 'exec')
