"""One 1125 launch, with its controller replaced by the integrated entry point."""

import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from skku_track_drive_pkg.modes.perpendicular_camera.configuration import TUNING_TOPIC, load_config
from skku_track_drive_pkg.modes.perpendicular_camera.model import model_path
from vehicle_bringup_pkg.modes.perpendicular_camera.launch import assemble as upstream_assemble
from vehicle_bringup_pkg.mode_launch import vehicle_remappings


def assemble(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    if arg('preview') not in ('true', 'false'):
        raise ValueError('preview must be true or false')
    preview = arg('preview') == 'true'
    path = Path(arg('config_file')).expanduser().resolve()
    document = yaml.safe_load(path.read_text(encoding='utf-8-sig'))
    if not isinstance(document, dict):
        raise ValueError('config_file must contain the 1125 camera parking YAML')
    settings = document.pop('mission2', None)
    if settings is not None and not isinstance(settings, dict):
        raise ValueError('mission2 must be a mapping in the same YAML as perpendicular parking')
    if not preview and (not settings or settings.get('commissioned') is not True):
        raise ValueError('live mode requires the measured mission2 block in config_file and commissioned: true')
    if not 0 < float(arg('command_timeout')) <= .25:
        raise ValueError('mission 2 requires a bounded controller health lease of at most .25 seconds')
    # 1125 validates exactly four root fields; give those nodes an isolated
    # session copy. Even its GUI Save action cannot overwrite the user's YAML.
    session_directory = TemporaryDirectory(prefix='seyeon-mission2-')
    runtime_path = Path(session_directory.name, 'perpendicular_camera.yaml')
    document['save_path'] = str(Path(session_directory.name, 'perpendicular_saved.yaml'))
    runtime_path.write_text(yaml.safe_dump(document, allow_unicode=True), encoding='utf-8')
    data = load_config(str(runtime_path))
    original_config_argument = context.launch_configurations['config_file']
    try:
        context.launch_configurations['config_file'] = str(runtime_path)
        actions = [action for action in upstream_assemble(context, preview) if isinstance(action, Node)]
    finally:
        context.launch_configurations['config_file'] = original_config_argument
    replaced = [index for index, action in enumerate(actions) if action.node_executable == 'camera_parking_controller_node']
    if len(replaced) != 1:
        raise RuntimeError('expected exactly one 1125 camera parking controller')
    namespace, remaps = '', []
    if preview:
        namespace = 'parking_camera_preview'
        remaps = vehicle_remappings('/sensors_only')
        for role in ('front', 'rear'):
            for suffix in ('oriented', 'segmentation', 'bev', 'observation', 'perception_status'):
                remaps.append((f'/parking_camera/{role}/{suffix}', f'/parking_camera_preview/{role}/{suffix}'))
            remaps.append((data[role]['image_topic'], f'/parking_camera_preview/{role}/image_raw'))
        for suffix in ('status', 'preview_enabled'):
            remaps.append(('/parking_camera/' + suffix, '/parking_camera_preview/' + suffix))
        remaps.append((TUNING_TOPIC, '/parking_camera_preview/tuning'))
    session = {role: {'device': arg(role + '_camera_device') or data[role]['device'],
                      'model_path': str(Path(model_path(arg(role + '_model') or data[role]['model_path'])).resolve())}
               for role in ('front', 'rear')}
    script = Path(__file__).resolve().with_name('mission2_parking_node.py')
    actions[replaced[0]] = Node(executable=str(script), prefix=[sys.executable],
                               name='perpendicular_camera_controller', namespace=namespace,
                               remappings=remaps, output='screen', parameters=[{
                                   'config_file': str(runtime_path), 'mission2_source': str(path),
                                   'preview': preview, 'session_overrides_json': json.dumps(session),
                                   'exit_enabled': False,
                                   'cmd_topic': '/sensors_only/topic_control_signal' if preview else 'topic_control_signal',
                               }])
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[EmitEvent(
        event=Shutdown(reason='Mission 2 component exited'))])) for node in actions]
    def cleanup(_context):
        session_directory.cleanup()
        return []

    note = ('Using mission2 settings from the existing perpendicular-camera YAML' if settings else
            'Perpendicular preview only: no measured mission2 block in the existing YAML')
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            LogInfo(msg=note), *handlers, *actions]


def generate_launch_description():
    defaults = {'preview': 'true', 'source': 'camera', 'gui': 'true', 'use_sim_time': 'false',
                'front_camera_device': '', 'rear_camera_device': '', 'front_model': '', 'rear_model': '',
                'command_timeout': '0.25', 'ui_timeout': '0.75', 'arduino_port': '/dev/arduino',
                'arduino_baud': '115200', 'auto_calibrate': 'false', 'calibration_tolerance': '35'}
    return LaunchDescription([
        DeclareLaunchArgument('config_file'),
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        SetParameter(name='use_sim_time', value=False), OpaqueFunction(function=assemble),
    ])
