"""Optional sidecar; never owns vehicle control or changes driving settings."""
import json
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from .mode_launch import command_topic, motor_enabled, hardware_enabled


def recording_arguments():
    return [DeclareLaunchArgument('record_dir', default_value='', description='New session directory; empty disables rosbag/control recording'),
            DeclareLaunchArgument('analysis', default_value='false', description='Publish read-only path/error/mission evidence')]


def analysis_enabled(context):
    return bool(LaunchConfiguration('record_dir', default='').perform(context).strip()) or LaunchConfiguration('analysis', default='false').perform(context).lower() in ('true', '1')


def recording_nodes(context, mode):
    directory = LaunchConfiguration('record_dir', default='').perform(context).strip()
    if not directory:
        return []
    prefix = '/vehicle' if motor_enabled(context) else '/sensors_only/vehicle' if hardware_enabled(context) else '/dry_run/vehicle'
    topic = command_topic(context)
    if mode == 'calibration' and motor_enabled(context):
        prefix, topic = '/parking_calibration/vehicle', '/parking_calibration/command'
    from .tuning_configuration import _profile
    root, _, _, allowed = _profile(mode)
    return [Node(package='skku_track_drive_pkg', executable='tuning_recorder_node', output='screen', parameters=[{
        'output_dir': directory, 'mode': mode, 'controller_root': root, 'tuning_keys_json': json.dumps(sorted(allowed)), 'command_topic': topic, 'vehicle_prefix': prefix,
        'input_topic': '/track/image_raw' if mode == 'track' else '/mission/image_raw' if mode == 'mission'
                       else '' if mode == 'calibration' else '/parking/scan'}])]
