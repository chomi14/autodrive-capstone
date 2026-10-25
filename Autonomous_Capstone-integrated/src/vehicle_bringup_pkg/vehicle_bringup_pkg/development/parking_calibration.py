"""Standalone manual measurement; no automatic parking/track controller."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction, LogInfo
from launch_ros.actions import Node
from vehicle_bringup_pkg.configuration import config_argument
from vehicle_bringup_pkg.recording import recording_nodes
from vehicle_bringup_pkg.mode_launch import common_arguments, motor_enabled, preview_remappings, vehicle_remappings, command_topic, vehicle_io_nodes, as_bool
from vehicle_bringup_pkg.tuning_configuration import resolve_tuning, tuning_launch_arguments


def assemble(context):
    settings, selected, saved, source = resolve_tuning(context, 'calibration')
    topic = '/parking_calibration/command' if motor_enabled(context) else command_topic(context)
    remappings = vehicle_remappings('/parking_calibration') if motor_enabled(context) else preview_remappings(context)
    settings['cmd_topic'] = topic
    actions = [LogInfo(msg=f'Manual calibration: {source} {selected}; S before changing PWM/steering'),
        Node(package='skku_track_drive_pkg', executable='parking_calibration_controller_node',
             name='parking_calibration_controller_node', output='screen', parameters=[settings], remappings=remappings)]
    if as_bool(context, 'gui'):
        actions.append(Node(package='skku_track_drive_pkg', executable='parking_calibration_tuner_node',
            name='parking_calibration_tuner_node', output='screen', parameters=[{
                'target_node': '/parking_calibration_controller_node', 'cmd_topic': topic,
                'status_topic': '/parking_calibration/status', 'save_path': str(saved),
                'loaded_config_path': str(selected), 'allow_speed_tuning': True}], remappings=remappings))
    if motor_enabled(context):
        actions.extend(vehicle_io_nodes(topic, remappings))
    actions.extend(recording_nodes(context, 'calibration'))
    return actions


def generate_launch_description():
    return LaunchDescription([config_argument(), *common_arguments('0.5', camera=False),
                              *tuning_launch_arguments('calibration'), OpaqueFunction(function=assemble)])
