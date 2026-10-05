"""Standalone mission/parking launch assembly; exactly one command producer."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from .configuration import config_argument
from .mode_launch import common_arguments, hardware_enabled, motor_enabled, preview_remappings, as_bool, camera_node, lidar_node, vehicle_io_nodes, command_topic
from .tuning_configuration import resolve_tuning, tuning_launch_arguments, _profile
from .perception_policy import perception_arguments
from .recording import recording_nodes, analysis_enabled


def assemble(context, mode):
    settings, selected, save_path, source = resolve_tuning(context, mode)
    parking = mode != 'mission'
    root = _profile(mode)[0]
    image_topic = '/mission/image_raw'
    debug_topic = '/parking/debug_image' if parking else '/mission/debug_image'
    settings.update({'publish_debug': True, 'publish_bev_debug': not parking, 'cmd_topic': command_topic(context)})
    if parking:
        settings.update({'mode': mode, 'scan_topic': '/parking/scan'})
        settings['driver_rotation_offset_deg'] = ParameterValue(LaunchConfiguration('lidar_rotation'), value_type=float)
        executable = 'parking_controller_node'
    else:
        settings.update({'model_path': LaunchConfiguration('model_path'), 'device': LaunchConfiguration('device'),
                         'image_topic': image_topic, 'debug_topic': debug_topic, 'image_reliability': 'reliable',
                         'publish_analysis': analysis_enabled(context), 'start_enabled': True, 'max_steering': 7.0, 'allow_speed_tuning': True,
                         'steering_sign': ParameterValue(LaunchConfiguration('steering_sign'), value_type=float)})
        executable = 'mission_controller_node'
    actions = [LogInfo(msg=f'{mode}: using {source} tuning {selected}; save to {save_path}')]
    if hardware_enabled(context):
        actions.extend([lidar_node('/parking/scan')] if parking else [camera_node(image_topic)])
    actions.append(Node(package='skku_track_drive_pkg', executable=executable, name=root,
                        output='screen', parameters=[settings], remappings=preview_remappings(context)))
    if as_bool(context, 'gui'):
        actions.append(Node(package='skku_track_drive_pkg', executable=f'{mode}_tuner_node', name=f'{mode}_tuner_node',
                            output='screen', parameters=[{
                                'target_node': '/' + root, 'cmd_topic': command_topic(context), 'debug_topic': debug_topic,
                                'status_topic': '/parking/status' if parking else '/mission/status',
                                'save_path': str(save_path), 'loaded_config_path': str(selected),
                                'allow_speed_tuning': not parking,
                            }], remappings=preview_remappings(context)))
    if motor_enabled(context):
        actions.extend(vehicle_io_nodes(perception_mode='mission' if not parking else None))
    actions.extend(recording_nodes(context, mode))
    return actions


def description(mode):
    camera = mode == 'mission'
    arguments = [config_argument(), *common_arguments('0.75' if camera else '0.25', camera=camera)]
    if camera:
        arguments.extend(perception_arguments('mission'))
        default_model = str(Path(get_package_share_directory('skku_track_drive_pkg'), 'models/mission/best.pt'))
        arguments.append(DeclareLaunchArgument('model_path', default_value=default_model,
                                             description='Segmentation model with lane1, lane2, obstacle, traffic_light'))
    arguments.extend(tuning_launch_arguments(mode))
    # Steering sign is already declared as a numeric parking tuning argument.
    if camera:
        arguments.append(DeclareLaunchArgument('steering_sign', default_value='1.0'))
    arguments.append(OpaqueFunction(function=lambda context: assemble(context, mode)))
    return LaunchDescription(arguments)
