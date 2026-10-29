"""Pure track drive pipeline with the shared traffic-light stop/go overlay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from vehicle_bringup_pkg.configuration import config_argument, VehicleDefault
from vehicle_bringup_pkg.mode_launch import (
    camera_node,
    command_topic,
    hardware_enabled,
    motor_enabled,
    preview_remappings,
    vehicle_io_nodes,
)
from vehicle_bringup_pkg.perception_policy import perception_arguments
from vehicle_bringup_pkg.recording import (
    analysis_enabled,
    recording_arguments,
    recording_nodes,
)
from vehicle_bringup_pkg.tuning_configuration import (
    resolve_tuning,
    tuning_launch_arguments,
)


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('1', 'true', 'yes', 'on'):
        return True
    if value in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false, got: {value}')


def _launch_nodes(context):
    tuning, selected, save_path, source = resolve_tuning(context, 'traffic_track')
    enable_visualization = _as_bool(context, 'enable_visualization')
    show_bev = _as_bool(context, 'show_bev')
    profile = _as_bool(context, 'profile')
    debug_log = _as_bool(context, 'debug_log')

    controller_parameters = dict(tuning)
    controller_parameters.update({
        'image_topic': '/track/image_raw',
        'cmd_topic': command_topic(context),
        'device': LaunchConfiguration('device'),
        'start_enabled': True,
        'allow_speed_tuning': True,
        'steering_sign': ParameterValue(
            LaunchConfiguration('steering_sign'), value_type=float
        ),
        'max_steering': 7.0,
        'publish_debug': enable_visualization,
        'publish_bev_debug': show_bev,
        'debug_log': debug_log,
        'profile': profile,
        'publish_analysis': analysis_enabled(context),
        'profile_input_fps': 30.0,
        'profile_report_interval': 100,
        'image_reliability': 'reliable',
    })

    return [
        LogInfo(msg=f'Using {source} traffic-light track tuning config: {selected}'),
        *([camera_node('/track/image_raw')] if hardware_enabled(context) else []),
        Node(
            package='skku_track_drive_pkg',
            executable='traffic_light_track_controller_node',
            name='traffic_light_track_controller_node',
            output='screen',
            parameters=[controller_parameters],
            remappings=preview_remappings(context),
        ),
        Node(
            package='skku_track_drive_pkg',
            condition=IfCondition(LaunchConfiguration('gui')),
            executable='traffic_light_track_tuner_node',
            name='traffic_light_track_tuner_node',
            output='screen',
            parameters=[{
                'target_node': '/traffic_light_track_controller_node',
                'cmd_topic': command_topic(context),
                'debug_topic': '/track_debug_image',
                'status_topic': '/traffic_light_track/status',
                'save_path': str(save_path),
                'loaded_config_path': str(selected),
                'allow_speed_tuning': True,
            }],
            remappings=preview_remappings(context),
        ),
        *(vehicle_io_nodes(perception_mode='track') if motor_enabled(context) else []),
        *recording_nodes(context, 'track', profile_mode='traffic_track'),
    ]


def generate_launch_description():
    return LaunchDescription([
        config_argument(),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument(
            'sensors_only',
            default_value='false',
            description='Keep real sensors/GUI; omit vehicle serial and arm gate',
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        *perception_arguments('track'),
        *recording_arguments(),
        DeclareLaunchArgument(
            'command_timeout',
            default_value='0.0',
            description='Controller health lease; 0 uses ui_timeout for health',
        ),
        DeclareLaunchArgument(
            'ui_timeout',
            default_value='0.75',
            description='Arm gate/tuner heartbeat timeout in seconds',
        ),
        DeclareLaunchArgument(
            'auto_calibrate',
            default_value=VehicleDefault('steering.auto_calibrate', False),
        ),
        DeclareLaunchArgument(
            'arduino_baud',
            default_value=VehicleDefault('arduino.baud', 115200),
        ),
        DeclareLaunchArgument(
            'camera_device',
            default_value=VehicleDefault('camera.front.device', '/dev/video0'),
        ),
        DeclareLaunchArgument(
            'arduino_port',
            default_value=VehicleDefault('arduino.port', '/dev/arduino'),
        ),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('visualize', default_value='True'),
        DeclareLaunchArgument(
            'enable_visualization', default_value=LaunchConfiguration('visualize')
        ),
        DeclareLaunchArgument('show_bev', default_value='true'),
        DeclareLaunchArgument('profile', default_value='false'),
        DeclareLaunchArgument('debug_log', default_value='false'),
        DeclareLaunchArgument('steering_sign', default_value='1.0'),
        DeclareLaunchArgument(
            'calibration_tolerance',
            default_value=VehicleDefault('steering.calibration_tolerance', '35'),
        ),
        *tuning_launch_arguments('traffic_track'),
        OpaqueFunction(function=_launch_nodes),
    ])
