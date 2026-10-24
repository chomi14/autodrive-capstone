"""Real-track drive pipeline with a parameter-only OpenCV tuning panel."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from vehicle_bringup_pkg.configuration import config_argument, VehicleDefault
from vehicle_bringup_pkg.perception_policy import perception_arguments
from vehicle_bringup_pkg.mode_launch import camera_node, lidar_node, vehicle_io_nodes, hardware_enabled, motor_enabled, preview_remappings, command_topic
from vehicle_bringup_pkg.tuning_configuration import (
    resolve_tuning,
    tuning_launch_arguments,
    TUNING_TYPES,
)


from vehicle_bringup_pkg.recording import recording_arguments, recording_nodes, analysis_enabled

def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('1', 'true', 'yes', 'on'):
        return True
    if value in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false, got: {value}')


def _launch_nodes(context):
    if _as_bool(context, 'bbox_obstacle_avoidance'):
        # Run one controller/camera/sender only. Keep the current track calibration.
        forwarded = {
            name: LaunchConfiguration(name).perform(context)
            for name in ('vehicle_config', 'dry_run', 'sensors_only', 'gui', 'camera_device',
                         'arduino_port', 'arduino_baud', 'auto_calibrate', 'calibration_tolerance',
                         'device', 'steering_sign', 'command_timeout', 'ui_timeout',
                         'perception_stop_s', 'record_dir', 'analysis', *TUNING_TYPES)
        }
        track_file = LaunchConfiguration('tuning_config').perform(context).strip()
        if track_file and not Path(track_file).expanduser().is_file():
            raise RuntimeError(f'Explicit track tuning config not found: {track_file}')
        forwarded.update({
            'track_tuning_path': track_file or LaunchConfiguration('saved_tuning_path').perform(context),
            'load_saved_track_tuning': 'true',
            'load_saved_tuning': 'false',
            # Override parent launch scope so a track YAML is never read as a mission YAML.
            'tuning_config': '',
            'saved_tuning_path': str(Path.home() / '.config/autodrive/mission_bbox_only_tuning.yaml'),
            'publish_debug': str(_as_bool(context, 'enable_visualization')).lower(),
            'publish_bev_debug': str(_as_bool(context, 'show_bev')).lower(),
            'profile': str(_as_bool(context, 'profile')).lower(),
            'debug_log': str(_as_bool(context, 'debug_log')).lower(),
        })
        model = LaunchConfiguration('obstacle_model_path').perform(context).strip()
        if model:
            forwarded['model_path'] = model
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('vehicle_bringup_pkg'))
                                             / 'launch/mission_bbox_only.launch.py')),
            launch_arguments=forwarded.items(),
        )]
    tuning_parameters, selected_config, save_path, config_source = resolve_tuning(context)

    camera_device = LaunchConfiguration('camera_device')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_rotation = LaunchConfiguration('lidar_rotation')
    use_lidar = LaunchConfiguration('use_lidar')
    arduino_port = LaunchConfiguration('arduino_port')
    device = LaunchConfiguration('device')
    steering_sign = LaunchConfiguration('steering_sign')
    cal_tolerance = LaunchConfiguration('calibration_tolerance')
    enable_visualization = _as_bool(context, 'enable_visualization')
    show_bev = _as_bool(context, 'show_bev')
    profile = _as_bool(context, 'profile')
    debug_log = _as_bool(context, 'debug_log')
    controller_parameters = dict(tuning_parameters)
    controller_parameters.update({
        # Keep the real-time track stream isolated from legacy /image_raw
        # publishers, some of which label OpenCV frames as generic 8UC3.
        'image_topic': '/track/image_raw',
        'cmd_topic': command_topic(context),
        'device': device,
        'start_enabled': True,
        'allow_speed_tuning': True,
        'steering_sign': ParameterValue(steering_sign, value_type=float),
        'max_steering': 7.0,
        'publish_debug': enable_visualization,
        'publish_bev_debug': show_bev,
        'debug_log': debug_log,
        'profile': profile,
        'publish_analysis': analysis_enabled(context),
        'profile_input_fps': 30.0,
        'profile_report_interval': 100,
        # 640x480 BGR8 is about 0.92 MB per frame.  With BEST_EFFORT, losing
        # one DDS fragment discards the entire image; isolated measurements on
        # this host dropped receiver throughput from ~30 Hz to ~15 Hz.
        'image_reliability': 'reliable',
    })

    return [
        LogInfo(msg=f'Using {config_source} tuning config: {selected_config}'),
        *([camera_node('/track/image_raw')] if hardware_enabled(context) else []),
        *([lidar_node('lidar_raw')] if hardware_enabled(context) and _as_bool(context, 'use_lidar') else []),
        Node(
            package='skku_track_drive_pkg',
            executable='track_controller_node',
            name='track_controller_node',
            output='screen',
            parameters=[controller_parameters],
            remappings=preview_remappings(context),
        ),
        Node(
            package='skku_track_drive_pkg',
            condition=IfCondition(LaunchConfiguration('gui')),
            executable='track_tuner_node',
            name='track_tuner_node',
            output='screen',
            parameters=[{
                'target_node': '/track_controller_node',
                'cmd_topic': command_topic(context),
                'debug_topic': '/track_debug_image',
                'save_path': str(save_path),
                'loaded_config_path': str(selected_config),
                'allow_speed_tuning': True,
            }],
            remappings=preview_remappings(context),
        ),
        # Tuner keys go through DriveArmNode. Both UI event loops must remain
        # alive; only fresh W + calibration READY authorize serial motion.
        *(vehicle_io_nodes(perception_mode='track') if motor_enabled(context) else []),
        *recording_nodes(context, 'track'),
    ]


def generate_launch_description():
    return LaunchDescription([
        config_argument(),
        DeclareLaunchArgument('bbox_obstacle_avoidance', default_value='false',
                              description='Enable obstacle bbox path checks using the mission model and current track tuning'),
        DeclareLaunchArgument('obstacle_model_path', default_value='',
                              description='Optional lane1/lane2/obstacle/traffic_light segmentation model'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('sensors_only', default_value='false', description='Keep real sensors/GUI; omit vehicle serial and arm gate'),
        DeclareLaunchArgument('gui', default_value='true'),
        *perception_arguments('track'),
        *recording_arguments(),
        # No inference-age stop. Reuse ui_timeout for finite controller health
        # when command_timeout is zero; serial itself still runs at 20 Hz.
        DeclareLaunchArgument('command_timeout', default_value='0.0',
                              description='Controller health lease; 0 disables inference age checking and uses ui_timeout for health'),
        DeclareLaunchArgument('ui_timeout', default_value='0.75',
                              description='Arm gate/tuner heartbeat timeout in seconds; disarms until fresh W'),
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
            'lidar_port',
            default_value=VehicleDefault('lidar.port', '/dev/lidar'),
        ),
        DeclareLaunchArgument(
            'lidar_rotation',
            default_value=VehicleDefault('lidar.rotation_offset_deg', '180.0'),
        ),
        DeclareLaunchArgument('use_lidar', default_value='false'),
        DeclareLaunchArgument(
            'arduino_port',
            default_value=VehicleDefault('arduino.port', '/dev/arduino'),
        ),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument(
            'visualize',
            default_value='True',
            description='Publish/display debug images; disabled for minimum control latency',
        ),
        DeclareLaunchArgument('enable_visualization', default_value=LaunchConfiguration('visualize')),
        DeclareLaunchArgument('show_bev', default_value='true'),
        DeclareLaunchArgument(
            'profile',
            default_value='false',
            description='Collect and periodically report controller timing statistics',
        ),
        DeclareLaunchArgument(
            'debug_log',
            default_value='false',
            description='Print detailed per-frame controller state',
        ),
        DeclareLaunchArgument('steering_sign', default_value='1.0'),
        DeclareLaunchArgument(
            'calibration_tolerance',
            default_value=VehicleDefault('steering.calibration_tolerance', '35'),
        ),
        *tuning_launch_arguments(),
        OpaqueFunction(function=_launch_nodes),
    ])
