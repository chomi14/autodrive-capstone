"""Real-track drive pipeline with a parameter-only OpenCV tuning panel."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from vehicle_bringup_pkg.configuration import config_argument, VehicleDefault
from vehicle_bringup_pkg.tuning_configuration import (
    resolve_tuning,
    tuning_launch_arguments,
)


# Track driving intentionally has no longitudinal-speed tuning.  While armed,
# both drive motors receive this same PWM command (safety stops still send 0).
FIXED_TRACK_SPEED = 250


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('1', 'true', 'yes', 'on'):
        return True
    if value in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false, got: {value}')


def _launch_nodes(context):
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
        'cmd_topic': 'topic_control_signal',
        'device': device,
        'start_enabled': True,
        'speed': FIXED_TRACK_SPEED,
        'allow_speed_tuning': False,
        'steering_sign': ParameterValue(steering_sign, value_type=float),
        'max_steering': 7.0,
        'publish_debug': enable_visualization,
        'publish_bev_debug': show_bev,
        'debug_log': debug_log,
        'profile': profile,
        'profile_input_fps': 30.0,
        'profile_report_interval': 100,
        # 640x480 BGR8 is about 0.92 MB per frame.  With BEST_EFFORT, losing
        # one DDS fragment discards the entire image; isolated measurements on
        # this host dropped receiver throughput from ~30 Hz to ~15 Hz.
        'image_reliability': 'reliable',
    })

    return [
        LogInfo(msg=f'Using {config_source} tuning config: {selected_config}'),
        Node(
            package='sensor_bringup_pkg',
            executable='camera_publisher_node',
            name='camera_publisher_node',
            output='screen',
            parameters=[{
                'device': camera_device,
                'topic': '/track/image_raw',
                'width': 640,
                'height': 480,
                'fps': 30.0,
                # Keep USB 2.0 transport compressed.  ROS still publishes BGR8,
                # so DDS reliability below remains necessary.
                'fourcc': 'MJPG',
                'buffer_size': 1,
                'reopen_after_failures': 2,
                'disable_dynamic_framerate': True,
                # Match the controller subscription.  Depth remains 1 in the
                # camera node so retransmission cannot build a frame backlog.
                'reliability': 'reliable',
                'show': False,
            }],
        ),
        Node(
            condition=IfCondition(use_lidar),
            package='sensor_bringup_pkg',
            executable='lidar_publisher_node_v2',
            name='lidar_publisher_node_v2',
            output='screen',
            parameters=[{
                'port': lidar_port,
                'topic': 'lidar_raw',
                'rotation_offset_deg': ParameterValue(lidar_rotation, value_type=float),
            }],
        ),
        Node(
            package='skku_track_drive_pkg',
            executable='track_controller_node',
            name='track_controller_node',
            output='screen',
            parameters=[controller_parameters],
        ),
        Node(
            package='skku_track_drive_pkg',
            executable='track_tuner_node',
            name='track_tuner_node',
            output='screen',
            parameters=[{
                'target_node': '/track_controller_node',
                'cmd_topic': '/topic_control_signal',
                'debug_topic': '/track_debug_image',
                'save_path': str(save_path),
                'loaded_config_path': str(selected_config),
                'allow_speed_tuning': False,
                'fixed_speed': FIXED_TRACK_SPEED,
            }],
        ),
        # This is the unchanged actuator safety chain. TrackTunerNode has no
        # serial/arm publisher and cannot bypass calibration_ready + armed.
        Node(
            package='vehicle_io_pkg',
            executable='serial_sender_node_v2',
            name='serial_sender_node_v2',
            output='screen',
            parameters=[{
                'port': arduino_port,
                'baud': ParameterValue(
                    LaunchConfiguration('arduino_baud'), value_type=int
                ),
                'topic': 'topic_control_signal',
                'arm_topic': 'vehicle/armed',
                'ready_topic': 'vehicle/calibration_ready',
                'auto_calibrate': ParameterValue(
                    LaunchConfiguration('auto_calibrate'), value_type=bool
                ),
                'calibration_tolerance': ParameterValue(
                    cal_tolerance, value_type=int
                ),
            }],
        ),
        Node(
            package='vehicle_io_pkg',
            executable='drive_arm_node',
            name='drive_arm_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'arm_topic': 'vehicle/armed',
                'ready_topic': 'vehicle/calibration_ready',
                'status_topic': 'vehicle/calibration_status',
                'show_window': True,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        config_argument(),
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
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument(
            'arduino_port',
            default_value=VehicleDefault('arduino.port', '/dev/arduino'),
        ),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Publish/display debug images; disabled for minimum control latency',
        ),
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
