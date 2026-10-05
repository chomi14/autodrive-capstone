"""Common sensor and gated serial launch components for all four modes."""
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from .configuration import VehicleDefault


def common_arguments(command_timeout='0.75', camera=True):
    arguments = [
        DeclareLaunchArgument('dry_run', default_value='false', description='Disable hardware sensors, serial and arm gate; controllers/GUI remain available'),
        DeclareLaunchArgument('sensors_only', default_value='false', description='Run actual sensors and GUI without vehicle serial/arm gate'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('command_timeout', default_value=command_timeout,
                             description='Independent controller health lease; 0 uses ui_timeout, never inference age'),
        DeclareLaunchArgument('ui_timeout', default_value='0.75'),
        DeclareLaunchArgument('auto_calibrate', default_value=VehicleDefault('steering.auto_calibrate', False)),
        DeclareLaunchArgument('arduino_baud', default_value=VehicleDefault('arduino.baud', 115200)),
        DeclareLaunchArgument('arduino_port', default_value=VehicleDefault('arduino.port', '/dev/arduino')),
        DeclareLaunchArgument('calibration_tolerance', default_value=VehicleDefault('steering.calibration_tolerance', 35)),
        DeclareLaunchArgument('lidar_port', default_value=VehicleDefault('lidar.port', '/dev/lidar')),
        DeclareLaunchArgument('lidar_rotation', default_value=VehicleDefault('lidar.rotation_offset_deg', 180.0)),
    ]
    from .recording import recording_arguments
    arguments.extend(recording_arguments())
    # Keep camera_device available for tooling even in LiDAR-only modes.
    arguments.append(DeclareLaunchArgument('camera_device', default_value=VehicleDefault('camera.front.device', '/dev/video0')))
    if camera:
        arguments.append(DeclareLaunchArgument('device', default_value='cuda:0'))
    return arguments


def hardware_enabled(context):
    return LaunchConfiguration('dry_run', default='false').perform(context).lower() in ('false', '0')


def as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false', '1', '0'):
        raise ValueError(f'{name}: expected true or false')
    return value in ('true', '1')


def command_topic(context):
    if not hardware_enabled(context):
        return '/dry_run/topic_control_signal'
    return 'topic_control_signal' if motor_enabled(context) else '/sensors_only/topic_control_signal'


def motor_enabled(context):
    return hardware_enabled(context) and LaunchConfiguration('sensors_only', default='false').perform(context).lower() in ('false', '0')


def preview_remappings(context):
    if motor_enabled(context):
        return []
    prefix = '/sensors_only' if hardware_enabled(context) else '/dry_run'
    # A preview GUI must not arm/heartbeat another live sender on this domain.
    return [(f'vehicle/{name}', f'{prefix}/vehicle/{name}') for name in
            ('drive_key', 'tuner_heartbeat', 'controller_heartbeat', 'command_status', 'arm_challenge', 'drive_state')]


def vehicle_remappings(prefix):
    return [(f'vehicle/{name}', f'{prefix}/vehicle/{name}') for name in
            ('armed', 'calibration_ready', 'calibration_status', 'drive_state',
             'drive_key', 'ui_heartbeat', 'tuner_heartbeat', 'controller_heartbeat', 'command_status', 'arm_challenge', 'arm_request')]


def camera_node(topic):
    return Node(
        package='sensor_bringup_pkg', executable='camera_publisher_node', name='camera_publisher_node', output='screen',
        parameters=[{'device': LaunchConfiguration('camera_device'), 'topic': topic,
                     'width': 640, 'height': 480, 'fps': 30.0, 'fourcc': 'MJPG',
                     'buffer_size': 1, 'reopen_after_failures': 2,
                     'disable_dynamic_framerate': True, 'reliability': 'reliable', 'show': False}],
    )


def lidar_node(topic):
    return Node(
        package='sensor_bringup_pkg', executable='lidar_publisher_node_v2', name='lidar_publisher_node_v2', output='screen',
        parameters=[{'port': LaunchConfiguration('lidar_port'), 'topic': topic,
                     'rotation_offset_deg': ParameterValue(LaunchConfiguration('lidar_rotation'), value_type=float)}],
    )


def vehicle_io_nodes(topic='topic_control_signal', remappings=None, perception_mode=None):
    from .perception_policy import measured_policy
    policy = measured_policy(perception_mode) if perception_mode else {}
    perception = {'require_perception_progress': bool(perception_mode), **{
        name: ParameterValue(LaunchConfiguration(name, default=str(value)), value_type=float)
        for name, value in policy.items()}}
    return [
        Node(package='vehicle_io_pkg', executable='serial_sender_node_v2', name='serial_sender_node_v2', output='screen',
             remappings=remappings or [],
             parameters=[{'port': LaunchConfiguration('arduino_port'),
                          'baud': ParameterValue(LaunchConfiguration('arduino_baud'), value_type=int),
                          'topic': topic, 'arm_topic': 'vehicle/armed',
                          'ready_topic': 'vehicle/calibration_ready',
                          'command_timeout': ParameterValue(LaunchConfiguration('command_timeout'), value_type=float),
                          'ui_timeout': ParameterValue(LaunchConfiguration('ui_timeout'), value_type=float),
                          'require_tuner_heartbeat': True,
                          'require_controller_heartbeat': True,
                          **perception,
                          'auto_calibrate': ParameterValue(LaunchConfiguration('auto_calibrate'), value_type=bool),
                          'calibration_tolerance': ParameterValue(LaunchConfiguration('calibration_tolerance'), value_type=int)}]),
        Node(package='vehicle_io_pkg', executable='drive_arm_node', name='drive_arm_node', output='screen', emulate_tty=True,
             remappings=remappings or [],
             parameters=[{'arm_topic': 'vehicle/armed', 'ready_topic': 'vehicle/calibration_ready',
                          'status_topic': 'vehicle/calibration_status', 'show_window': True}]),
    ]
