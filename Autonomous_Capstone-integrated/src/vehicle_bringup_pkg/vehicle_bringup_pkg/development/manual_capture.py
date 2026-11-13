from vehicle_bringup_pkg.configuration import config_argument, VehicleDefault
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_cameras(context):
    if not IfCondition(LaunchConfiguration('use_rear_camera')).evaluate(context):
        return []
    devices = [os.path.realpath(LaunchConfiguration(name).perform(context))
               for name in ('camera_device', 'rear_camera_device')]
    if devices[0] == devices[1] or (
        all(os.path.exists(path) for path in devices) and os.path.samefile(*devices)
    ):
        raise ValueError('Front/rear cameras must use different devices')
    return []


def generate_launch_description():
    camera_device = LaunchConfiguration('camera_device')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_rotation = LaunchConfiguration('lidar_rotation')
    use_lidar = LaunchConfiguration('use_lidar')
    arduino_port = LaunchConfiguration('arduino_port')
    output_dir = LaunchConfiguration('output_dir')
    speed_step = LaunchConfiguration('speed_step')
    steering_step = LaunchConfiguration('steering_step')
    left_sign = LaunchConfiguration('left_sign')
    auto_interval = LaunchConfiguration('auto_interval')
    cal_tolerance = LaunchConfiguration('calibration_tolerance')

    return LaunchDescription([
        config_argument(),
        DeclareLaunchArgument('auto_calibrate', default_value=VehicleDefault('steering.auto_calibrate', False)),
        DeclareLaunchArgument('arduino_baud', default_value=VehicleDefault('arduino.baud', 115200)),
        DeclareLaunchArgument('camera_device', default_value=VehicleDefault('camera.front.device', '/dev/video0')),
        DeclareLaunchArgument('use_rear_camera', default_value='false'),
        DeclareLaunchArgument('rear_camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_fourcc', default_value='MJPG'),
        DeclareLaunchArgument('auto_capture', default_value='false'),
        DeclareLaunchArgument('max_pair_skew', default_value='0.10'),
        DeclareLaunchArgument('max_frame_age', default_value='0.50'),
        DeclareLaunchArgument('lidar_port', default_value=VehicleDefault('lidar.port', '/dev/lidar')),
        DeclareLaunchArgument('lidar_rotation', default_value=VehicleDefault('lidar.rotation_offset_deg', '180.0')),
        DeclareLaunchArgument('use_lidar', default_value='false'),
        DeclareLaunchArgument('arduino_port', default_value=VehicleDefault('arduino.port', '/dev/arduino')),
        DeclareLaunchArgument('output_dir', default_value='~/autodrive_dataset/manual_drive'),
        DeclareLaunchArgument('speed_step', default_value='20'),
        DeclareLaunchArgument('steering_step', default_value='1'),
        DeclareLaunchArgument('left_sign', default_value='-1'),
        DeclareLaunchArgument('auto_interval', default_value='0.20'),
        DeclareLaunchArgument('calibration_tolerance', default_value=VehicleDefault('steering.calibration_tolerance', '35')),

        OpaqueFunction(function=validate_cameras),
        Node(
            package='sensor_bringup_pkg', executable='camera_publisher_node',
            name='camera_publisher_node', output='screen',
            parameters=[{
                'device': camera_device,
                'topic': 'image_raw',
                'fourcc': LaunchConfiguration('camera_fourcc'),
                'frame_id': 'manual_front_camera',
                'show': False,
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rear_camera')),
            package='sensor_bringup_pkg', executable='camera_publisher_node',
            name='rear_camera_publisher_node', output='screen',
            parameters=[{
                'device': LaunchConfiguration('rear_camera_device'),
                'topic': 'manual/rear/image_raw',
                'fourcc': LaunchConfiguration('camera_fourcc'),
                'frame_id': 'manual_rear_camera',
                'show': False,
            }],
        ),
        Node(
            condition=IfCondition(use_lidar),
            package='sensor_bringup_pkg', executable='lidar_publisher_node_v2',
            name='lidar_publisher_node_v2', output='screen',
            parameters=[{
                'port': lidar_port,
                'topic': 'lidar_raw',
                'rotation_offset_deg': ParameterValue(lidar_rotation, value_type=float),
            }],
        ),
        Node(
            package='vehicle_io_pkg', executable='serial_sender_node_v2',
            name='serial_sender_node_v2', output='screen',
            parameters=[{
                'port': arduino_port,
                'baud': ParameterValue(LaunchConfiguration('arduino_baud'), value_type=int),
                'topic': 'topic_control_signal',
                'arm_topic': 'vehicle/armed',
                'ready_topic': 'vehicle/calibration_ready',
                'auto_calibrate': ParameterValue(LaunchConfiguration('auto_calibrate'), value_type=bool),
                'calibration_tolerance': ParameterValue(cal_tolerance, value_type=int),
            }],
        ),
        Node(
            package='manual_drive_pkg', executable='manual_drive_capture_node',
            name='manual_drive_capture_node', output='screen', emulate_tty=True,
            parameters=[{
                'image_topic': 'image_raw',
                'use_rear_camera': ParameterValue(LaunchConfiguration('use_rear_camera'), value_type=bool),
                'rear_image_topic': 'manual/rear/image_raw',
                'auto_capture': ParameterValue(LaunchConfiguration('auto_capture'), value_type=bool),
                'max_pair_skew': ParameterValue(LaunchConfiguration('max_pair_skew'), value_type=float),
                'max_frame_age': ParameterValue(LaunchConfiguration('max_frame_age'), value_type=float),
                'cmd_topic': 'topic_control_signal',
                'arm_topic': 'vehicle/armed',
                'ready_topic': 'vehicle/calibration_ready',
                'output_dir': output_dir,
                'speed_step': ParameterValue(speed_step, value_type=int),
                'steering_step': ParameterValue(steering_step, value_type=int),
                'left_sign': ParameterValue(left_sign, value_type=int),
                'auto_capture_interval': ParameterValue(auto_interval, value_type=float),
            }],
        ),
    ])
