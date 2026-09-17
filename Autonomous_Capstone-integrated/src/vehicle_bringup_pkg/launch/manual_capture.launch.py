from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),
        DeclareLaunchArgument('lidar_rotation', default_value='180.0'),
        DeclareLaunchArgument('use_lidar', default_value='false'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/arduino'),
        DeclareLaunchArgument('output_dir', default_value='~/ros2_ws/datasets/manual_drive'),
        DeclareLaunchArgument('speed_step', default_value='20'),
        DeclareLaunchArgument('steering_step', default_value='1'),
        DeclareLaunchArgument('left_sign', default_value='-1'),
        DeclareLaunchArgument('auto_interval', default_value='0.20'),
        DeclareLaunchArgument('calibration_tolerance', default_value='35'),

        Node(
            package='sensor_bringup_pkg', executable='camera_publisher_node',
            name='camera_publisher_node', output='screen',
            parameters=[{
                'device': camera_device,
                'topic': 'image_raw',
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
                'topic': 'topic_control_signal',
                'arm_topic': 'vehicle/armed',
                'ready_topic': 'vehicle/calibration_ready',
                'auto_calibrate': True,
                'calibration_tolerance': ParameterValue(cal_tolerance, value_type=int),
            }],
        ),
        Node(
            package='manual_drive_pkg', executable='manual_drive_capture_node',
            name='manual_drive_capture_node', output='screen', emulate_tty=True,
            parameters=[{
                'image_topic': 'image_raw',
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
