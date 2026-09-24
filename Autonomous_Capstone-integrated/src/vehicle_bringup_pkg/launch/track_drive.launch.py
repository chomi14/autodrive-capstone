from vehicle_bringup_pkg.configuration import config_argument, VehicleDefault
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
    device = LaunchConfiguration('device')
    speed = LaunchConfiguration('speed')
    steering_sign = LaunchConfiguration('steering_sign')
    cal_tolerance = LaunchConfiguration('calibration_tolerance')

    return LaunchDescription([
        config_argument(),
        DeclareLaunchArgument('auto_calibrate', default_value=VehicleDefault('steering.auto_calibrate', False)),
        DeclareLaunchArgument('arduino_baud', default_value=VehicleDefault('arduino.baud', 115200)),
        DeclareLaunchArgument('camera_device', default_value=VehicleDefault('camera.front.device', '/dev/video0')),
        DeclareLaunchArgument('lidar_port', default_value=VehicleDefault('lidar.port', '/dev/lidar')),
        DeclareLaunchArgument('lidar_rotation', default_value=VehicleDefault('lidar.rotation_offset_deg', '180.0')),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('arduino_port', default_value=VehicleDefault('arduino.port', '/dev/arduino')),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('speed', default_value='80'),
        DeclareLaunchArgument('steering_sign', default_value='1.0'),
        DeclareLaunchArgument('calibration_tolerance', default_value=VehicleDefault('steering.calibration_tolerance', '35')),

        # All sensors are started with the run.  LiDAR is optional only so the
        # track code can still be debugged on a bench without the sensor attached.
        Node(
            package='sensor_bringup_pkg', executable='camera_publisher_node',
            name='camera_publisher_node', output='screen',
            parameters=[{
                'device': camera_device,
                'topic': 'image_raw',
                'width': 640,
                'height': 480,
                'fps': 30.0,
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

        # Controller is internally active immediately, but the serial bridge
        # blocks all motion until calibration is READY and the operator presses W.
        Node(
            package='skku_track_drive_pkg', executable='track_controller_node',
            name='track_controller_node', output='screen',
            parameters=[{
                'image_topic': 'image_raw',
                'cmd_topic': 'topic_control_signal',
                'device': device,
                'speed': ParameterValue(speed, value_type=int),
                'start_enabled': True,
                'steering_sign': ParameterValue(steering_sign, value_type=float),
                'max_steering': 7.0,
                'publish_debug': True,
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
            package='vehicle_io_pkg', executable='drive_arm_node',
            name='drive_arm_node', output='screen', emulate_tty=True,
            parameters=[{
                'arm_topic': 'vehicle/armed',
                'ready_topic': 'vehicle/calibration_ready',
                'status_topic': 'vehicle/calibration_status',
                'show_window': True,
            }],
        ),
    ])
