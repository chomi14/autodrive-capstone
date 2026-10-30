from vehicle_bringup_pkg.configuration import config_argument, VehicleDefault
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_device = LaunchConfiguration('camera_device')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_rotation = LaunchConfiguration('lidar_rotation')
    arduino_port = LaunchConfiguration('arduino_port')
    cal_tolerance = LaunchConfiguration('calibration_tolerance')

    return LaunchDescription([
        config_argument(),
        DeclareLaunchArgument('auto_calibrate', default_value=VehicleDefault('steering.auto_calibrate', False)),
        DeclareLaunchArgument('arduino_baud', default_value=VehicleDefault('arduino.baud', 115200)),
        DeclareLaunchArgument('camera_device', default_value=VehicleDefault('camera.front.device', '/dev/video0')),
        DeclareLaunchArgument('lidar_port', default_value=VehicleDefault('lidar.port', '/dev/lidar')),
        DeclareLaunchArgument('lidar_rotation', default_value=VehicleDefault('lidar.rotation_offset_deg', '180.0')),
        DeclareLaunchArgument('arduino_port', default_value=VehicleDefault('arduino.port', '/dev/arduino')),
        DeclareLaunchArgument('calibration_tolerance', default_value=VehicleDefault('steering.calibration_tolerance', '35')),

        # -------- Sensors --------
        Node(
            package='sensor_bringup_pkg',
            executable='camera_publisher_node',
            name='camera_publisher_node',
            output='screen',
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

        # -------- Camera perception --------
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='yolov8_node',
            output='screen',
        ),
        Node(
            package='camera_perception_pkg',
            executable='traffic_light_detector_node',
            name='traffic_light_detector_node',
            output='screen',
        ),
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='lane_info_extractor_node',
            output='screen',
        ),

        # -------- LiDAR perception --------
        Node(
            package='lidar_perception_pkg',
            executable='lidar_processor_node',
            name='lidar_processor_node',
            output='screen',
        ),
        Node(
            package='lidar_perception_pkg',
            executable='lidar_obstacle_detector_node',
            name='lidar_obstacle_detector_node',
            output='screen',
        ),

        # -------- Mission / path / motion --------
        Node(
            package='decision_making_pkg',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
        ),
        Node(
            package='decision_making_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
        ),
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='motion_planner_node',
            output='screen',
        ),

        # -------- Runtime steering calibration + operator W start gate --------
        Node(
            package='vehicle_io_pkg',
            executable='serial_sender_node_v2',
            name='serial_sender_node_v2',
            output='screen',
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
    ])
