"""Run contest perception with the canonical vehicle safety/serial path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('camera_pkg')
    default_model = os.path.join(share_dir, 'model', 'final.pt')
    default_config = os.path.join(share_dir, 'config', 'contest_vehicle.yaml')

    front_camera = LaunchConfiguration('front_camera')
    aux_camera = LaunchConfiguration('aux_camera')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    use_aux = LaunchConfiguration('use_aux')
    show_image = LaunchConfiguration('show_image')
    publish_debug = LaunchConfiguration('publish_debug')
    max_speed = LaunchConfiguration('max_speed')
    steering_sign = LaunchConfiguration('steering_sign')
    arduino_port = LaunchConfiguration('arduino_port')
    arduino_baud = LaunchConfiguration('arduino_baud')
    auto_calibrate = LaunchConfiguration('auto_calibrate')

    return LaunchDescription([
        DeclareLaunchArgument('front_camera', default_value='/dev/video2'),
        DeclareLaunchArgument('aux_camera', default_value='/dev/video4'),
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('use_aux', default_value='false'),
        DeclareLaunchArgument('show_image', default_value='false'),
        DeclareLaunchArgument('publish_debug', default_value='false'),
        DeclareLaunchArgument('max_speed', default_value='100'),
        DeclareLaunchArgument('steering_sign', default_value='1'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/arduino'),
        DeclareLaunchArgument('arduino_baud', default_value='115200'),
        DeclareLaunchArgument('auto_calibrate', default_value='false'),

        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),

        Node(
            package='camera_pkg',
            executable='image',
            name='image',
            namespace='cam0',
            output='screen',
            parameters=[{
                'data_source': 'camera',
                'camera_device': front_camera,
                'pub_topic': 'image_raw',
                'show_image': ParameterValue(show_image, value_type=bool),
                'timer_period': 0.03,
            }],
        ),
        Node(
            package='camera_pkg',
            executable='yolo_seg',
            name='yolo_seg',
            namespace='cam0',
            output='screen',
            parameters=[{
                'device': device,
                'model_path': model_path,
                'threshold': 0.5,
            }],
        ),
        Node(
            package='camera_pkg',
            executable='lane',
            name='lane_detector',
            namespace='cam0',
            output='screen',
            parameters=[{
                'camera_topic': 'image_raw',
                'detection_topic': 'detections',
                'publish_debug': ParameterValue(publish_debug, value_type=bool),
                'publish_viz': ParameterValue(publish_debug, value_type=bool),
            }],
        ),

        # The second segmentation pipeline is optional because two YOLO model
        # instances can materially increase latency on a small GPU.
        Node(
            condition=IfCondition(use_aux),
            package='camera_pkg',
            executable='image',
            name='image',
            namespace='cam1',
            output='screen',
            parameters=[{
                'data_source': 'camera',
                'camera_device': aux_camera,
                'pub_topic': 'image_raw',
                'show_image': ParameterValue(show_image, value_type=bool),
                'timer_period': 0.05,
            }],
        ),
        Node(
            condition=IfCondition(use_aux),
            package='camera_pkg',
            executable='yolo_seg',
            name='yolo_seg',
            namespace='cam1',
            output='screen',
            parameters=[{
                'device': device,
                'model_path': model_path,
                'threshold': 0.5,
            }],
        ),
        Node(
            condition=IfCondition(use_aux),
            package='camera_pkg',
            executable='traffic_light',
            name='traffic_light',
            output='screen',
            parameters=[{'publish_mask': ParameterValue(publish_debug, value_type=bool)}],
        ),

        Node(
            package='decision_making_pkg',
            executable='motion',
            name='contest_motion',
            output='screen',
            remappings=[('motion_command', '/contest/motion_command')],
        ),
        Node(
            package='control_pkg',
            executable='motion_command_guard',
            name='contest_motion_command_guard',
            output='screen',
            parameters=[default_config, {
                'max_speed': ParameterValue(max_speed, value_type=int),
                'steering_sign': ParameterValue(steering_sign, value_type=int),
            }],
        ),

        # These two nodes come from Autonomous_Capstone-integrated.  They keep
        # motor output stopped until READY and an operator presses W to arm.
        Node(
            package='vehicle_io_pkg',
            executable='serial_sender_node_v2',
            name='serial_sender_node_v2',
            output='screen',
            parameters=[default_config, {
                'port': arduino_port,
                'baud': ParameterValue(arduino_baud, value_type=int),
                'auto_calibrate': ParameterValue(auto_calibrate, value_type=bool),
            }],
        ),
        Node(
            package='vehicle_io_pkg',
            executable='drive_arm_node',
            name='drive_arm_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'arm_topic': '/vehicle/armed',
                'ready_topic': '/vehicle/calibration_ready',
                'status_topic': '/vehicle/calibration_status',
                'show_window': True,
            }],
        ),
    ])
