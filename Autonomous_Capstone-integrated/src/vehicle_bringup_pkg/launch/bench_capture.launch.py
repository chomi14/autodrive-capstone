"""Front-camera labeling bench with no Arduino serial bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_device = LaunchConfiguration('camera_device')
    output_dir = LaunchConfiguration('output_dir')
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0'),
        DeclareLaunchArgument(
            'output_dir', default_value='/home/autolab/autodrive_dataset/bench_test'),
        Node(package='sensor_bringup_pkg', executable='camera_publisher_node',
             name='front_camera_publisher_node', output='screen',
             parameters=[{'device': camera_device, 'topic': '/camera/front/image_raw',
                          'frame_id': 'front_camera_frame', 'show': False}]),
        Node(package='manual_drive_pkg', executable='manual_drive_capture_node',
             name='manual_drive_capture_node', output='screen', emulate_tty=True,
             parameters=[{'image_topic': '/camera/front/image_raw',
                          'cmd_topic': '/bench/topic_control_signal',
                          'arm_topic': '/bench/armed',
                          'ready_topic': '/bench/calibration_ready',
                          'output_dir': output_dir, 'dry_run': True,
                          'left_sign': -1, 'show_window': True}]),
    ])
