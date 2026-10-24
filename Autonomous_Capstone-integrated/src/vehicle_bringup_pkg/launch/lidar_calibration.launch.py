"""LiDAR + calibration GUI only; no motor/serial command nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from vehicle_bringup_pkg.mode_launch import lidar_node
from skku_track_drive_pkg.lidar_camera_fusion import CALIBRATION_PATH, SCAN_TOPIC

# 센서 장치와 upstream 기본 회전. 나머지 조정값은 lidar_camera_fusion.py.
LIDAR_PORT = '/dev/lidar'
DRIVER_ROTATION_DEG = '180.0'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT),
        DeclareLaunchArgument('lidar_rotation', default_value=DRIVER_ROTATION_DEG),
        DeclareLaunchArgument('calibration_path', default_value=CALIBRATION_PATH),
        lidar_node(SCAN_TOPIC),
        Node(package='skku_track_drive_pkg', executable='lidar_calibration_node', output='screen',
             parameters=[{'calibration_path': LaunchConfiguration('calibration_path')}]),
    ])
