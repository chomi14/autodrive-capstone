from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',  # 실제 포트에 맞게 조정
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        ),
    ])
