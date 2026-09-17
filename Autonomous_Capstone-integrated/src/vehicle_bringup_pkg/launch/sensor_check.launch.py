from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_device = LaunchConfiguration('camera_device')
    aux_camera_device = LaunchConfiguration('aux_camera_device')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_rotation = LaunchConfiguration('lidar_rotation')

    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0'),
        DeclareLaunchArgument('aux_camera_device', default_value='/dev/v4l/by-path/pci-0000:00:14.0-usb-0:4.1:1.0-video-index0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
        DeclareLaunchArgument('lidar_rotation', default_value='180.0'),

        Node(
            package='sensor_bringup_pkg',
            executable='camera_publisher_node',
            name='front_camera_publisher_node',
            output='screen',
            parameters=[{
                'device': camera_device,
                'topic': '/camera/front/image_raw',
                'frame_id': 'front_camera_frame',
                'width': 640,
                'height': 480,
                'fps': 30.0,
                'show': False,
            }],
        ),
        Node(
            package='sensor_bringup_pkg', executable='camera_publisher_node',
            name='aux_camera_publisher_node', output='screen',
            parameters=[{'device': aux_camera_device, 'topic': '/camera/aux/image_raw',
                         'frame_id': 'aux_camera_frame', 'width': 640, 'height': 480,
                         'fps': 30.0, 'show': False}],
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
    ])
