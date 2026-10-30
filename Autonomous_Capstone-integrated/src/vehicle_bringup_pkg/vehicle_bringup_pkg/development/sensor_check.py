from vehicle_bringup_pkg.configuration import config_argument, VehicleDefault
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
        config_argument(),
        DeclareLaunchArgument('aux_camera_topic', default_value=VehicleDefault('camera.aux.topic', '/camera/aux/image_raw')),
        DeclareLaunchArgument('lidar_topic', default_value=VehicleDefault('lidar.topic', '/lidar_raw')),
        DeclareLaunchArgument('camera_topic', default_value=VehicleDefault('camera.front.topic', '/camera/front/image_raw')),
        DeclareLaunchArgument('camera_device', default_value=VehicleDefault('camera.front.device', '/dev/video0')),
        DeclareLaunchArgument('aux_camera_device', default_value=VehicleDefault('camera.aux.device', '/dev/video2')),
        DeclareLaunchArgument('lidar_port', default_value=VehicleDefault('lidar.port', '/dev/lidar')),
        DeclareLaunchArgument('lidar_rotation', default_value=VehicleDefault('lidar.rotation_offset_deg', '180.0')),

        Node(
            package='sensor_bringup_pkg',
            executable='camera_publisher_node',
            name='front_camera_publisher_node',
            output='screen',
            parameters=[{
                'device': camera_device,
                'topic': LaunchConfiguration('camera_topic'),
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
            parameters=[{'device': aux_camera_device, 'topic': LaunchConfiguration('aux_camera_topic'),
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
                'topic': LaunchConfiguration('lidar_topic'),
                'rotation_offset_deg': ParameterValue(lidar_rotation, value_type=float),
            }],
        ),
    ])
