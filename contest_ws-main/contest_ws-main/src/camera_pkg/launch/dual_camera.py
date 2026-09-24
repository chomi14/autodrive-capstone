from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cam0', default_value='2', description='Camera device index for first webcam'),
        DeclareLaunchArgument('cam1', default_value='4', description='Camera device index for second webcam'),

        Node(
            package='camera_pkg',
            executable='image',
            name='camera0',
            namespace='cam0',
            parameters=[
                {'data_source': 'camera'},
                {'cam_num': LaunchConfiguration('cam0')},
                {'pub_topic': '/cam0/image_raw'},
                {'show_image': True},
                {'timer_period': 0.03}
            ]
        ),

        Node(
            package='camera_pkg',
            executable='image',
            name='camera1',
            namespace='cam1',
            parameters=[
                {'data_source': 'camera'},
                {'cam_num': LaunchConfiguration('cam1')},
                {'pub_topic': '/cam1/image_raw'},
                {'show_image': True},
                {'timer_period': 0.03}
            ]
        )
    ])

