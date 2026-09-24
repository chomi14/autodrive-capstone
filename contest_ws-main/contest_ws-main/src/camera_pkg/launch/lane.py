from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        #################### CAMERA ######################
        DeclareLaunchArgument('cam0', default_value='2', description='Camera device index for first webcam'),
        DeclareLaunchArgument('cam1', default_value='4', description='Camera device index for second webcam'),
        DeclareLaunchArgument('window_name_cam0', default_value='Lane - 0', description='Window name for first webcam'),
        DeclareLaunchArgument('window_name_cam1', default_value='Lane - 1', description='Window name for second webcam'),

        Node(
            package='camera_pkg',
            executable='lane',
            name='lane_detector_cam0',
            namespace='cam0',
            parameters=[
                {'camera_topic': '/cam0/image_raw'},
                {'detection_topic': '/cam0/detections'}
            ],
            output='screen'
        ),

        Node(
            package='camera_pkg',
            executable='lane',
            name='lane_detector_cam1',
            namespace='cam1',
            parameters=[
                {'camera_topic': '/cam1/image_raw'},
                {'detection_topic': '/cam1/detections'}
            ],
            output='screen'
        ),
    ])

