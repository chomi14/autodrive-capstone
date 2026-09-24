# dual_yolo_obj.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
import os

def generate_launch_description():
    # 현재 launch 파일의 위치를 기준으로 모델 파일 경로 설정
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(launch_dir, '..', 'camera_pkg', 'model', 'yolo11n.pt')
    
    return LaunchDescription([
        DeclareLaunchArgument('input0', default_value='/cam0/image_raw', description='Input for YOLO object 0'),
        DeclareLaunchArgument('input1', default_value='/cam1/image_raw', description='Input for YOLO object 1'),

        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),

        Node(
            package='camera_pkg',
            executable='yolo_obj',
            name='yolo_obj0',
            namespace='cam0',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': model_path},
                {'threshold': 0.5},
                {'enable': True},
                {'show_result': True}
            ],
            remappings=[
                ('/image_raw', LaunchConfiguration('input0'))
            ]
        ),

        Node(
            package='camera_pkg',
            executable='yolo_obj',
            name='yolo_obj1',
            namespace='cam1',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': model_path},
                {'threshold': 0.5},
                {'enable': True},
                {'show_result': True}
            ],
            remappings=[
                ('/image_raw', LaunchConfiguration('input1'))
            ]
        )
    ])


