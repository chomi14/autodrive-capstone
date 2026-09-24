# dual_yolo_seg.py (Node 버전)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 현재 launch 파일의 위치를 기준으로 모델 파일 경로 설정
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(launch_dir, '..', 'camera_pkg', 'model', 'best.pt')
    
    return LaunchDescription([
        DeclareLaunchArgument('input0', default_value='/cam0/image_raw', description='Input for YOLO seg 0'),
        DeclareLaunchArgument('input1', default_value='/cam1/image_raw', description='Input for YOLO seg 1'),

        # GPU 환경 설정
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),

        Node(
            package='camera_pkg',
            executable='yolo_seg',
            name='yolo_seg0',
            namespace='cam0',
            output='screen',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': model_path},
                {'threshold': 0.5}
            ],
            remappings=[
                ('/image_raw', LaunchConfiguration('input0')),
                ('detections', 'detections')  # => /cam0/detections
            ]
        ),

        Node(
            package='camera_pkg',
            executable='yolo_seg',
            name='yolo_seg1',
            namespace='cam1',
            output='screen',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': model_path},
                {'threshold': 0.5}
            ],
            remappings=[
                ('/image_raw', LaunchConfiguration('input1')),
                ('detections', 'detections')  # => /cam1/detections
            ]
        )
    ])


