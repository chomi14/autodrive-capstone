from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
import os

def generate_launch_description():
    # 현재 launch 파일의 위치를 기준으로 모델 파일 경로 설정
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(launch_dir, '..', 'camera_pkg', 'model', 'best.pt')
    
    return LaunchDescription([
        
        
        #################### CAMERA ######################
        Node(
            package='camera_pkg',
            executable='image',
            name='camera0',
            namespace='cam0',
            parameters=[
                {'data_source': 'image'}, #camera, video, image 선택
                {'cam_num': 2},  # DeclareLaunchArgument 'cam0'에서 전달된 값을 여기 사용
                {'pub_topic': '/cam0/image_raw'},
                {'window_name': 'Raw 0'},  # 카메라 0의 창 이름
                {'show_image': False},
                {'timer_period': 0.03},  #프레임 0.03 or 1.0
            ]
        ),

        #################### YOLO SEG ######################

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
                ('image_raw', '/cam0/image_raw'),  # 카메라의 image_raw 토픽과 매핑
                ('detections', '/cam0/detections')  # YOLO가 감지한 결과를 detections에 퍼블리시
            ]
        ),

        #################### traffic light ######################
        Node(
            package='camera_pkg',  # 패키지 이름
            executable='traffic_light',  
            name='traffic_light',  # 노드 이름
            output='screen',
            parameters=[] 
        ),
        
        #################### Land detect ######################
        
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
    ])

