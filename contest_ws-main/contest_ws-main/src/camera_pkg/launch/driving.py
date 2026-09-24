import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    # 현재 launch 파일의 위치를 기준으로 모델 파일 경로 설정
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(launch_dir, '..', 'camera_pkg', 'model', 'best.pt')
    
    return LaunchDescription([
        #################### CAMERA ######################
        Node(
            package='camera_pkg',
            executable='image',  # 실행할 노드 파일명
            name='camera0',
            namespace='cam0',
            parameters=[
                {'data_source': 'image'},  # camera, video, image 선택
                {'cam_num': 2},
                {'pub_topic': '/cam0/image_raw'},
                {'window_name': 'Raw 0'},
                {'show_image': False},
                {'timer_period': 0.03},  # float형으로, fps조절 
            ]
        ),
        
        #################### YOLO SEG ######################
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),
        Node(
            package='camera_pkg',
            executable='yolo_seg',  # YOLO Segmentation 노드 실행
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
        
        #################### TRAFFIC LIGHT ######################
        Node(
            package='camera_pkg',
            executable='traffic_light',
            name='traffic_light',
            output='screen',
            parameters=[]
        ),
        
        #################### LANE DETECT ######################
        Node(
            package='camera_pkg',
            executable='lane',  # 차선 감지 노드
            name='lane_detector_cam0',
            namespace='cam0',
            parameters=[
                {'camera_topic': '/cam0/image_raw'},
                {'detection_topic': '/cam0/detections'}
            ],
            output='screen'
        ),

        #################### MOTION ######################
        Node(
            package='decision_making_pkg',  # motion 패키지
            executable='motion',  # 실행할 motion 노드
            name='motion_node',
            output='screen'
        ),
        
        #################### CONTROL ######################
        Node(
            package='control_pkg',  # control 패키지
            executable='control',  # 실행할 control 노드
            name='control_node',
            output='screen'
        )
    ])

