import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
     # 각 카메라에 맞는 모델 경로
    cam0_model_path = "/home/sg/contest_ws/src/camera_pkg/camera_pkg/model/final.pt"
    cam1_model_path = "/home/sg/contest_ws/src/camera_pkg/camera_pkg/model/final.pt"

    return LaunchDescription([
        
        #################### Static TF ######################
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_link',
            arguments=[
                '0', '0', '0',    # x, y, z 오프셋 (m)
                '0', '0', '0',    # roll, pitch, yaw (rad)
                'base_link',      # 부모 프레임
                'laser_frame'     # 자식 프레임
            ]
        ),


        #################### Lidar ######################
        Node(
                package='rplidar_ros',
                executable='rplidar_composition',
                name='rplidar',
                output='screen',
                parameters=[
                    {'serial_port': '/dev/ttyUSB0'},
                    {'serial_baudrate': 115200},  # A1 모델은 115200, A2/S1은 256000
                    {'frame_id': 'laser_frame'},
                    {'inverted': False},
                    {'angle_compensate': True}
                ]
            ),

        
        #################### CAMERA1(LANE) ######################
        
        Node(
            package='camera_pkg',
            executable='image',  # 실행할 노드 파일명
            name='cam0',
            namespace='cam0',
            parameters=[
                {'data_source': 'camera'},  # camera, video, image 선택
                {'cam_num': 2},
                {'img_dir': '/home/sg/contest_ws/src/camera_pkg/camera_pkg/lib/image'},
                {'pub_topic': '/cam0/image_raw'},
                {'window_name': 'Raw 0'},
                {'show_image': False},
                {'timer_period': 0.03},  # float형으로, fps조절 
            ]
        ),

        #################### CAMERA2(Traffic_light) ######################
        
        Node(
            package='camera_pkg',
            executable='image',  # 실행할 노드 파일명
            name='cam1',
            namespace='cam1',
            parameters=[
                {'data_source': 'camera'},  # camera, video, image 선택
                {'cam_num': 4},
                {'img_dir': '/home/sg/contest_ws/src/camera_pkg/camera_pkg/lib/traffic_light'},
                {'pub_topic': '/cam1/image_raw'},
                {'window_name': 'Raw 0'},
                {'show_image': False},
                {'timer_period': 0.03},  # float형으로, fps조절 
            ]
        ),
        
        #################### CAMERA1 YOLO SEG ######################
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),
        Node(
            package='camera_pkg',
            executable='yolo_seg',  
            name='yolo_seg0',
            namespace='cam0',
            output='screen',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': cam0_model_path},
                {'threshold': 0.5}
            ],
            remappings=[
                ('image_raw', '/cam0/image_raw'),  # 카메라의 image_raw 토픽과 매핑
                ('detections', '/cam0/detections')  # YOLO가 감지한 결과를 detections에 퍼블리시
            ]
        ),

        #################### CAMERA2 YOLO SEG ######################
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),
        Node(
            package='camera_pkg',
            executable='yolo_seg',  
            name='yolo_seg1',
            namespace='cam1',
            output='screen',
            parameters=[
                {'device': 'cuda:0'},
                {'model_path': cam1_model_path},
                {'threshold': 0.5}
            ],
            remappings=[
                ('image_raw', '/cam1/image_raw'),  # 카메라의 image_raw 토픽과 매핑
                ('detections', '/cam1/detections')  # YOLO가 감지한 결과를 detections에 퍼블리시
            ]
        ),



        #################### LANE DETECT ######################
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

        #################### TRAFFIC LIGHT DETECT ######################
        Node(
            package='camera_pkg',
            executable='traffic_light',
            name='traffic_light',
            output='screen',
            parameters=[]
        ),

        #################### OBSTACLE DETECT ######################
        Node(
            package='camera_pkg',
            executable='obstacle',
            name='obstacle',
            output='screen',
            parameters=[]
        ),
        
        
        #################### MOTION ######################
        Node(
            package='decision_making_pkg',  
            executable='motion_mission',  
            name='motion_mission',
            output='screen'
        ),
        
        #################### CONTROL ######################
        # Node(
        #     package='control_pkg',  
        #     executable='control',  
        #     name='control_node',
        #     output='screen'
        # )
        
    ])

