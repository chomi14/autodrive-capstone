# ROS Interface Contract

기준 namespace `/`. 아래는 소스/launch 정적 조사 결과이며 실제 차량의 live graph를
조회하지 않았다. 상대 topic은 namespace/remap에 따라 바뀔 수 있다.
canonical package는 센서 `sensor_bringup_pkg`, I/O `vehicle_io_pkg`, 수동 `manual_drive_pkg`,
트랙 `skku_track_drive_pkg`, launch `vehicle_bringup_pkg`, 메시지 `interfaces_pkg`이다.
미션 perception/planning은 기존 package를 유지한다.

## 공식 인터페이스

| Publisher | Topic | Message Type | Subscriber | Purpose |
|---|---|---|---|---|
| sensor_bringup camera (front) | /camera/front/image_raw | sensor_msgs/msg/Image | bench capture; 향후 track/manual/mission remap | 전방 BGR8 이미지, front_camera_frame |
| sensor_bringup camera (aux) | /camera/aux/image_raw | sensor_msgs/msg/Image | 현재 내장 소비자 없음; rqt/향후 dataset | 보조 BGR8 이미지, aux_camera_frame |
| sensor_bringup lidar | /lidar_raw | sensor_msgs/msg/LaserScan | lidar_processor, 외부 RViz | laser_frame, rotation_offset_deg=180 기본 |
| manual capture 또는 track controller 또는 mission motion planner | /topic_control_signal | interfaces_pkg/msg/MotionCommand | vehicle_io serial sender | 공식 차량 명령, 단일 publisher 원칙 |
| drive_arm 또는 manual capture | /vehicle/armed | std_msgs/msg/Bool | vehicle_io serial sender | operator ARM 요청 상태 (실제 출력 승인 피드백 아님) |
| vehicle_io serial sender | /vehicle/calibration_ready | std_msgs/msg/Bool | drive_arm, manual capture | firmware 기준값 신뢰 모드 진입/선택적 자동 보정 완료 |
| vehicle_io serial sender | /vehicle/calibration_status | std_msgs/msg/String | drive_arm, 운영자 | WAIT_CONFIG/READY/LOCKED 등 진단 |

MotionCommand 필드: `int32 steering`, `int32 left_speed`, `int32 right_speed`.
steering -7=좌, 0=차량별 CENTER, +7=우. 속도 -255..255 signed PWM 명령이며 m/s가 아니다.
READY && ARM일 때만 전달한다. READY는 물리 검증 완료를 의미하지 않는다.
ARM 전/부팅/정지 시 drive 및 steering PWM OFF. 정상 armed 조향 0은 CENTER 추종이다.
정상 명령을 500 ms 미만 간격으로 계속 보내야 하며 중단 시 MCU가 전체 PWM을 차단한다.
STOP은 serial X, wire motion 형식은 `s{steering}l{left}r{right}\n`.

센서 publisher QoS는 RELIABLE/KEEP_LAST(1)이고, canonical command는 RELIABLE/KEEP_LAST(1).
armed/ready/status는 RELIABLE/TRANSIENT_LOCAL/KEEP_LAST(1).
상태를 받는 도구는 transient_local QoS를 사용한다. /vehicle/armed의 latched true는
운전자 요청이므로 이것만으로 bridge의 실제 승인/전기적 출력 상태를 판정하면 안 된다.
동시에 여러 controller나 arm UI를 실행하지 않는다. command mux/실제 gate feedback은 향후 과제.

## 현재 전체 application topic 표

위 공식 표의 topic 외에 아래 경로가 존재한다. ROS 표준 `/rosout`, `/parameter_events`와
서비스/action/lifecycle 관리 엔드포인트는 application 메시지 계약과 구분한다.

| Publisher | Topic | Message Type | Subscriber | Purpose |
|---|---|---|---|---|
| sensor_bringup camera (manual/track/mission), legacy image_publisher | /image_raw | sensor_msgs/msg/Image | manual, track, YOLO, debug YOLO, sw_verification | 호환 전방 입력 (중복 publisher 실행 금지) |
| legacy lidar publisher | /lidar_raw | sensor_msgs/msg/LaserScan | lidar_processor | canonical과 중복; 실차 선택 금지 |
| lidar_processor | /lidar_processed | sensor_msgs/msg/LaserScan | obstacle_detector, parking_control, ver2_parking_control, sw_verification | 필터링 scan |
| lidar_obstacle_detector | /lidar_obstacle_info | std_msgs/msg/Bool | mission_manager (backup 포함) | 장애물 존재 |
| yolov8_node | /detections | interfaces_pkg/msg/DetectionArray | lane extractor, traffic light detector, mission_manager, motion planner/proto, debug YOLO | 공통 객체 검출 |
| traffic_light_detector | /yolov8_traffic_light_info | std_msgs/msg/String | mission_manager (backup 포함) | 신호등 상태 |
| lane_info_extractor | /yolov8_lane_info | interfaces_pkg/msg/LaneInfo | path_planner | 선택 차선 |
| lane_info_extractor | /roi_image | sensor_msgs/msg/Image | path_visualizer | ROI 디버그 |
| path_planner | /path_planning_result | interfaces_pkg/msg/PathPlanningResult | motion_planner/proto, path_visualizer | 목표 경로 |
| mission_manager (또는 backup) | /mission_target_lane | std_msgs/msg/Int32 | lane_info_extractor | 목표 차선 선택 |
| mission_manager (또는 backup) | /mission_force_stop | std_msgs/msg/Bool | motion_planner | 미션 정지 요구 |
| mission_manager (또는 backup) | /mission_speed_limit | std_msgs/msg/Int32 | motion_planner | 미션 PWM 상한 |
| mission_manager (또는 backup) | /mission_state | std_msgs/msg/String | 내장 소비자 없음 | 미션 진단 |
| parking_mission, parking_control, ver2_parking_control, motion_planner_proto | /topic_control_signal | interfaces_pkg/msg/MotionCommand | serial sender (canonical/legacy), sw_verification | 대체 command source; 서로 동시 실행 금지 |
| track_controller | /track_debug_image | sensor_msgs/msg/Image | rqt 등 외부 도구 | 트랙 관측 |
| path_visualizer | /path_visualized_img | sensor_msgs/msg/Image | 외부 도구 | 경로 시각화 |
| yolov8_visualizer | /yolov8_visualized_img | sensor_msgs/msg/Image | 외부 도구 | 검출 시각화 |
| yolov8_visualizer | /dgb_bb_markers | visualization_msgs/msg/MarkerArray | RViz | bbox marker (기존 dgb 철자 유지) |
| yolov8_visualizer | /dgb_kp_markers | visualization_msgs/msg/MarkerArray | RViz | keypoint marker |
| sensor_bringup / legacy lidar TransformBroadcaster | /tf | tf2_msgs/msg/TFMessage | TF/RViz | base_link → laser_frame |
| bench manual capture (dry_run) | /bench/topic_control_signal | interfaces_pkg/msg/MotionCommand | 내장 serial subscriber 없음 | bench 격리; dry_run은 구동 0 |
| bench manual capture | /bench/armed | std_msgs/msg/Bool | 내장 serial subscriber 없음 | bench 격리, false |
| 없음 | /bench/calibration_ready | std_msgs/msg/Bool | bench manual capture | dry_run에서는 실제 READY 불필요 |

motion_planner의 traffic light/LiDAR 직접 구독은 주석 처리되어 있다.
현재 경로는 mission_manager가 이를 해석해 force_stop/speed_limit으로 전달한다.
모델 내부 `skku_track_drive_pkg/messages.py`의 데이터 구조는 ROS interface 타입이 아니다.

## Migration (이번에는 강제 topic rename 없음)

| 현재 경로 | 공식/향후 경로 | 이번 상태 | 이행 방법 |
|---|---|---|---|
| /image_raw (manual/track/mission/direct node 기본) | /camera/front/image_raw | 유지 | feature branch에서 publisher와 모든 subscriber를 같은 launch remap으로 전환; record/replay 및 QoS 확인 |
| /camera/front/image_raw (sensor_check/bench) | 동일 | 사용 중 | camera_topic 또는 YAML camera.front.topic으로 설정 |
| /camera/aux/image_raw | 동일 | sensor_check만 제공 | 역할 확인 후 dataset/주행에 별도로 연결 |
| /lidar_raw | 동일 | 유지 | legacy 입력을 중지하고 sensor_bringup 하나만 실행 |
| /topic_control_signal | 동일 | 기존 이름을 공식으로 채택 | /vehicle/command 별칭은 아직 없음; 임의로 새 이름 publish 금지 |
| /vehicle/armed | 동일 | 요청 상태 의미 명시 | 향후 실제 gate feedback 별도 설계, 현재 Bool 의미 바꾸지 않음 |
| /vehicle/calibration_ready | 동일 | 유지 | auto_calibrate=false일 때 설치된 firmware baseline 신뢰 의미 |

센서 launch와 주행 launch를 동시에 실행해 같은 장치/topic을 중복 소유하지 않는다.
