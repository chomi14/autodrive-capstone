# Canonical Workspace 감사 (2026-09-17)

대상: `/home/autolab/autodrive_ws/Autonomous_Capstone-integrated`만 수정.
변경 전 branch `main`, working tree clean. 실제 Git 최상위는 부모
`/home/autolab/autodrive_ws`이다. 이 디렉터리는 그 저장소의 하위 workspace이며,
독립 저장소 전환/브랜치 생성/커밋은 하지 않았다. Git 명령은 경로 범위를 확인해야 한다.
다른 workspace 코드 복사, 파일 삭제, package rename, 알고리즘 변경, upload/구동 없음.

## 구조 판단 및 충돌 (변경 전 감사)

6개 canonical 계층은 이미 존재한다. 다만 미션 perception/planning은 legacy 계열
package의 실제 사용 기능이므로 통째로 legacy/deprecated 처리하면 안 된다.
미션 통합 launch는 아직 `launch_pkg/mission_launch.py`에 남겨 호환 유지한다.
트랙과 미션은 별도 알고리즘 경로이며 이번 작업에서 합치지 않았다.
`/image_raw`와 `/camera/front/image_raw`는 launch마다 다르다. 강제 rename하지 않았다.
카메라/serial/LiDAR 중복 노드는 동시에 실행하면 장치 및 topic 경쟁이 발생한다.

| Package/File | Classification | Current Role | Replacement/Canonical | Risk |
|---|---|---|---|---|
| interfaces_pkg (msg/*, CMakeLists) | KEEP | ROS 공통 메시지 | interfaces_pkg | MotionCommand 단위/부호 합의 필요 |
| sensor_bringup_pkg (camera_publisher_node, lidar_publisher_node, rplidar_driver) | KEEP | 센서 입력/TF | sensor_bringup_pkg | 장치 alias, USB 연결, QoS 실차 확인 |
| vehicle_io_pkg (serial_sender_node, drive_arm_node) | KEEP | READY + ARM 게이트/serial | vehicle_io_pkg | firmware와 함께 적용 필요 |
| manual_drive_pkg (manual_drive_capture_node) | KEEP | WASD/이미지+라벨/bench | manual_drive_pkg | 실제/bench topic 구분 |
| skku_track_drive_pkg (track_controller_node, perception/planners, models/best.pt) | KEEP | 현재 트랙 알고리즘 | skku_track_drive_pkg | 알고리즘/모델 불변; 성능 검증 별도 |
| vehicle_bringup_pkg (launch/*, config/*) | KEEP | 센서/bench/수동/트랙 통합 | vehicle_bringup_pkg | topic 호환 기간 필요 |
| camera_perception_pkg (package 전체) | MERGE_CANDIDATE | 미션 perception + 중복 센서 | 센서는 sensor_bringup; perception 유지 | 미션에서 실제 사용 |
| camera_perception_pkg/image_publisher_node.py | MERGE_CANDIDATE | 카메라 + 이미지/영상 재생 | sensor_bringup; 재생 기능 향후 이식 | 단순 삭제 시 offline 입력 상실 |
| camera_perception_pkg/yolov8_node.py, lane_info_extractor_node.py, traffic_light_detector_node.py, lib/* | KEEP | 미션 검출/차선/신호등 | 현 위치 유지 | best.pt 상대경로, cuda:0 기본 |
| lidar_perception_pkg (package 전체) | MERGE_CANDIDATE | 미션 처리 + 중복 장치 입력 | 입력만 sensor_bringup | 필터/주차 기능 보존 |
| lidar_perception_pkg/lidar_publisher_node.py | REMOVE_CANDIDATE | 구 LiDAR 입력 | sensor_bringup_pkg | /dev/ttyUSB0 하드코딩; 지금 삭제 금지 |
| lidar_perception_pkg/lib/lidar_perception_func_lib.py | REMOVE_CANDIDATE | 중복 RPLidar 드라이버 | sensor_bringup_pkg/rplidar_driver.py | legacy publisher 의존, 삭제 금지 |
| lidar_perception_pkg/lidar_processor_node.py, lidar_obstacle_detector_node.py | KEEP | 미션 LiDAR 처리 | 현 위치 유지 | mission_launch 의존 |
| lidar_perception_pkg/parking_control_node.py, ver2_parking_control_node.py | LEGACY | 별도 주차 제어 | 향후 gate 연결 검증 | command publisher 경쟁 |
| serial_communication_pkg (serial_sender_node, sw_verification_node, lib/*) | LEGACY | 직접 serial 출력/검증 | vehicle_io_pkg | READY/ARM 우회, import 시 serial open |
| launch_pkg (package 전체) | MERGE_CANDIDATE | 미션 및 과거 launch | vehicle_bringup_pkg로 향후 진입점 통합 | 기존 진입점 보존 |
| launch_pkg/mission_launch.py | KEEP | 새 I/O gate 사용 미션 launch | 현 위치 유지; 향후 wrapper | 3개 미션 package 필수 |
| launch_pkg/main_launch.py, parking_launch.py | LEGACY | 구 serial sender 실행 | track/manual 및 향후 gated parking | 실차 사용 금지 |
| decision_making_pkg (mission_manager_node, path_planner_node, motion_planner_node, lib/*) | KEEP | 사용 중인 미션 알고리즘 | 현 위치 유지 | 트랙 planner와 역할 유사하나 동일 기능 아님 |
| decision_making_pkg/parking_mission_node.py | MERGE_CANDIDATE | 시간 기반 주차 | 향후 gated launch | 현재 parking_launch가 legacy serial 사용 |
| decision_making_pkg/mission_manager_backup.py, motion_planner_proto.py | LEGACY | 백업/프로토타입, entry point 없음 | active *_node.py | 임의 실행 금지 |
| debug_pkg (path_visualizer_node, yolov8_visualizer_node) | KEEP | 미션 관측/시각화 | debug_pkg | 구 image topic 의존 |
| control/driving_user_pins/driving_user_pins.ino | KEEP | 현재 차량 firmware | 이 파일만 주행 기준 | watchdog/물리 핀 검증 필요 |
| control/steering_limit_calibration/* | KEEP | 명시적 bench endpoint 측정/ADC 읽기 | 현 위치 유지 | c 입력 시 실제 조향, 실차 검증 필요 |
| control/driving/* | LEGACY | 과거 주행 firmware | driving_user_pins | 왼쪽 IN1=6 IN2=7: 현재 배선과 반대, watchdog 없음 |
| control/motor_test/*, sw_verification/* | LEGACY | 직접 모터 시험 | 수동 검증 전용 | 자동 실행/업로드 금지 |
| control/check_variable_resistor/*, ultrasonic/* | LEGACY | 센서 진단 | 참고용 유지 | 주행 firmware 아님 |
| data_collection/data_collection.py | LEGACY | 직접 serial + 영상 수집 | manual_drive_pkg | safety gate 우회 |
| tools/*.sh | KEEP | 장치 조사/설치/alias 보조 | 수동 관리 | sudo/udev 설치는 이번에 실행 안 함 |
| root *.pt | MERGE_CANDIDATE | 과거/미션 가중치 | models/ 계획 참조 | 삭제/이동 금지 |

## Launch 안전 경로

| Launch | Serial path | Gate | 사용 판단 |
|---|---|---|---|
| vehicle_bringup_pkg/sensor_check.launch.py | 없음 | 해당 없음 | 센서만; LiDAR 자체 회전 가능 |
| vehicle_bringup_pkg/bench_capture.launch.py | 없음 | dry_run, /bench/* | CANONICAL - USE THIS (라벨링 bench) |
| vehicle_bringup_pkg/manual_capture.launch.py | vehicle_io_pkg/serial_sender_node_v2 | READY + operator ARM | CANONICAL - USE THIS |
| vehicle_bringup_pkg/track_drive.launch.py | vehicle_io_pkg/serial_sender_node_v2 | READY + drive_arm_node | CANONICAL - USE THIS |
| launch_pkg/mission_launch.py | vehicle_io_pkg/serial_sender_node_v2 | READY + drive_arm_node | 호환 미션 경로 유지 |
| launch_pkg/main_launch.py | serial_communication_pkg/serial_sender_node | 없음 | LEGACY - DO NOT USE FOR VEHICLE RUN |
| launch_pkg/parking_launch.py | serial_communication_pkg/serial_sender_node | 없음 | LEGACY - DO NOT USE FOR VEHICLE RUN |

`serial_communication_pkg/sw_verification_node.py`, `data_collection/data_collection.py`도
직접 포트를 열므로 vehicle_io 안전 게이트를 거치지 않는다. legacy sender는 import 자체가
포트 open을 유발한다. 이번 import 검증에서는 해당 모듈을 실행하지 않는다.
문서 표시는 실행을 기술적으로 차단하는 장치가 아니다.

## 설정과 안전 변경

공유 설정 원본: `src/vehicle_bringup_pkg/config/vehicle.yaml` (install share/config 배포).
`vehicle_config:=/absolute/profile.yaml`로 부분 profile을 덮어쓸 수 있다.
우선순위: 기존 launch argument > 선택 profile > 공유 YAML > 코드 fallback.
파일이 지정되었는데 없거나 YAML이 잘못되면 조용히 다른 장치를 쓰지 않고 실패한다.
카메라 by-path는 노트북 PCI/USB topology에 종속되므로 공유 파일에 영구 고정하지 않는다.
제공된 Front/Aux C920 경로는 `laptop_autolab.example.yaml`에만 저장했다.
공유 `/dev/video0`, `/dev/video2`도 탐색 기본값일 뿐 장치 정체성을 보증하지 않는다.
`/dev/arduino`, `/dev/lidar`는 목표 alias이며 udev 생성 여부는 별도 확인 필요.

기존 `camera_device`, `aux_camera_device`, `lidar_port`, `lidar_rotation`, `arduino_port`,
`calibration_tolerance` 보존. `arduino_baud`, `auto_calibrate`, `vehicle_config` 추가.
센서/bench의 카메라 topic은 YAML 및 topic argument 사용.
트랙/수동/미션은 `/image_raw`, `/lidar_raw`, `/topic_control_signal` 계약을 그대로 유지한다.
이 경로의 topic까지 YAML으로 일괄 전환하지 않았다. ROS_INTERFACE migration 참조.
직접 `ros2 run`은 노드 parameter fallback을 쓰며 YAML을 자동으로 읽지 않는다.
steering ADC/max_step YAML은 차량 기록용이며 firmware로 자동 전송하지 않는다.
기존 K 프로토콜은 성공한 endpoint 측정 후에만 적용되므로, 이를 우회해 시작 시 ADC를
주입하지 않는다. 실제 기준값은 firmware DEFAULT_LEFT/CENTER/RIGHT가 권위값이다.

DEFAULT_LEFT=600, DEFAULT_CENTER=522, DEFAULT_RIGHT=445, MAX=7 유지.
CENTER=522는 이전 정수 midpoint를 보존한 임시 값이며 직진 실측값이 아니다.
ADC가 증가/감소하는 두 배선 방향 모두 piecewise mapping 지원.
Kleft,right는 baseline에서 CENTER의 상대 위치를 보존해 runtime center를 계산한다.
자동 endpoint 측정만으로 물리적 직진을 알아낼 수 없으므로 별도 직진 측정 필수.
`steering_limit_calibration`은 정지 상태 p 입력으로 ADC를 읽고 x로 중단 가능.
그 스케치의 RESULT_CENTER는 여전히 endpoint midpoint 추정값이다.

Firmware는 정상 전체 command frame에만 timestamp를 갱신한다. 500 ms 이상 단절 시
좌/우 PWM과 조향 PWM을 모두 0으로 하고 새 정상 command 전까지 유지한다.
시작/X/DISARM도 조향 중앙 복귀 없이 출력 OFF. 부호/핀은 원래 차량 값을 유지했다.
잘린/초과길이/잘못된/범위 밖 입력은 watchdog을 연장하지 않는다.
Serial 처리량에 상한을 두어 입력 flood가 watchdog 검사를 굶기지 않게 했다.
명시적 C 보정도 정상 zero command heartbeat 단절 시 중단한다.
PC gate는 보정 중에만 zero frame을 보내고 평상시 DISARM에는 X를 보낸다.
모든 gated launch와 노드 자체 auto_calibrate 기본값은 false.
`auto_calibrate:=true`는 운전자가 의도한 물리적 endpoint 시험 때만 사용한다.
정상 구동 중 0 조향 command는 의도적인 CENTER 제어이므로 X와 다르다.

## 남은 안전 제한

Watchdog은 명령 신선도만 판단하며 반복되는 잘못된 의도/상위 센서 freeze는 검출하지 못한다.
현재 /vehicle/armed는 operator 요청이며 실제 bridge 승인 상태를 돌려주는 피드백이 아니다.
여러 command publisher/multiple arm UI를 동시에 켜지 않는다. 별도 arbitration/arm lease는 향후 과제.
조향 기계 걸림, 전기적 브레이크/관성 정지거리, USB buffer의 잔여 command는 실차 검증 필요.
