# 트랙·미션·수직주차·평행주차 실행 구성

네 모드와 수동 보정의 `record_dir` 옵션, 입력 재생, 모델 비교와 측정 양식은
[튜닝 기록·분석 안내](TUNING_RECORD_ANALYSIS_KR.md)에 있다.

작업 디렉터리: `/home/autolab/autodrive_ws/Autonomous_Capstone-integrated`.
속도·정지 경로·센서 전용·수동 보정의 최신 변경은
[실차 튜닝 전 검증 보고](REAL_VEHICLE_TUNING_PREP_KR.md)를 함께 확인한다.
트랙/미션의 카메라·추론 공백의 명령 유지와 정지은
[실측·명령 유지 검증](PERCEPTION_DELAY_SAFETY_KR.md)을 참고한다.

네 모드는 **한 번에 하나씩** 실행한다. 각 launch에는 최종 `MotionCommand` 생성 노드
하나와 `vehicle_io_pkg/serial_sender_node_v2` 하나만 있다. GUI는 모터 명령을 발행하지
않으며 기존 `vehicle/drive_key`와 challenge/heartbeat를 통해 W/S를 공통 arm gate로
전달한다. 펌웨어와 통신 규약은 변경하지 않았다: 조향 -7..+7, 음수=좌측,
양수=우측, 속도 -255..+255, 음수=후진, 프레임 `s{steer}l{left}r{right}\n`.

## 실행 구성 표

모델 경로는 설치된 `skku_track_drive_pkg`의 share 디렉터리 기준이다.
설정 기본값은 `vehicle_bringup_pkg`의 `config/`에서 읽는다.

| 모드 / launch | 실제 모델 | 센서 | 인지·판단·제어 | GUI 실행 파일 | 기본 튜닝 설정 / 사용자 저장 |
|---|---|---|---|---|---|
| 트랙 `track_drive_tuning.launch.py` | `models/best.pt`: lane2, traffic_light | 공통 카메라 | 기존 `track_controller_node`: YOLO → lane2 BEV → 기존 PathPlanner → 기존 Stanley. 장애물/신호등 미션 판단 없음 | `track_tuner_node` | `track_tuning.yaml` / `~/.config/autodrive/track_tuning.yaml` |
| 미션 `mission_drive_tuning.launch.py` | `models/mission/best.pt`: lane1, lane2, obstacle, traffic_light | 공통 카메라 | `mission_controller_node`가 TrackController를 상속. 같은 차선·경로·Stanley 계산 후 MissionCore가 필요할 때 개입 | `mission_tuner_node` | `mission_tuning.yaml` / `~/.config/autodrive/mission_tuning.yaml` |
| 수직주차 `perpendicular_parking.launch.py` | 사용 안 함 | 공통 그릴 앞 LiDAR | `parking_controller_node` 실행 파일, ROS 이름 `perpendicular_parking_controller_node`. 기존 ver2의 상태 순서와 공유 시간표를 사용 | `perpendicular_tuner_node` | `perpendicular_parking.yaml` / `~/.config/autodrive/perpendicular_parking.yaml` |
| 평행주차 `parallel_parking.launch.py` | 사용 안 함 | 공통 그릴 앞 LiDAR | 같은 주차 실행 파일, ROS 이름 `parallel_parking_controller_node`. 실제 공간 탐색과 bicycle model 기반 진입·정렬 상태를 사용 | `parallel_tuner_node` | `parallel_parking.yaml` / `~/.config/autodrive/parallel_parking.yaml` |

네 모드 공통 출력: `serial_sender_node_v2` + `drive_arm_node`.
트랙/미션 sender의 추가 지연 기준은 `config/perception_delay_policy.yaml`의 별도
모드 profile이다(기존 6초 기준까지 명령 유지 후 X/disarm). 기존 GUI P 저장 파일과 분리한다.
공통 카메라 실행 파일은 `sensor_bringup_pkg/camera_publisher_node`, LiDAR는
`sensor_bringup_pkg/lidar_publisher_node_v2`이다. 인지·경로·제어 계산은 표의
각 controller 안에서 수행하며 별도의 중복 제어 노드를 실행하지 않는다.
차량 장치 기본 설정: `vehicle_bringup_pkg/config/vehicle.yaml`.
명시적인 `camera_device`, `arduino_port`, `lidar_port` 인자가 차량 설정보다 우선한다.
트랙은 필요한 카메라만 기본 실행하며 `use_lidar:=true`를 명시할 때만 LiDAR를 추가한다.
미션은 카메라의 현재 경로 차단 판단을 사용하며 구 미션 launch의 LiDAR Bool 정지 노드를
중복 실행하지 않는다. 두 주차 모드는 카메라·YOLO를 실행하지 않는다.

`launch_pkg/mission_launch.py`는 과거 미션 파이프라인용 호환 launch이며 유지했다.
이번 네 모드와 병행 실행하지 않는다. 기존 `launch_pkg/parking_launch.py`와
`ver2_parking_control_node` 자체는 과거 직접 출력 경로이므로 새 주차 launch 대신 실행하지 않는다.

## 빌드 및 source

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash
```

아래 `/dev/video2`, `/dev/arduino`, `/dev/lidar`는 사용 장치에 맞게 선택한다.
`lidar_rotation:=180.0`은 기존 드라이버 기본값을 명시한 것이며, 실제 장착 방향의
측정을 대신하지 않는다. 자동 endpoint calibration은 기본 false이며 자동 시험에서
true로 바꾸지 않았다.

```bash
# 1. 트랙 주행: 기존 모델/Stanley, 기본 PWM 250. 명시 speed와 GUI 변경 가능
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py \
  camera_device:=/dev/video2 arduino_port:=/dev/arduino device:=cuda:0

# 2. 장애물 회피 + 신호등: 기본 미션 모델을 자동 선택
ros2 launch vehicle_bringup_pkg mission_drive_tuning.launch.py \
  camera_device:=/dev/video2 arduino_port:=/dev/arduino device:=cuda:0

# 3. 수직주차: 실측 주차 설정 입력 후 W
ros2 launch vehicle_bringup_pkg perpendicular_parking.launch.py \
  lidar_port:=/dev/lidar lidar_rotation:=180.0 arduino_port:=/dev/arduino

# 4. 평행주차: 실측 주차/속도/조향 설정 입력 후 W
ros2 launch vehicle_bringup_pkg parallel_parking.launch.py \
  lidar_port:=/dev/lidar lidar_rotation:=180.0 arduino_port:=/dev/arduino
```

미션에서 출발 시 초록불을 기다리는 조건을 추가하려면 `wait_for_green:=1`.
별도 미션 모델은 `model_path:=/절대경로/mission.pt`; 모델에 네 클래스와 segmentation
task가 모두 있어야 한다. 검출 전용 모델이나 `best_zero.pt` 등 누락 클래스 모델은
시작 시 오류를 보고하고 사용하지 않는다.

저장값 대신 명시적 설정으로 시작하려면 각 launch에 `tuning_config:=/절대경로/설정.yaml`.
저장 위치는 `saved_tuning_path:=/절대경로/모드별설정.yaml`로 선택할 수 있다.
설정 파일의 ROS 루트 이름도 모드별로 다르다. 다른 모드 파일을 불러오거나 덮어쓰려
하면 거부한다. 기본 순서는 패키지 기본값 → 명시적 YAML 또는 기존 사용자 저장 YAML
→ 명시적 launch 파라미터이다. YAML의 0 치수는 미측정이라는 뜻이다.

## GUI와 저장

- **W**: 공통 gate에 출발/재개 요청. 차량 READY와 새 W challenge 및 GUI heartbeat가 필요하다.
- **S**: 공통 gate에서 정지/disarm. X/Space도 정지한다.
- **P**: ROS가 승인한 현재 파라미터를 해당 모드 파일에 원자적으로 저장한다.
  아직 승인되지 않은 변경은 저장하지 않으므로 승인 로그 이후 P를 누른다.
- **D**: debug 화면 표시 전환. 트랙·미션에서 **B**는 BEV 표시 전환.
- 주차에서 **R**: S/disarm 후 탐색 상태와 추정 위치를 초기화한다.
- GUI/디버그/Controls 창에 초점을 놓고 W/S를 누를 수 있다. 창을 닫으면 S를 보내고
  tuner heartbeat를 중단한다. GUI가 없으면 실제 모터 주행을 ARM할 수 없다.

트랙 GUI는 속도와 기존 조향/인지 튜닝을 제공한다. 미션 GUI는 그 위에 장애물 확인·해제
프레임 수, 근접 y, 경로 폭 여유, 대체 차선 최소 면적, 회피 유지 시간/속도, 카메라
지연 표시(`image_timeout_s`, 정지 조건 아님), 신호등 폭/하단 y, 적/녹 확인 프레임,
HSV 범위/최소 색 비율을 노출한다.
미션 상태와 선택 차선, 신호등 색/비율/확인 횟수, 경로 차단 여부는 상태 화면과
`/mission/status`에 표시된다. 원본 이미지에는 차선/경로와 차단 장애물 박스를 표시한다.
추론과 독립적인 생존 신호/20Hz 송신 및 정지 원인 구분은
[INFERENCE_SERIAL_LIVENESS_KR.md](INFERENCE_SERIAL_LIVENESS_KR.md)를 참고한다.

주차 GUI는 치수/장착 위치·각도·관측 범위·검사 섹터/거리·연속 스캔·PWM/조향·시간과
거리별 시간표 또는 공간/진입/정렬 값을 노출한다. 여러 Controls 창에 나누어 표시한다.
`/parking/debug_image`에는 차량과 LiDAR, 검사 부채꼴, 유효 반사점, 상태, 거리,
조향·PWM을 표시한다. `/parking/status`에는 전체 상태를 JSON으로 발행한다.
실측 치수/속도/조향/방향 또는 계획에 영향을 주는 주차 값은 S 상태에서 변경한다.
평행주차의 보정된 PWM을 바꾸려면 먼저 `geometry_confirmed=0`으로 놓고 m/s를 다시
측정한다. 설정·확인·저장 후 새 W로 시작한다.

## 미션 개입 방식

미션용 pt는 통합본 루트 `best.pt`와 동일한 파일을 패키지에 설치하도록 복사했다.
트랙 모델은 변경하지 않았다. 클래스 번호를 코드에 고정하지 않고 모델의 클래스
이름을 사용한다.

장애물의 bounding box 하단을 현재 BEV 경로를 카메라 영상으로 역투영한 선과
비교한다. 경로 밖의 장애물은 차선 변경을 일으키지 않는다. 근접한 경로 차단 물체는
확인 중 정지하며, 확인 후 대체 차선 마스크가 있고 그 차선도 막히지 않았을 때
lane1 ↔ lane2를 선택한다. 선택만 바뀌며 조향은 기존 차선 중심/경로/Stanley가
계산한다. 차선이 바뀌면 이전 차선의 held 중심점을 지워 잘못된 경로를 재사용하지 않는다.
회피 차선이 없거나 막힌 경우 정지한다. 장애물 통과 후 선택한 차선을 유지하며
다음 경로 차단 장애물에 다시 판단한다.

신호등은 `traffic_light` 박스의 HSV 색상으로 판단한다. 빈 ROI나 색상 증거가 없는
경우 Unknown이다. 확인된 빨간불은 확인된 초록불까지 정지 상태를 유지한다.
노란불은 속도를 제한한다. 초기 초록불 대기는 선택 사항이다. 신호등·장애물 개입이
없으면 기존 경로·Stanley 명령을 그대로 반환한다. 예외나 카메라 stale로 미션
판단이 중단되면 정지 명령을 발행한다. 트랙 모드에는 이 미션 판단이 연결되지 않는다.

## 주차 구현 조사와 재사용 범위

`autodrive_ws`의 통합본·교육용·contest 워크스페이스와 `~/Autonomous_Capstone`에서
주차 코드를 검색하고 상태/조향/실행 연결을 확인했다. 이름만으로 분류하지 않았다.

| 기존 후보 | 실제 동작과 연결 | 이번 사용 |
|---|---|---|
| `lidar_perception_pkg/parking_control_node.py` | 슬라롬 위치 이동 뒤 전진 회전·후진 회전·측면 검사·출차. 별도 실행 파일 | 수직주차 계열 참고 |
| `lidar_perception_pkg/ver2_parking_control_node.py` | 슬라롬을 없애고 거리별 시간표로 같은 수직 진입 동작 수행. 기존 주차 launch에는 연결되지 않음 | `parking_timing.py`로 시간표를 공유하고 동일한 상태/조향 순서를 새 gate·파라미터·화면에 연결 |
| `decision_making_pkg/parking_mission_node.py` | 센서 없이 서로 반대인 후진 조향 두 구간·직선 후진·출차. `launch_pkg/parking_launch.py`에서 실행 | S자 후진 제어의 참고 후보. 공간 탐색/차량 치수/끝 자세가 없어 완성된 평행주차로 판정하지 않음 |
| contest `motion_parking.py` | 두 차량 감지·미감지 순서, 양측 감지로 후진 정렬, 양측 소실로 정지·출차. `final_parking.py`에서 실행하며 구 출력 bridge는 주석 | 수직 진입 계열. 현재 통신 범위를 벗어난 30 조향 등이 있어 새 경로에서 직접 실행하지 않음 |
| contest `rear_parking_fsm.py` | `/odom` yaw 변화 85°를 후진 회전 완료 조건으로 사용. 별도 entry point이며 `final_parking.py`에서는 실행하지 않음 | 최종 주차 자세를 도로와 수직으로 돌리는 후보. 완성된 평행주차가 아님 |

평행주차의 공간 탐색→길이 판정→진입 위치 결정→두 후진 곡선→정렬→완료를 모두
갖춘 재사용 구현은 확인하지 못했다. 그래서 수직주차 이름을 바꿔 연결하지 않고
별도 평행주차 상태 전환과 진입 기하를 구현했다. 두 주차 모드는 Scan 변환,
섹터 검사, 명령 범위/부호, 화면, 파라미터, W/S 및 serial 코드를 공유한다.

## 그릴 앞 LiDAR와 필요한 측정값

주차 계산 원점은 **뒷차축 중앙**, +X는 전방, +Y는 왼쪽, 각도는 반시계 방향이다.
`LaserScan.angle_min`과 `angle_increment`로 각 샘플을 해석하며 배열 인덱스를
각도로 가정하지 않는다. 드라이버의 `lidar_rotation`은 upstream에서 한 번만 적용한다.
그 결과에 대해 소비자 쪽의 실측 `scan_angle_sign`(±1), `lidar_yaw_deg`,
`lidar_x_m`, `lidar_y_m`을 적용한다. 화면의 검사 부채꼴 원점도 실제 LiDAR 위치다.
이 화면 변환은 raw scan의 TF 원점이 뒷차축이라는 가정을 사용하지 않는다.

`visible_min_deg`/`visible_max_deg`로 실제 차체/그릴 가림 범위를 제외하고
`range_min_m`/`range_max_m`로 검사 거리를 선택한다. 기본 -135..135°는 **미검증
초기 범위**이며 실측해야 한다. 무반사 inf/NaN 또는 범위 밖은 빈 주차 공간이나
안전한 후방이라는 증거로 사용하지 않는다. fresh 스캔 단위로 연속 감지를 세며
같은 스캔을 재사용하는 timer tick은 감지 횟수를 늘리지 않는다.

| 실측값 | 설정 이름 / 목적 |
|---|---|
| 축거, 앞차축~앞범퍼, 뒷차축~뒤범퍼, 차량 폭 | `wheelbase_m`, `front_overhang_m`, `rear_overhang_m`, `vehicle_width_m`. 차량 길이는 세 종방향 치수의 합 |
| 뒷차축~LiDAR 전후/좌우 위치 | `lidar_x_m`, `lidar_y_m`. 그릴 앞이므로 x는 일반적으로 양수지만 실측 전에는 0으로 보존 |
| 전방/좌측/우측 물체의 scan 방향, 장착 yaw, 실제 가림 | `lidar_rotation`, `scan_angle_sign`, `lidar_yaw_deg`, `visible_min_deg`, `visible_max_deg` |
| 선택 PWM에서 실제 전진·후진 m/s | 평행주차 `forward_mps`, `reverse_mps`. PWM 값을 거리/속도로 간주하지 않음 |
| 조향 ±7의 실제 앞바퀴 각도 및 좌우 부호 | 평행주차 `max_wheel_angle_deg`, 공통 `steering_sign`. 좌우 비대칭과 타이어 슬립은 현재 모델에 포함되지 않음 |
| 평행 공간 깊이 및 시작 차선→최종 차량 중심 횡이동 | `slot_depth_m`, `entry_lateral_m`. 후자는 주차 쪽으로 이동할 양의 거리 |
| 시작 자세 | 차량이 도로/주차 공간 경계와 평행한 자세로 출발. R 시 이 자세를 yaw=0으로 둠. 현재 arbitrary yaw 시작 보정은 없음 |
| 공간 길이·진입 여유·종료 허용오차 | 길이는 두 경계 감지 사이의 보정 속도 적분으로 추정. `gap_margin_m`, `entry_advance_m`, `center_tolerance_m`, `align_tolerance_deg`는 실측 코스에서 조절 |

두 주차 설정에서 `geometry_confirmed=0`과 미측정 치수 0은 자동으로 만들어낸
차량 제원이 아니다. 실제 값을 입력한 후 S 상태에서 확인값을 1로 바꾸고 저장한다.
기본 상태에서는 GUI/관측은 동작하지만 이동 명령은 0이다.

## 실제 구현한 주차 단계와 한계

**수직주차**: SEARCH → FWD_TURN → SETTLE → REV_STRAIGHT → REV_TURN →
REV_SIDE_CHECK → PARKED. `exit_enabled=1`일 때 pause 후 EXIT_STRAIGHT →
EXIT_TURN → EXIT_FINISH → DONE을 수행한다. 기본 exit_enabled는 0이다.
기존 시간표는 보존했고 거리 경계는 1.0/1.5/2.0m, 진입 최대 2.5m로 일관되게
수정하여 네 구간을 사용할 수 있게 했다. 기존의 배열 기반 각도와 반복 timer 감지,
launch 직후 자동 시간 진행은 새 경로에서 사용하지 않는다. S/disarm·stale·collision
정지 동안 주차 상태 시간은 진행하지 않는다. 출차는 기존 시간 기반 동작이며
끝선 인식으로 검증하는 출차는 아니다.

**평행주차**: SEARCH_FIRST_CAR → SEARCH_GAP → MEASURE_GAP → APPROACH_ENTRY →
ENTRY_SETTLE → REVERSE_ARC_IN → REVERSE_COUNTER_STEER → ALIGN → DONE.
너무 짧은 공간은 다시 탐색한다. 공간 길이와 차량 길이/여유, 조향 반경으로
두 곡선의 각도와 뒷차축 진입 x를 결정한다. 반대 조향으로 도로와 평행한 자세로
돌아오고 측면 경계 직선 fitting으로 정렬 여부를 확인한다. 관측 경계가 부족하면
정지하여 기다리고 timeout 시 FAULT_STOP이다. **평행주차 출차는 구현하지 않았다.**

현재 위치는 실측 PWM별 속도와 bicycle model의 **적분 추정**이며 encoder/odom/SLAM
피드백이 아니다. 슬롯 중심 위치와 완료 위치 역시 추정값이다. 미확인 치수 상태에서
진입을 허용하지 않으며 실제 성공률을 검증했다고 보고하지 않는다. 그릴 앞 LiDAR만으로
차체 뒤 가림 영역의 충돌 여유를 보장할 수 없다. 실제 코스 시험 전에 후방 관측 수단과
독립적인 이동거리/자세 측정을 정해야 한다. 미측정 기본 상태에서의 전달 범위는
인지 화면, 파라미터, 상태 전환/제어 구현과 모의 검증이다.

## 검증 및 모터 없이 실행

빌드·Python 컴파일·모의 입력·launch topology·기존 W/S/통신 회귀를 실행했다.
현재 검사 결과는 최신 검증 보고를 참고한다.
격리 ROS_DOMAIN_ID=143에서 네 launch를 `dry_run:=true gui:=false`로 시작/종료하고,
실제 ROS parameter service로 GUI 값 수정/승인/저장을 확인했다. HighGUI의 창 함수만
대체하여 실제 화면 drawing과 파라미터/저장 callback을 검증했다. 디스플레이 장치에서의
실제 창 배치는 실차 화면 확인 대상이다. 원래 환경의 SciPy/NumPy 지원 버전 경고는
남아 있으나 이번 검사에서 실패는 없었다. 모터 장치 연결·구동과 펌웨어 업로드는 하지 않았다.

`dry_run`은 카메라·LiDAR·시리얼·arm gate를 실행하지 않고 명령 토픽도
`/dry_run/topic_control_signal`로 분리한다. 예를 들어 화면/상태만 확인하려면:

실제 센서와 GUI를 한 launch로 확인하려면 `sensors_only:=true`를 사용한다.
이는 센서를 켜고 차량 시리얼/gate를 생략하며 명령·GUI 키 토픽도 preview로 분리한다.

```bash
ros2 launch vehicle_bringup_pkg perpendicular_parking.launch.py dry_run:=true
ros2 launch vehicle_bringup_pkg parallel_parking.launch.py dry_run:=true
```

센서 입력도 없으므로 위 실행은 미측정/스캔 대기 상태를 보여준다. recorded scan을
`/parking/scan`으로 재생할 수 있다. 실제 차량 도메인과 분리하려면 앞에
`ROS_DOMAIN_ID=143`을 붙인다. GUI callback 검증 스크립트는 직접 창을 열지 않는다:

```bash
python3 -m pytest -q \
  src/skku_track_drive_pkg/test/test_mission_parking.py \
  src/skku_track_drive_pkg/test/test_lane_processing.py \
  tools/tests/test_four_mode_launches.py \
  tools/tests/test_drive_safety.py tools/tests/test_canonical_safety.py
ROS_DOMAIN_ID=143 ROS_LOG_DIR=/tmp/codex_four_modes_validation \
  python3 tools/validate_mode_guis.py
```

GUI 검증 결과/임시 저장/렌더링:
`/tmp/codex_four_modes_validation/gui_validation.json`, `*.saved.yaml`, `*.tuner.png`,
`perpendicular.lidar.png`, `parallel.lidar.png`.
주차 상태 화면과 LiDAR 화면은 실제 dry-run controller의 발행 결과이며 센서 미연결 상태다.
사용자 `~/.config/autodrive` 저장 파일은 검증에서 덮어쓰지 않았다.
