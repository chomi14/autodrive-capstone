# 실차 튜닝 전 수정·검증 (2026-10-04)

작업 디렉터리: `/home/autolab/autodrive_ws/Autonomous_Capstone-integrated`.
차량 모터·실제 센서·시리얼 포트를 실행하거나 펌웨어를 업로드하지 않았다.
아래 실제 장치 명령은 운영자가 수행할 절차이며 자동 검사와 구분한다.

## 속도·정지 경로의 변경 이력

`fc96813b8a127723ba97f1951db702172439619a` (2026-09-24)은 트랙 launch에
`FIXED_TRACK_SPEED=250`, controller/GUI의 `allow_speed_tuning=False`를 넣었다.
주석상 조향 튜닝 중 종방향 튜닝을 금지하려는 정책이었다. settings.update가
YAML과 명시적 speed를 마지막에 250으로 덮어썼다. 같은 commit에서 인지 예외 시
마지막 조향과 설정 속도를 발행하는 fallback도 들어왔다.

`command_timeout=0.0`과 sender의 20Hz 캐시 재송신은 Git HEAD에 없고
수정 전 미커밋 작업 트리에 있었다. 당시 설명은 영상 지연·controller 종료 중에도
주행을 유지하려는 정책이었다. 확정된 commit/작성자를 추정하지 않는다.

트랙의 강제 덮어쓰기를 제거하고 GUI/ROS speed 변경을 허용했다.
패키지 track_tuning.yaml의 speed를 250으로 명시하여 이전 **실효 기본값**을 보존했다.
이전 YAML의 80은 launch가 덮어쓰던 값이다. 사용자 트랙 YAML은 수정하지 않았다.
우선순위: 패키지 → explicit tuning_config 또는 사용자 저장 YAML → launch 인자 → 실행 중 GUI 승인값.

트랙 command_timeout=0.0, sender 기본 0.5, 미션 0.75, 주차 0.25,
UI timeout=0.75, 지원 펌웨어 watchdog=0.5초는 더 짧게 바꾸지 않았다.
최신 수정에서는 독립적인 제어기 생존 신호와 성공 결과 진행을 함께 검사한다.
인지 공백 중 마지막 유효 조향·PWM을 20Hz로 유지하며 기존 6초 기준에서 X/disarm한다.
0은 인지 명령 AGE 검사 해제이며, 생존 검사에는 기존 UI 0.75초를 사용한다.
상세 변경과 지연/종료 재현은 [INFERENCE_SERIAL_LIVENESS_KR.md](INFERENCE_SERIAL_LIVENESS_KR.md).
실측 분포·기준 산출·최신 중단 검증은 [PERCEPTION_DELAY_SAFETY_KR.md](PERCEPTION_DELAY_SAFETY_KR.md).

| 상황 | 수정 후 동작 |
|---|---|
| 카메라/추론 지연, 제어기 생존 정상 | 6초 미만 명령 유지→6초 기준 X/disarm. 미션 image_timeout은 별도 지연 진단 |
| 인지 예외 callback | 기존 fallback 자리에 zero MotionCommand. 마지막 주행 명령을 새 명령처럼 발행하지 않음 |
| controller만 종료 | 종료 신호 또는 유한한 생존 만료→sender X/disarm. sender까지 종료되면 firmware 통신 watchdog |
| 전체 launch Ctrl+C / sender SIGINT·SIGTERM | 기존 cleanup에서 ROS와 독립적으로 X 쓰기 시도 후 포트 닫기 |
| S / GUI 종료 / UI 중단 | 기존 disarm→X, challenge 교체. 오래된 W/Bool True는 재출발 불가 |

zero MotionCommand는 속도 0·조향 목표 0이며 정상 제어에서는 조향을 중앙으로
움직일 수 있다. **S/종료의 X는 모든 PWM을 끄는 명령**이다. STOP 로그는 실제 PWM
측정이 아니다. 설치된 펌웨어 버전과 실제 정지는 실차 확인 대상이다.
유효 프레임에서 단순 차선 미검출일 때의 기존 유지 동작은 바꾸지 않았다.
예외 뒤 정상 프레임이 복구되면 gate가 살아 있는 동안 정상 명령으로 돌아온다.
S 또는 통신 watchdog으로 disarm한 뒤에는 새 W가 필요하다.

## 미션 초기화와 독립 저장

기존 미션 사용자 파일이 없어 tools/bootstrap_mission_tuning.py로
`~/.config/autodrive/mission_tuning.yaml`을 생성했다. 트랙 공통값의 일회성 복사이며
이후 미션 실행·P는 트랙 파일을 읽거나 바꾸지 않는다. 패키지 미션 기본 공통값도
이 최초 기준으로 정리했다. bootstrap 재실행은 기존 파일을 보존하고 차이만 출력한다.

| 현재 트랙에서 추출한 공통 파라미터 | 반영한 값 |
|---|---|
| stanley_gain / heading_gain / lookahead_index | 0.027 / 0.55 / 30 |
| confidence / bev_top_shift / look_shift | 0.5 / -15 / 0 |
| ema_alpha / virtual_lane_width | 0.6 / 290 |
| car_center_x / max_steering_angle | 325.0 / 50.0 |
| center_ema_alpha / max_center_jump_px | 0.35 / 80.0 |
| max_missed_frames / min_component_area | 12 / 250 |

speed는 제외하여 미션 80, 장애물·신호등 임계값은 보존했다.
YAML에 없는 공통값은 같은 TrackController 기본 선언을 사용한다.
같은 합성 검출을 5프레임 넣고 공통값을 맞춘 실제 callback에서 경로점·조향이
일치했다. 속도는 250/80으로 독립적이었다. 실제 영상에서는 모델이 달라 검출 결과는 달라질 수 있다.

## 빌드·실행

```bash
cd ~/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash
```

센서 전용 `sensors_only:=true`: 실제 센서/GUI, 차량 sender/gate 생략.
계산 토픽은 `/sensors_only/topic_control_signal`, GUI W/S·heartbeat도 preview로 분리한다.
주차는 disarm 상태로 관측만 한다. 기존 dry_run은 센서까지 끄며 두 옵션을 함께 주면 우선한다.
한 번에 한 모드만 선택한다.

```bash
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py sensors_only:=true camera_device:=/dev/video2 device:=cuda:0 speed:=30
ros2 launch vehicle_bringup_pkg mission_drive_tuning.launch.py sensors_only:=true camera_device:=/dev/video2 device:=cuda:0 speed:=30
ros2 launch vehicle_bringup_pkg perpendicular_parking.launch.py sensors_only:=true lidar_port:=/dev/lidar lidar_rotation:=180.0
ros2 launch vehicle_bringup_pkg parallel_parking.launch.py sensors_only:=true lidar_port:=/dev/lidar lidar_rotation:=180.0
```

사용자가 바퀴를 띄워 W/S·Ctrl+C를 확인한 뒤 실제 저속 주행할 때 sensors_only를 제거한다.

```bash
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py camera_device:=/dev/video2 arduino_port:=/dev/arduino device:=cuda:0 speed:=30 auto_calibrate:=false
ros2 launch vehicle_bringup_pkg mission_drive_tuning.launch.py camera_device:=/dev/video2 arduino_port:=/dev/arduino device:=cuda:0 speed:=30 auto_calibrate:=false
```

## 주차 보정 도구와 측정

기존 도구를 실행하지 않고 확인했다. integrated manual_capture의 S는 감속/후진이고
Bool True ARM은 현재 challenge gate에서 출발 승인되지 않는다. winter motor_test는
조이스틱/ArduinoSerial, contest keyboard_controller는 터미널/직접 시리얼 방식이다.
motor_test.ino들은 별도 스케치다. 요구한 W 출발·S 정지에 맞는 독립 보정 도구는 없어
기존 TrackTuner와 vehicle_io를 재사용한 parking_calibration.launch.py를 추가했다.

자동 주차/트랙 controller나 센서는 실행하지 않는다. live controller들은 공통 command lease로
한 번에 하나만 명령을 만들며 sender의 exclusive serial 잠금도 중복 사용을 거부한다.
보정 명령은 `/parking_calibration/command`, 키·heartbeat·상태는
`/parking_calibration/vehicle/*`로 분리한다. 기존 자동 launch를 종료하고 시작한다.

```bash
# 모터 없이 GUI/파라미터 확인
ros2 launch vehicle_bringup_pkg parking_calibration.launch.py dry_run:=true
# 전진: 운영자가 W 시작·S 정지
ros2 launch vehicle_bringup_pkg parking_calibration.launch.py arduino_port:=/dev/arduino pwm:=60 steering_step:=0 auto_calibrate:=false
# 후진
ros2 launch vehicle_bringup_pkg parking_calibration.launch.py arduino_port:=/dev/arduino pwm:=-60 steering_step:=0 auto_calibrate:=false
# 바퀴를 띄운 조향각 측정: 구동 0, W 이후 조향은 움직임
ros2 launch vehicle_bringup_pkg parking_calibration.launch.py arduino_port:=/dev/arduino pwm:=0 steering_step:=7 auto_calibrate:=false
```

GUI는 signed PWM -255..255, 조향 -7..7을 제공한다. S 상태에서만 변경하고 새 W로 적용한다.
W는 실제 sender 승인 상태를 따르고 S는 공통 X 정지다. 조향 -7도 별도 측정한다.
P는 `~/.config/autodrive/parking_calibration.yaml`에 시험 명령만 저장한다.
치수/속도/geometry_confirmed를 자동 주차 YAML에 자동 반영하지 않는다.

1. 바퀴를 띄워 W/S, 새 W, Ctrl+C의 실제 정지를 확인한다.
2. 조향 0에서 바닥 두 표시선 사이 거리 d(m), 실제 통과 시간 t(s)를 측정한다.
   선택 전진 PWM에서 forward_mps=d/t, 후진 PWM에서 reverse_mps=abs(d)/t를 여러 번 측정한다.
   배터리·바닥·하중 조건을 기록한다. GUI elapsed_s는 ARM 상태 시간으로 통과 시간과 다르다.
3. 직진 바퀴 방향을 0°로 두고 ±7에서 분도기/각도기로 앞바퀴 평면 각도를 측정한다.
   좌우 바퀴와 좌/우 조향 각도를 기록한다. 단일 bicycle 각도와 차이가 크면 모델 한계를 검토한다.

| 주차 YAML 반영 항목 | 단위·측정 기준 |
|---|---|
| forward_pwm / reverse_pwm | 측정 때 사용한 양의 PWM 크기 |
| forward_mps / reverse_mps | 위 PWM에서 측정한 양의 m/s |
| max_wheel_angle_deg | ±7에서 실측 바퀴 각도(°), 트랙 max_steering_angle과 구분 |
| steering_sign / steer_step | 확인한 좌우 부호(±1) / 사용할 명령 단계(1..7) |
| wheelbase_m / front_overhang_m / rear_overhang_m / vehicle_width_m | 축거·앞차축→앞범퍼·뒷차축→뒤범퍼·차폭(m) |
| lidar_x_m / lidar_y_m | 뒷차축 중앙 기준 그릴 LiDAR 회전 중심 위치(m), 전방+x·좌측+y |
| scan_angle_sign / lidar_yaw_deg / visible_min_deg / visible_max_deg | 알려진 물체로 확인한 방향·잔여 yaw·관측 범위(°) |
| slot_depth_m / entry_lateral_m | 공간 깊이·시작→최종 중심선 횡이동(m), 평행주차 |
| case1_*_s~case4_*_s | 선택 PWM에서 재보정한 수직주차 구간 시간(s) |
| geometry_confirmed | 측정/관측 완료 후 S 상태에서 운영자가 수동 확인(0→1) |

각 주차 GUI의 P로 별도 저장한다. 평행주차 PWM 변경 전 geometry_confirmed=0으로 놓고
m/s를 다시 측정한다. 측정값을 임의로 채우거나 확인값을 자동으로 켜지 않았다.
그릴 LiDAR의 lidar_rotation과 lidar_yaw_deg에 같은 회전을 중복 입력하지 않는다.
드라이버 TF는 실제 장착 위치를 표현하지 않으므로 주차 GUI 내부 좌표로 확인한다.

## 검증 범위

7개 패키지 빌드 및 기존 자동 검사 95개에 생존/지연 회귀 검사를 추가했다.
최신 검사 결과는 [INFERENCE_SERIAL_LIVENESS_KR.md](INFERENCE_SERIAL_LIVENESS_KR.md)에 기록한다.
생존 조건부 주기 송신, 생존/통신 만료 뒤 새 W 요구, W/S/종료 정지와 producer 잠금을 확인했다.
격리 ROS_DOMAIN_ID=143에서 5개 dry-run launch를 시작·종료하고 실제 ROS 서비스를
통한 headless GUI 변경/P 저장을 확인했다. 트랙은 launch speed=23→GUI speed=37,
합성 카메라 프레임에서 승인 파라미터=GUI 값=발행 좌/우 PWM=37이 일치했다.
fake sender 직렬 프레임도 `s-2l37r37\n`로 일치한다. 실제 모터 PWM은 미측정이다.

```bash
python3 -m pytest -q tools/tests/test_controller_liveness.py tools/tests/test_tuning_revision.py tools/tests/test_four_mode_launches.py tools/tests/test_drive_safety.py tools/tests/test_canonical_safety.py src/skku_track_drive_pkg/test/test_mission_parking.py src/skku_track_drive_pkg/test/test_lane_processing.py
ROS_DOMAIN_ID=143 ROS_LOG_DIR=/tmp/codex_four_modes_validation python3 tools/validate_mode_guis.py
python3 tools/bootstrap_mission_tuning.py
```

결과/화면/임시 YAML: `/tmp/codex_four_modes_validation/`.
트랙 속도의 별도 실제 GetParameters 재조회 결과는
`/tmp/codex_speed_validation/gui_validation.json`에 있으며 승인값·ROS 재조회·발행 PWM이 모두 37이다.
실제 센서 성능·지연, GUI 포커스, 모터 정지·속도·바퀴 각도, 실차 주차는 미검증이다.
기존 SciPy/NumPy 지원 버전 경고가 있으며 이번 작업에서 의존성을 임의로 바꾸지 않았다.
