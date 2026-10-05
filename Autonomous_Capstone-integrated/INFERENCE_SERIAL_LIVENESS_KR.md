# 추론 지연과 시리얼 송신 분리 (2026-10-04)

작업 디렉터리: `/home/autolab/autodrive_ws/Autonomous_Capstone-integrated`.
모터 구동·실제 센서 실행·시리얼 장치 연결·펌웨어 업로드는 수행하지 않았다.
위 문장은 생존 분리 작업 당시 범위다. 이후 내장 카메라/CUDA를 센서 전용으로 실측하고
트랙·미션에 유한한 인지 중단 정지를 연결했다. 2026-10-05부터 감속 없이 기존 6초 기준까지 명령을 유지한다. 최신 동작과 기록은
[PERCEPTION_DELAY_SAFETY_KR.md](PERCEPTION_DELAY_SAFETY_KR.md)를 참고한다.

## 변경과 적용 범위

앞선 새 추론 결과 전용 송신은 카메라/YOLO가 500ms 이상 지연될 때 Arduino의
통신 watchdog을 작동시켰다. 미션의 command timeout 750ms와 image timeout
750ms도 같은 추론 완료 지연을 정지 원인으로 취급했다.

네 단독 launch와 수동 주차 보정 launch는 이제 `require_controller_heartbeat=True`를
사용한다. 제어 노드마다 100ms 생존 신호 callback을 별도 callback group에서 실행하고
2개 worker의 executor를 사용한다. 생존 신호는 YOLO 완료, 이미지 수신, 파라미터/추론
lock에 의존하지 않는다. 프로세스 instance UUID와 현재 W challenge를 포함한다.
sender가 인지/추론 코드나 건강 신호를 대신 실행하지 않는다.

정상 주행 명령 callback은 최신 명령을 교체하기만 하고, sender의 기존 50ms timer가
20Hz로 송신한다. 단, 새 속도 0 명령은 timer를 기다리지 않고 즉시 적용하며 이후에도
0을 유지한다. 기존 조향/속도 계산 및 PWM 기본값은 변경하지 않았다.

생존 신호·두 UI 신호·READY·ARM이 모두 유효할 때만 마지막 명령을 재송신한다.
생존 만료/제어기 교체/종료/S/시리얼 오류는 disarm과 X 전송, challenge 교체로 이어진다.
정지 뒤 유효 명령과 생존 신호가 복구돼도 출발하지 않으며 새 W가 필요하다.
이전 challenge의 W/생존 신호는 거부하고, 만료를 검사한 다음 새 신호를 수용한다.

제어 프로세스 Ctrl+C/SIGTERM은 생존 신호를 먼저 비활성화한 뒤 cleanup한다.
진행 중인 추론을 기다리는 동안에도 생존 신호가 계속 송신되지 않는다.
SIGKILL/SIGSTOP 또는 executor의 처리되지 않은 예외는 생존 신호 만료로 처리한다.
전체 launch Ctrl+C는 기존 sender의 ROS와 독립적인 `X\n` 쓰기/포트 닫기 경로를 유지한다.

`dry_run`/`sensors_only` 생존 신호도 각 미리보기 토픽으로 격리한다.
수동 보정은 `/parking_calibration/vehicle/*`에 격리한다.
구형 launch/생산자는 기본 `require_controller_heartbeat=False`로 기존 명령 AGE 검사와
새 명령 전용 송신을 사용한다. 생존 신호 없는 생산자를 무한 재송신하도록 바꾸지 않았다.

## timeout 감사

이 문서의 생존 분리 변경에서는 기존 timeout 숫자를 줄이지 않았다. 이후 카메라/worker
중단 대응 요청에 따라 기존 20Hz sender timer에 6초 결과 중단 정지 조건을
연결했다. 현재 단계 감속은 제거하고 최종 정지 기준만 유지한다. 별도 timer는 만들지 않았으며 상세 근거는
[카메라·추론 중단 시 명령 유지와 정지](PERCEPTION_DELAY_SAFETY_KR.md)에 있다.

| 검사/주기 | 기존 값 | 현재 적용 대상·동작 |
|---|---:|---|
| 트랙 `command_timeout` | 0.0s | 추론 명령 AGE 검사 해제 유지. 제어 프로세스 생존에는 기존 `ui_timeout` 0.75s를 재사용하여 유한한 유지 시간 보장 |
| 미션 `command_timeout` | 0.75s | 마지막 추론 명령 시각 대신 독립적인 생존 신호의 수신 시각 검사 |
| 수직·평행주차 `command_timeout` | 0.25s | 독립적인 생존 신호 검사. 주차 scan/geometry/상태 제어는 그대로 |
| 수동 보정 `command_timeout` | 0.5s | 독립적인 생존 신호 검사 |
| 모든 모드 `ui_timeout` | 0.75s | gate 및 tuner의 키/GUI event loop 생존. 인지 FPS와 무관하며 만료 시 X/disarm |
| 생존 신호 / UI 신호 | 0.1s | 추론 완료와 독립적으로 발행 |
| 미션 `image_timeout_s` | 0.75s, GUI 조절/P 저장 유지 | 마지막 성공한 인지/미션 처리 이후 시간의 진단 임계값. `system_state=IMAGE_DELAY_HOLD`, `image_age_s`를 GUI/status에 표시. 더는 이 지연만으로 zero를 발행하지 않음 |
| 트랙·미션 `perception_stop_s` | 기존 최종 기준 6.0s | 유효결과가 중단되면 마지막 조향·PWM을 그대로 유지하다 X/disarm. 반복 생존 신호는 이 나이를 갱신하지 않음 |
| sender timer / write `timeout` | 0.05s / 0.05s | 주기 송신 / OS 시리얼 write 대기 상한. 후자는 인지 AGE timeout이 아님 |
| sender 송신 공백 검사 | 0.5s | sender 자체가 정상 motion 쓰기를 못 했을 때 기존 firmware watchdog 만료 가능성을 확인. 복구 시 먼저 disarm하여 자동 재출발 방지 |
| 두 펌웨어 `COMMAND_TIMEOUT_MS` | 500ms | Arduino가 마지막 유효 motion 프레임을 수신한 이후 통신 단절. 정상 제어기+느린 추론에서는 주기 송신으로 만료하지 않음 |
| 주차 `scan_timeout_s` | 0.5s | LiDAR 유효 입력 상실에 대한 기존 주차 판단. YOLO 지연과 무관하므로 유지 |

펌웨어는 `src/control/driving/driving.ino`, `driving_user_pins/driving_user_pins.ino`를
둘 다 확인했다. 유효 motion 프레임만 통신 시각을 갱신한다. 상태 조회/잘못된 프레임/
제어 loop 자체로 갱신하지 않으며 X와 통신 만료는 구동·조향 PWM을 모두 0으로 만든다.
이번 작업에서는 펌웨어 파일·핀·보정값·watchdog 숫자를 추가 수정하지 않았다.
실제 보드에 설치된 버전은 소프트웨어 소스로 확인할 수 없다.

## 정지 원인 구분

| 상황 | 동작 | 재출발 조건 |
|---|---|---|
| 카메라/YOLO 지연, 제어기·UI 정상 | 6초 미만 명령 유지→6초 기준 X/disarm. 미션 속도 0은 유지 | 완전 정지 전 최근 결과 복구, 완전 정지 후 새 성공 결과+새 W |
| 빨간불/초록불 대기/차단 장애물/안전한 회피 경로 없음 | 기존 MissionCore가 속도 0을 결정. 지연 진단과 별개로 해당 명령 유지 | 기존 미션 해제 판단(초록 확인/장애물 해제 등). ARM 유지 중의 미션 정지는 새 W를 추가 요구하지 않음 |
| 이미지 변환/추론/인지 처리 예외 | 기존 오류 로그와 zero MotionCommand 발행 유지. 느리게 완료되는 callback과 예외는 구분 | gate가 유효하면 정상 프레임 복구. 기존 정책 유지 |
| S / GUI 닫기 | 기존 gate→sender가 즉시 X/disarm. 이후 늦은 인지 명령으로 재출발 불가 | 복구된 명령/생존/UI/READY + 새 W |
| controller Ctrl+C/SIGTERM | 종료 생존 신호로 X/disarm. 전달 실패 시 유한한 생존 만료로 정지 | controller 복구 + 새 W |
| controller SIGKILL/SIGSTOP/프로세스 전체 고장 | 생존 만료 뒤 sender가 X/disarm. 더는 cached motion 재송신 안 함 | controller 복구 + 새 W |
| sender 전체 중단/SIGKILL/USB 단절 | firmware의 기존 500ms 통신 watchdog. sender가 살아 복구되면 기존 송신 공백 검사로 disarm | 원인 복구 후 정상 시작 + 새 W |

zero motion은 조향 목표를 유지/중앙 보정할 수 있다. 운영자 S/종료의 X는 모든 PWM을
끄는 명령이다. 로그/가짜 시리얼의 성공은 실제 PWM 0 또는 차량 관성 정지를 측정한 결과가 아니다.

## 검증과 실행

7개 패키지 빌드, Python 회귀 검사 105개 통과. 가짜 Arduino I/O로 두 스케치의
host C++ 시험도 통과했다. 20Hz 동일 명령 유지, X, 500ms 통신 만료를 검증했다.

ROS_DOMAIN_ID=144에서 실제 TrackController/MissionController 및 생산용 sender
callback/executor를 실행했다. 센서 대신 합성 Image, YOLO 대신 `sleep(2.0)` 검출기,
시리얼 대신 메모리 포트를 사용했다. 2초 지연 중 1.25초 관찰에서 새 인지 명령 없이
각 25회 motion 송신, 최대 송신 간격 0.0515/0.0507초를 확인한 뒤 지연 도중 S를 시험했다.
이후 1.2초 카메라 입력 공백에서도 ARM과 주기 송신을 유지했다.

| 실제 ROS 모의 검사 | 트랙 | 미션 |
|---|---:|---:|
| 추론 진행 중 controller Ctrl+C→disarm/X 관찰 | 0.0259s | 0.0953s |
| controller SIGSTOP→disarm/X 관찰 | 0.7763s | 0.7680s |
| controller SIGKILL→disarm/X 관찰 | 0.7922s | 0.7034s |

수치는 해당 머신의 관찰치이며 보장 시간 또는 실차 측정값이 아니다.
재실행의 최신 수치는 results.json에서 확인한다. 생존 만료의 기준은
마지막 수신 시각이며, timer 약 50ms와 OS/DDS 스케줄 지연이 더해진다.
S 후 늦은 인지 결과, 이전 W, 프로세스 교체, SIGCONT만으로 자동 재출발하지 않았고
새 W 뒤에만 재출발했다. 의도적인 속도 0도 지연 중 유지됐다.
추가 unit 검사에서는 10초 인지 공백 및 모든 모드의 기존 생존 만료 값을 검사했다.
기존 sender SIGINT/SIGTERM 검사에서 ROS context 종료 후에도 X 쓰기→close 순서를 확인했다.
네 모드 및 수동 보정의 5개 dry-run launch도 재실행하여 실제 ROS 파라미터 서비스,
headless GUI 승인값/조회값 일치 및 P 저장을 확인했다. 기존 SciPy/NumPy 경고 1개는 유지됐다.

```bash
cd ~/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash
python3 -m pytest -q tools/tests/test_controller_liveness.py tools/tests/test_drive_safety.py tools/tests/test_canonical_safety.py tools/tests/test_tuning_revision.py tools/tests/test_four_mode_launches.py src/skku_track_drive_pkg/test/test_mission_parking.py src/skku_track_drive_pkg/test/test_lane_processing.py
ROS_DOMAIN_ID=144 ROS_LOG_DIR=/tmp/autodrive_liveness_validation/ros_logs python3 tools/validate_inference_liveness.py
```

결과: `/tmp/autodrive_liveness_validation/results.json`, 모드별 controller 로그와 모의 송신 기록.
검증 도구는 ROS_DOMAIN_ID=144 이외에는 시작을 거부한다.

기존 실행 명령/인자는 유지한다. 각 launch에서 새 생존 신호를 자동 연결하므로 별도 인자가
필요 없다. 실제 센서/GUI 확인 시 차량 serial sender 없이 다음 명령을 각각 사용한다.

```bash
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py sensors_only:=true
ros2 launch vehicle_bringup_pkg mission_drive_tuning.launch.py sensors_only:=true
ros2 launch vehicle_bringup_pkg perpendicular_parking.launch.py sensors_only:=true
ros2 launch vehicle_bringup_pkg parallel_parking.launch.py sensors_only:=true
```

`dry_run:=true`는 기존처럼 카메라·LiDAR·시리얼·gate 모두 제외한다.
`sensors_only:=true`는 해당 모드 센서와 GUI만 연결하며 차량 serial/gate는 실행하지 않는다.

## 남는 한계

- 생존 자체는 새로운 장면의 인지 증명이 아니다. 이후 추가한 성공 결과 sequence/프레임
  나이 검사로 카메라/worker가 중단된 경우 기존 6초 기준에서 X/disarm한다.
  동일 영상이 새 입력/성공 결과로 반복되는 영상 고정은 이 검사로 구분하지 않는다.
- Python GIL을 장시간 점유하는 native 함수, OS 전체 정지, executor 전체 고장은 독립
  callback도 막을 수 있다. 이 경우 생존 만료 또는 firmware 통신 watchdog 정지가 가능하다.
  Python thread 방식은 실시간 OS나 독립 하드웨어 제어기가 아니다.
- controller만 종료하면 sender가 살아 있는 동안 기존 생존 시간 이내의 마지막 명령이
  남을 수 있다. sender까지 죽으면 마지막 시리얼 프레임 뒤 500ms firmware 정지가 필요하다.
- 실제 YOLO/CUDA 부하, USB/DDS 지연, OpenCV 키 포커스, 설치 firmware, 모터 출력과
  실차 제동/관성은 미검증이다. 실제 GUI/통신이 지연되면 S 전달도 지연될 수 있다.
  바퀴를 띄운 기존 W/S·Ctrl+C 실차 확인 절차는 그대로 필요하다.
