# 트랙 시작·정지 수정 및 검증 보고 (2026-10-04 갱신)

속도 설정·센서 전용·주차 보정의 최신 변경과 실행 명령은
[실차 튜닝 전 검증 보고](REAL_VEHICLE_TUNING_PREP_KR.md)를 함께 확인한다.

## 확인한 실행 경로와 원인

현재 체크아웃의 `vehicle_bringup_pkg/launch/track_drive_tuning.launch.py`와
빌드된 entry point를 모두 확인했다. 설치된 launch/Python 모듈도 이 워크스페이스
`src`를 가리킨다. 조사 시 호스트에서 해당 ROS 주행 프로세스는 발견하지 못했으며,
증상이 발생했을 때의 키 수신·토픽·시리얼 로그와 업로드된 펌웨어는 확인하지 않았다.
보드 포트를 열거나 모터를 구동하지 않았다.

| 단계 | 현재 실행 파일/경로 |
|---|---|
| 카메라 | sensor_bringup_pkg / camera_publisher_node → camera_publisher_node.py |
| LiDAR (use_lidar=true일 때) | sensor_bringup_pkg / lidar_publisher_node_v2 → lidar_publisher_node.py |
| 제어 | skku_track_drive_pkg / track_controller_node → track_controller_node.py |
| 튜닝 | skku_track_drive_pkg / track_tuner_node → track_tuner_node.py |
| 시작·정지 | vehicle_io_pkg / drive_arm_node → drive_arm_node.py |
| 송신 | vehicle_io_pkg / serial_sender_node_v2 → serial_sender_node.py:main |

수정 전 코드에서 확인한 사실:

- `vehicle_start_gate`는 W와 X/Space만 처리했다. S는 처리하지 않았다.
- `Track Tuner`/`Track Debug View` 프로세스의 `waitKey()`는 P/D/B/Q만 처리했다.
  이 창에 포커스를 둔 채 X/Space를 눌러도 disarm 메시지가 나가지 않았다.
- 송신 노드는 `vehicle/armed`의 Bool을 저장했다. 제어 명령 timeout 및 UI 생존 확인이
  없어서 마지막 True가 남고, 명령이 끊겨도 별도로 정지하지 않았다.
- 송신 노드는 평상시 정지와 종료에 `X\n`을 보냈지만, 기존 `driving.ino`는 X를
  처리하지 않았고 watchdog도 없었다. 이 펌웨어가 업로드돼 있었다면 증상과 일치한다.
- `driving_user_pins.ino`는 이미 X와 500 ms watchdog을 지원했다. 실제 보드가
  이 버전인지 확인하지 않았으므로 펌웨어 불일치를 실제 원인으로 단정하지 않는다.
- 기존 종료 코드도 X 전송을 시도했다. 실제 Ctrl+C 실패가 종료 코드 실행 실패인지,
  시리얼 전송 실패인지, 보드의 X 미지원인지는 현장 로그 없이 확정할 수 없다.

## 수정 동작과 로그

`vehicle_start_gate`, `Track Tuner`, `Track Debug View` 중 하나에 포커스를 두고
W/w로 시작, S/s로 정지한다. launch 터미널의 일반 키 입력은 사용하지 않는다.
X/Space도 정지 별칭으로 유지했다. S는 ROS·카메라·튜닝 화면을 종료하지 않는다.
Q/ESC 또는 튜닝 창 닫기는 disarm을 요청하고 튜닝 heartbeat를 중단한다.
튜닝 창을 닫은 뒤에는 튜너를 다시 실행해야 이 launch의 시작 조건을 만족한다.

흐름은 다음과 같다.

```text
튜닝/디버그 W/S → vehicle/drive_key → drive_arm_node
게이트 창 W/S ────────────────────→ drive_arm_node
  → vehicle/armed (False는 즉시 정지 요청, True만으로는 시작 불가)
  → vehicle/arm_request (W 시점의 challenge)
  → serial_sender_node_v2 (READY + 신선한 명령 + UI 생존 확인)
  → 주행 s{steering}l{left_speed}r{right_speed}\n 또는 정지 X\n
  → Arduino 유효 프레임 파싱 → 구동/조향 PWM
```

송신 노드는 초기 disarm이며 기존 calibration_ready 조건과 새 W가 모두 필요하다.
기존 `auto_calibrate=false` 정책은 유지한다. 이 경우 READY는 설치된 펌웨어 보정값을
신뢰한다는 뜻이며, 실제 보드의 보정/프로토콜 확인이 완료됐다는 뜻은 아니다.

S, UI timeout, 시리얼 오류/송신 중단에서 disarm하고 challenge를 교체한다.
인지 명령 AGE timeout은 이 launch에서 기본 해제하고 독립적인 제어기 생존을 검사한다.
그 뒤 주행 명령, 오래된 Bool True, 이전 challenge의 W가 도착해도 출발하지 않는다.
W는 당시 challenge를 갖는 일회성 요청이며, heartbeat는 시작 요청을 반복하지 않는다.
송신 노드 재시작에도 challenge가 바뀌므로 저장된 True로 출발하지 않는다.

두 UI가 각각 0.1초 간격으로 heartbeat를 보내며, 이 launch는 둘 다 요구한다.
기본 `command_timeout=0.0`은 제어 명령의 AGE 검사만 해제한다.
최신 변경에서는 제어기 생존 신호를 별도 callback group/executor worker에서 발행하고,
이 신호가 살아 있을 때만 마지막 명령을 20Hz로 송신한다. 인지 지연과 종료를 구분한다.
command_timeout=0에서는 기존 ui_timeout=0.75초로 생존을 검사한다. 제어기 종료/고장은
종료 신호 또는 생존 만료로 X/disarm한다. sender 고장은 firmware watchdog으로 보완한다.
S 이후와 watchdog 정지 후에는 유효 명령 수신과 새 W가 모두 필요하다.
초기에도 유효 명령을 한 번 받기 전 W는 거절한다.

UI timeout 0.75초와 안전 확인 timer 0.05초는 유지한다. UI timeout은 heartbeat 약
7.5주기를 허용한다. 또한 송신 노드 자체의 구동 프레임 송신이 0.5초 이상 멈췄다면
Arduino watchdog이 이미 PWM을 껐을 수 있으므로, 복구 뒤 반복 송신 대신 disarm한다.
이 통신 중단 검사는 영상 처리 간격과 별개이며 새 W 없이 자동 재출발하지 않는다.

`command_timeout:=0.5`처럼 양수를 지정하면 해당 값으로 **생존 신호** 만료를 검사한다.
다른 launch에 영향을 주지 않도록 송신 노드 자체의 기본값은 기존 0.5초를 유지했다.
timeout이 켜진 경우 새 명령/heartbeat가 만료 뒤 먼저 도착해도 기존 만료를 검사한다.

로그는 `KEY received`, `ARM state=False/True`, `STOP frame X: written to serial/write failed`
및 종료 시 `shutdown STOP X write=...`를 구분한다. 이는 요청 수신/소프트웨어 상태/
시리얼 쓰기 결과이며 실제 PWM이나 차량 정지 확인 로그가 아니다.
실제 송신 상태 확인에는 `/vehicle/drive_state`를 사용한다. `/vehicle/armed`의 True는
요청 상태이므로 그것만으로 송신 노드가 승인했다고 판단하지 않는다.

SIGINT/Ctrl+C, SIGTERM, 예외 및 생성 도중 실패에서도 ROS publish와 별개로 포트를
닫기 전에 X 쓰기를 시도한다. 쓰기는 기존 0.05초 write_timeout을 사용한다.
SIGKILL/USB 분리로 X를 보낼 수 없으면 Arduino watchdog이 보완한다.
시리얼 오류는 해당 송신 프로세스를 잠근다. 연결을 복구한 뒤 launch를 다시 실행하고
READY를 확인하여 새 W를 눌러야 한다. 자동 연결 복구/자동 출발은 하지 않는다.

## Arduino 변경과 업로드

두 스케치를 안전 처리 범위에서 수정했다. 핀·조향 부호·PWM 주행 게인·보정값은 유지했다.

| 파일 | 변경/유지 내용 | 보존한 배치/보정 |
|---|---|---|
| `src/control/driving/driving.ino` | X 처리, 초기 PWM 0, 500 ms watchdog 추가. 제어 시각/유효 명령 수신 시각 분리. 전체 프레임 범위/종료/overflow/NUL 검사 | 조향 D2/D3, 오른쪽 D4/D5, 왼쪽 D7/D6, POT A2, 끝값 430/275 |
| `src/control/driving_user_pins/driving_user_pins.ino` | 기존 X/500 ms watchdog 재사용. 정지 때 중앙 목표 변경 제거. 보정 중 차단된 일반 주행 프레임은 보정 heartbeat를 갱신하지 않음 | 조향 D3/D2, 오른쪽 D4/D5, 왼쪽 D7/D6, POT A2, LEFT/CENTER/RIGHT 440/355/270 |

정지는 좌우 구동과 조향의 모든 PWM 출력을 0으로 만들며 조향을 중앙으로 움직이지 않는다.
잘못된 프레임, 상태 조회, 제어 loop 실행 자체는 주행 watchdog 시각을 갱신하지 않는다.
유효 시리얼 프레임이 없으면 Arduino는 마지막 정상 프레임 후 500 ms에 PWM을 끈다.
생존이 유효하면 반복 송신하여 인지 지연만으로 통신 watchdog이 만료하지 않게 한다.
제어 노드 종료/SIGKILL은 sender의 생존 검사로 X/disarm한다. 송신 노드 종료/SIGKILL,
USB 분리로 프레임이 끊기면 firmware watchdog이 작동한다.
이번 생존 조건부 송신 변경에서 Arduino watchdog이나 핀/보정값을 추가 변경하지 않았다.

**실제 보드 버전을 확인해야 하며 이번 작업에서 업로드하지 않았다.**
현재 스케치가 지원 정지 동작을 갖는지 확인한 뒤 업로드 필요 여부를 결정한다.
저장소 안내의 주행용 파일은 `driving_user_pins.ino`이지만 두 스케치의 조향 핀 순서와
보정값이 다르므로 파일을 무조건 바꿔 업로드하면 안 된다. 기존 driving 배치라면 수정된
`driving.ino`를 사용한다. 이 스케치는 자동 보정 프로토콜이 없으므로 `auto_calibrate=false`로
사용한다. 두 파일을 한 스케치에 합치지 않는다.

## 소프트웨어 검증

- 세 ROS 패키지 `vehicle_io_pkg`, `skku_track_drive_pkg`, `vehicle_bringup_pkg` 빌드 성공.
- 설치된 entry point와 import 경로 확인, `ros2 launch ... --show-args` 확인. 하드웨어 노드는 시작하지 않음.
- Python mock/회귀 테스트: 40개 통과. W→주행→S→명령 계속→정지 유지→새 W,
  초기 disarm, READY 차단, 늦은 명령/heartbeat, 각 UI timeout, 오래된 True/W,
  초기 STOP 쓰기 실패, 시리얼 오류/짧은 쓰기, 튜닝 키 대소문자 및 창 닫기 경로를 확인.
- 최신 생존 모드: 인지 지연 동안 생존 조건부 20Hz 송신, 생존 만료에서 정지,
  최초/정지 후 유효 명령 + 새 W 조건, UI 종료/시리얼 오류/종료 정지, 잘못된 명령 무시,
  송신 노드 정지 후 Arduino watchdog 시간 초과 시 자동 재출발 차단을 확인.
- fake serial을 가진 별도 프로세스에 실제 SIGINT/SIGTERM을 보내 X 쓰기→포트 닫기 순서를 확인.
  ROS context 종료 후와 생성 도중 예외에서도 같은 정지 시도를 확인.
- 두 .ino를 C++ host fake Arduino I/O로 각각 컴파일·실행. 초기 전체 PWM 0, X,
  정확한 500 ms 경계, 잘못된/과대/NUL 프레임, 조회, 동일 명령 재수신, 시각 wraparound,
  user_pins 보정 heartbeat 경로, 두 펌웨어의 20 Hz 동일 프레임 10초 수신 유지와
  그 뒤 X/통신 watchdog 정지를 확인.
- 기존 펌웨어 테스트의 오래된 보정 기대값을 현재 보존된 baseline에 맞게 수정했다.
- `git diff --check` 통과. Arduino AVR toolchain/보드용 빌드는 수행하지 못했다.
- 실제 OpenCV 포커스 동작, DDS 전달 지연, USB 장애, 실제 모터 PWM/차량 동작은 미검증.
  테스트 과정의 기존 SciPy/NumPy 버전 경고는 발생했으나 테스트 실패는 없었다.

### 일시적 처리 지연과 timeout 판단

트랙 기본 command_timeout=0.0 값은 유지했다. 무조건 재송신과 새 추론 전용 송신의
한계를 해결하기 위해 독립적인 생존 신호가 유효할 때만 주기 송신한다.
추론 지연 중 마지막 유효 조향·PWM을 유지하고, 기존 6초 인지 중단 기준에서 X/disarm한다.
제어 프로세스 종료는 기존 생존 만료로 X/disarm하며 S/Ctrl+C 경로를 유지한다.
새 감사 표·실제 ROS 모의 검증·남는 한계는
[INFERENCE_SERIAL_LIVENESS_KR.md](INFERENCE_SERIAL_LIVENESS_KR.md)를 참고한다.
최신 성공 결과 guard 및 실제 간격 측정은 [PERCEPTION_DELAY_SAFETY_KR.md](PERCEPTION_DELAY_SAFETY_KR.md).

`tools/measure_control_gaps.py`는 명령 토픽을 구독하여 수신 간격 p95/p99/max와
마지막 수신 이후 무응답 시간을 측정한다. 시리얼을 열거나 명령/arm을 발행하지 않는다.
이미 실행 중인 launch를 disarm 상태로 두고 W 없이 대표 부하에서 측정한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/measure_control_gaps.py --duration 60
```

별도 구독자의 측정이므로 송신 노드의 정확한 수신 시각을 보장하지는 않는다.
측정 도구 자체는 격리한 ROS domain의 가상 토픽에서 검증했다. 정상 주기 사이에
약 120 ms 공백을 넣어 최대 간격/기준 초과 집계와 마지막 수신 뒤 무응답 검출을 확인했다.
실제 명령 스트림의 측정값은 얻지 못했다. 이 도구는 처리 지연 분석이나 나중에 양수
command_timeout을 다시 사용할 때 참고할 수 있다. 측정 도구의 500 ms 비교 기준은
진단용이며 timeout 숫자를 변경하지 않는다. 지원 펌웨어의 500ms 통신 watchdog은 별도로 유효하다.

재현 명령 (워크스페이스 루트에서 실행):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/track_drive_safety_logs
python3 -m unittest discover -s tools/tests -p 'test_*.py' -v

g++ -std=c++17 -Wall -Wextra -pedantic tools/tests/firmware_safety_test.cpp -o /tmp/firmware_safety_user_pins
/tmp/firmware_safety_user_pins
g++ -std=c++17 -Wall -Wextra -pedantic -DTEST_LEGACY tools/tests/firmware_safety_test.cpp -o /tmp/firmware_safety_legacy
/tmp/firmware_safety_legacy
```

빌드·실행:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select vehicle_io_pkg skku_track_drive_pkg vehicle_bringup_pkg
source install/setup.bash
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py \
  command_timeout:=0.0 ui_timeout:=0.75
```

카메라/Arduino 포트와 CUDA 장치는 기존 vehicle 설정/launch 인자를 사용한다.
차선 인지, 조향 제어 알고리즘, 주행 게인은 변경하지 않았다.
사용자가 설정한 기존 시각화 옵션은 보존했다.

## 바퀴를 띄운 실차 시험 (사용자가 수행)

차량을 고정하고 구동 바퀴를 바닥에서 띄운 뒤 실제 설치 펌웨어의 정지 기능을 확인한다.
업로드는 자동 수행하지 않는다. 트랙 기본 PWM은 250이며 첫 시험은 명시 speed로 낮춘다.

1. launch 직후 W 없이 구동·조향 PWM 0을 확인한다. READY 이전 W가 거절되는지 확인한다.
   auto_calibrate=true를 명시한 경우 보정 중 조향 구동은 별도 의도된 작업이다.
2. 각 `vehicle_start_gate`/`Track Tuner`/`Track Debug View`에 포커스를 두고 W/w, S/s를
   확인한다. 키 수신→ARM 상태→X 쓰기 로그를 대조하고 S 후에도 카메라/튜닝 창이 살아있는지 확인한다.
3. S 이후 `/topic_control_signal`이 계속 발행되어도 구동·조향 PWM 0을 유지하는지,
   새 W 뒤에만 다시 구동하는지 확인한다.
4. W 뒤 launch 터미널 Ctrl+C를 누르고 종료 X 전송과 PWM 0을 확인한다.
5. W 뒤 인지 지연만 발생시키면 20Hz 송신과 기존 명령 유지가 가능한지 확인한다.
   제어 노드만 종료하면 종료 신호 또는 생존 만료(트랙 기본 0.75초)로 X/disarm해야 한다.
   제어 노드를 복구해도 새 W 이전에는 구동하지 않아야 한다.
6. 게이트 UI와 튜너 UI를 각각 종료하여 0.75초 timeout/disarm을 확인한다.
   UI 복구만으로 구동하지 않아야 한다. 송신 노드의 SIGTERM과 SIGKILL도 각각 확인한다.
   SIGKILL은 종료 X 로그가 없으며 펌웨어 watchdog으로 마지막 명령 후 약 0.5초에 PWM 0이어야 한다.
7. Arduino가 모터 전원으로 계속 켜진 상태에서 USB/시리얼 연결을 끊어 watchdog을 확인한다.
   연결 복구 후 launch 재실행·READY·새 W가 필요하다.
8. PWM 0은 계측 또는 해당 출력 확인으로 검증한다. 정지 프레임 쓰기 로그만으로 실제
   모터 정지를 판정하지 않는다. PWM 0 이후 차량의 관성 정지까지 보장하는 시험이 아니다.
