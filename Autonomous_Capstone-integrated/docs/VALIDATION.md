# 검증 및 실차 시험 순서

작업 범위는 이 workspace뿐이다. 센서/ROS node/serial port를 실제로 시작하지 않았고,
펌웨어 upload 및 motor actuation은 수행하지 않았다. 실차 안전 인증 결과가 아니다.

| 검증 | 결과 | 한계 |
|---|---|---|
| `source /opt/ros/humble/setup.bash; colcon build --symlink-install` | 12개 package 성공: `Summary: 12 packages finished [49.0s]` | Python ament build만으로 모든 runtime 의존성을 검증하지는 못함 |
| Python AST syntax | src 및 test 91개 파일 성공 | 노드 실행 아님 |
| canonical module import | sensor camera/LiDAR, vehicle serial/arm, manual, track, configuration 7개 성공 | 객체 생성/모델 추론 안 함; legacy serial module은 import 부작용 때문에 제외 |
| `tools/tests/test_canonical_safety.py` | 7 tests 성공 | Mock serial; launch의 argument 선언만 실행, Node action 실행 안 함 |
| `tools/tests/firmware_safety_test.cpp` | g++ C++11 -Wall -Wextra -Werror compile 및 assert 성공 | 가짜 Arduino I/O host 시험, AVR compiler/전기 출력 검증 아님 |
| `git diff --check` | 성공 | 물리 기능 보증 아님 |
| arduino-cli | PATH에서 찾지 못함 | Arduino Mega 대상 compile 미수행; 설치/다운로드/업로드 안 함 |

Host firmware 시험: 부팅 및 query 후 PWM 0, 499/500 ms 경계, timeout 후 OFF 유지,
새 명령 재개, X 정지, 정상/오류/초과 길이/embedded NUL 입력, 큰 정수 거부,
ADC 증가/감소 방향의 LEFT/CENTER/RIGHT 및 중간값, runtime K center 보존,
보정 heartbeat/단절, unsigned millis rollover.
Python 시험: YAML profile/fallback/잘못된 구조, 기존 인자 override, gated launch의
자동 보정 기본 OFF, 수동 baseline 모드의 즉시 READY, 자동 보정의 CONFIG 대기,
DISARM X/보정 heartbeat/ARM command, baseline 검증,
중복 CONFIG가 자동 측정을 다시 시작하지 않는지 확인.

처음 launch 단위 테스트는 ROS 기본 로그 `/home/autolab/.ros/log` 쓰기 제한으로 실패했다.
로그를 `/tmp/canonical_ros_logs`로 지정해 재실행했다. 테스트가 발견한 빈 YAML 배열 허용
문제를 수정한 후 최종 5 tests가 통과했다.

Import 시 환경 경고: SciPy는 NumPy <1.25를 요구하지만 설치 NumPy는 1.26.4.
Import 자체는 성공했다. 이 작업에서 전역 환경/알고리즘은 변경하지 않았다.
향후 별도 환경 고정 및 recorded-data 수치회귀 확인이 필요하다.

재현 명령:

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ROS_LOG_DIR=/tmp/canonical_ros_logs python3 tools/tests/test_canonical_safety.py
g++ -std=c++11 -Wall -Wextra -Werror tools/tests/firmware_safety_test.cpp -o /tmp/canonical_firmware_test
/tmp/canonical_firmware_test
```

Arduino 도구 준비 후 운영자가 별도로 compile할 대상:
`src/control/driving_user_pins` 및 `src/control/steering_limit_calibration`.
보드가 Mega 2560인지 확인 후 `arduino:avr:mega` FQBN 사용 여부를 결정한다.
호스트 mock test 통과를 실제 보드 compile 성공으로 해석하지 않는다.

## 실차 검증 필요 항목과 순서

아래는 사람이 이후 수행할 시험 계획이며 이번에 실행하지 않았다.

1. 차량 고정/구동륜 부양, 물리 비상정지 확보. 현재 firmware/배선/기준 ADC를 백업하고
   Steering D3/D2, Right D4/D5, Left D7/D6, Pot A2를 확인한다.
2. Mega 대상 두 스케치 compile. 운영자가 검토 후 수동 upload한다.
   새 PC bridge와 driving_user_pins firmware를 한 쌍으로 적용한다. 구 firmware는
   X 이후 자체 중앙 복귀가 재개될 수 있으므로 새 bridge만으로 안전하다고 판단하지 않는다.
3. ARM/명령 없이 전원 투입 → serial 연결/재연결 → CONFIG 조회만 수행.
   구동/조향 PWM 모두 0이고 바퀴가 움직이지 않는지 확인한다.
4. 별도 진단 스케치에서 모터 정지 상태로 앞바퀴를 직진에 맞추고 p로 CENTER ADC를 측정한다.
   endpoint 측정 c는 작업자가 의도했을 때만 수행한다. RESULT_CENTER는 midpoint 추정이므로
   직진 실측값으로 대체한다. DEFAULT_LEFT/CENTER/RIGHT와 YAML 기록을 함께 갱신한다.
5. 장치 식별 후 `/dev/arduino`, `/dev/lidar` alias와 로컬 Front/Aux profile을 검증한다.
   센서 전용으로 두 카메라의 물리 방향, topic/QoS, LiDAR 180도 기준을 확인한다.
6. 기본 `auto_calibrate=false`로 수동 launch. READY 전에 ARM이 무시되고 launch 직후
   조향이 자동 움직이지 않는지 확인한다. READY 후 명시적 ARM에서만 저속 명령을 허용한다.
7. 부양 상태 저속에서 좌/우 구동 부호와 -7/0/+7 조향을 점진적으로 확인한다.
   X/SPACE/DISARM에서 구동과 조향 출력이 즉시 꺼지는지 확인한다.
8. 저속 명령 중 publisher 중단, serial bridge 강제 종료, USB 분리를 각각 시험한다.
   마지막 MCU 정상 frame 후 약 500 ms 이내 PWM 0인지 측정한다 (USB buffer 지연 별도 기록).
   실제 정지거리/관성/모터 드라이버 coast/brake 동작은 별도로 측정한다.
9. 선택적 `auto_calibrate=true` endpoint 시험: 허용 오차 이내 적용, 초과 시 LOCK,
   heartbeat 단절 시 조향 OFF, 보정 후 임의 출발 없음, operator ARM 필요를 확인한다.
10. bench capture 결과 이미지/라벨/부호를 확인하고 수동 저속 폐쇄구역 주행으로 확장한다.
11. 트랙/미션은 한 번에 하나만 선택해 저속 시험. command publisher 충돌, 추론 지연,
    입력 정지 시 상위 알고리즘의 stale command 반복 가능성을 확인한다.
12. commit/profile/firmware/model hash, 환경 버전, 운영자, 시험 결과를 기록한 뒤 리뷰하고
    develop/main merge 여부를 결정한다. legacy parking/main launch는 실차에서 사용하지 않는다.
