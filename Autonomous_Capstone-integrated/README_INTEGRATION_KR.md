# SKKU 자율주행 차량 Ubuntu/ROS2 통합본 v2

기준 자료:

- `Autonomous_Capstone-main`: 지난 학기 ROS2 Humble 워크스페이스
- `2026_skku_autodrive-final`: 여름방학 PyCharm/Windows 트랙주행 코드

현재 차량 배선:

```text
Steering  IN1=D3, IN2=D2
Right     IN1=D4, IN2=D5
Left      IN1=D7, IN2=D6
Pot OUT   A2
```

이번 v2의 핵심 변경:

1. 수동 WASD를 **누적형 제어**로 변경
   - `W`: 속도 +step
   - `S`: 속도 -step
   - `A`: 조향 한 단계씩 좌측
   - `D`: 조향 한 단계씩 우측
2. 최초 기준값 측정용 `steering_limit_calibration.ino` 추가
3. 트랙/미션 launch 시 Arduino가 자동으로 좌/우 조향 끝을 다시 측정
4. 측정값이 기존 기준값에서 **조금만 변했을 때만** PC가 런타임 보정값을 Arduino에 다시 전송
5. 자동 보정이 끝나도 차량은 출발하지 않으며, `W`를 눌러야 ARM되어 주행 시작
6. 보정값 차이가 너무 크거나 센서/기구 이상으로 판단되면 차량은 LOCK 상태 유지

---

## 0. Arduino: 최초 1회 기준 조향값 측정

먼저 다음 스케치를 Arduino Mega에 업로드한다.

```text
src/control/steering_limit_calibration/steering_limit_calibration.ino
```

차량이 굴러가지 않도록 고정한 뒤 Serial Monitor를 **115200 baud**로 열고 `c` + Enter.

자동 순서:

```text
LEFT 끝 측정
   ↓
RIGHT 끝 측정
   ↓
CENTER 복귀
   ↓
RESULT_LEFT / RESULT_RIGHT / RESULT_CENTER 출력
```

예:

```text
RESULT_LEFT=598
RESULT_RIGHT=447
RESULT_CENTER=522
```

그 값을 다음 파일의 기준값에 **한 번만** 반영한다.

```text
src/control/driving_user_pins/driving_user_pins.ino
```

```cpp
const int DEFAULT_LEFT = 598;
const int DEFAULT_RIGHT = 447;
```

그 뒤에는 `driving_user_pins.ino`를 업로드해서 계속 사용한다.

> 자동 보정은 매 실행마다 기준값을 영구 저장하는 방식이 아니다. 측정값이 기준값과 소폭 달라졌을 때 그 실행 동안만 runtime 값으로 적용한다.

---

## 1. 주행용 Arduino 펌웨어

업로드:

```text
src/control/driving_user_pins/driving_user_pins.ino
```

ROS2-PC와 Arduino 사이의 추가 calibration protocol:

```text
?              기준값 조회
C              자동 좌/우 끝 측정
KLEFT,RIGHT    PC가 승인한 runtime 보정값 적용
X              비상정지
```

일반 주행 명령은 기존 형식을 유지한다.

```text
s{steering}l{left_speed}r{right_speed}\n
```

조향 convention:

```text
-7 = 최대 좌측
 0 = 중앙
+7 = 최대 우측
```

---

## 2. 최초 환경 설치 / 빌드

```bash
cd ~/ros2_ws
chmod +x tools/*.sh
./tools/install_runtime.sh
sudo usermod -aG dialout $USER
```

로그아웃/로그인 후:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 3. 장치 찾기 / 포트 고정

```bash
cd ~/ros2_ws
./tools/list_devices.sh
```

직접 확인:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/ 2>/dev/null
lsusb
```

예: Arduino `/dev/ttyACM0`, LiDAR `/dev/ttyUSB0`

```bash
./tools/create_serial_alias.sh /dev/ttyACM0 arduino
./tools/create_serial_alias.sh /dev/ttyUSB0 lidar
```

재연결 후:

```bash
ls -l /dev/arduino
ls -l /dev/lidar
```

카메라는 가능하면 `/dev/v4l/by-id/...-video-index0` stable path 사용 권장.

---

## 4. 센서 단독 확인

### Camera

```bash
ros2 run sensor_bringup_pkg camera_publisher_node --ros-args \
  -p device:=/dev/video0
```

```bash
ros2 topic hz /image_raw
ros2 run rqt_image_view rqt_image_view
```

### LiDAR

```bash
ros2 run sensor_bringup_pkg lidar_publisher_node_v2 --ros-args \
  -p port:=/dev/lidar \
  -p rotation_offset_deg:=180.0
```

```bash
ros2 topic hz /lidar_raw
ros2 topic echo /lidar_raw --once
rviz2
```

RViz:

```text
Fixed Frame = laser_frame 또는 base_link
LaserScan Topic = /lidar_raw
```

---

## 5. 센서 한 번에 확인

```bash
ros2 launch vehicle_bringup_pkg sensor_check.launch.py \
  camera_device:=/dev/video0 \
  lidar_port:=/dev/lidar \
  lidar_rotation:=180.0
```

---

## 6. 수동 WASD + 자동 startup calibration + 라벨링

```bash
ros2 launch vehicle_bringup_pkg manual_capture.launch.py \
  camera_device:=/dev/video0 \
  arduino_port:=/dev/arduino \
  output_dir:=$HOME/ros2_ws/datasets/track_manual \
  speed_step:=20 \
  steering_step:=1
```

LiDAR도 동시에 켜려면:

```bash
... use_lidar:=true lidar_port:=/dev/lidar
```

launch 직후 순서:

```text
Arduino 기준값 조회
   ↓
좌측 끝 자동 측정
   ↓
우측 끝 자동 측정
   ↓
중앙 복귀
   ↓
기준값과 비교
   ↓
허용 오차 내 → runtime 보정 적용 → READY
허용 오차 초과 → LOCKED
```

기본 허용 오차:

```text
calibration_tolerance = ±35 ADC count
```

필요하면 launch에서 조정:

```bash
... calibration_tolerance:=25
```

### 누적형 WASD

카메라 창에 포커스를 둔다.

```text
W : speed += 20
S : speed -= 20
A : steering 한 단계 LEFT
D : steering 한 단계 RIGHT
X / SPACE : speed=0, steering=0, DISARM
C : 현재 프레임 수동 저장
R : 자동 캡처 ON/OFF
Q : DISARM 후 종료
```

예:

```text
초기 speed = 0
W → +20
W → +40
W → +60
S → +40
S → +20
S → 0
S → -20
S → -40
```

조향:

```text
초기 = 0
A → -1
A → -2
A → -3
D → -2
D → -1
D → 0
D → +1
```

자동 캡처는 `R`을 켠 상태에서 실제로 움직일 때만 저장한다.

결과:

```text
~/ros2_ws/datasets/track_manual/
├── images/
└── labels.csv
```

---

## 7. 트랙주행: launch → 자동 보정 → W 시작

```bash
ros2 launch vehicle_bringup_pkg track_drive.launch.py \
  camera_device:=/dev/video0 \
  lidar_port:=/dev/lidar \
  arduino_port:=/dev/arduino \
  device:=cpu \
  speed:=80
```

이 launch는 기본적으로:

```text
Camera
LiDAR
Track controller
Arduino serial/calibration node
W start safety gate
```

를 모두 켠다.

실행 직후 차량은 **절대 바로 출발하지 않는다**.

작은 `vehicle_start_gate` 창이 뜬다.

```text
CALIBRATING / LOCKED
     ↓
READY - PRESS W TO START
```

창에 포커스를 둔 뒤:

```text
W       실제 자율주행 시작(ARM)
X/SPACE 즉시 정지(DISARM)
```

보정 중 W를 눌러도 무시한다. READY가 된 뒤 다시 W를 눌러야 한다.

Debug:

```bash
ros2 run rqt_image_view rqt_image_view
```

`/track_debug_image` 선택.

상태 확인:

```bash
ros2 topic echo /vehicle/calibration_ready
ros2 topic echo /vehicle/calibration_status
ros2 topic echo /vehicle/armed
ros2 topic echo /topic_control_signal
```

---

## 8. 미션주행: 모든 센서/노드 + 자동 보정 + W 시작

```bash
ros2 launch launch_pkg mission_launch.py \
  camera_device:=/dev/video0 \
  lidar_port:=/dev/lidar \
  arduino_port:=/dev/arduino \
  lidar_rotation:=180.0
```

현재 safe mission launch 구성:

```text
Camera publisher
LiDAR publisher
YOLO
Traffic-light detector
Lane extractor
LiDAR processor
LiDAR obstacle detector
Mission manager
Path planner
Motion planner
Arduino serial + startup calibration
W start gate
```

여기도 순서는 동일하다.

```text
launch
 → 자동 steering calibration
 → READY
 → W
 → 실제 mission command가 Arduino로 전달됨
```

`X` 또는 `SPACE`를 누르면 serial bridge에서 최종 출력 자체를 차단하므로 상위 motion planner가 계속 명령을 내고 있어도 구동 명령은 전달되지 않는다.

---

## 9. 자동 보정이 거부되는 경우

예:

```text
LOCKED: Calibration rejected: endpoint shift is larger than allowed ...
```

이때는 자동으로 큰 변화를 받아들이지 않는다.

가능 원인:

```text
가변저항 홀더가 크게 이동함
가변저항 배선 문제
조향 기구 변경
기존 DEFAULT_LEFT/RIGHT가 다른 차량 값
끝점 검출 실패
```

다시 `steering_limit_calibration.ino`를 업로드하여 기준값을 측정하고 `DEFAULT_LEFT`, `DEFAULT_RIGHT`를 갱신한다.

> 매 launch마다 물리 끝까지 조향하는 과정이 있으므로 초기 테스트는 차량을 고정하고 낮은 PWM에서 확인한다. `CAL_STEERING_SPEED=90`에서 조향이 실제로 움직이지 않는 차량이라면 펌웨어의 값을 조금씩 올려 확인한다.
