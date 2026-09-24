# 자율주행 프로젝트 인수인계 분석 보고서

> 조사 기준일: 2026-09-23 (Asia/Seoul)  
> 조사 위치: `/home/autolab/autodrive_ws/h-mobility-class`  
> 실제 Git 최상위 디렉터리: `/home/autolab/autodrive_ws`  
> 주의: 이 문서는 당시 로컬 파일과 로컬 Git 참조만을 근거로 작성했다. `git fetch`/`pull`은 수행하지 않았으므로 원격 서버의 최신 상태까지 보장하지 않는다.

# 1. 프로젝트 한 줄 설명

카메라·LiDAR 입력을 ROS 2 노드에서 인식·경로계획·제어 명령으로 변환하고, 직렬 통신을 통해 Arduino 기반 조향/구동 모터를 제어하는 소형 자율주행 차량 프로젝트다.

현재 저장소에는 과거 교육/개발용 워크스페이스인 `h-mobility-class`와, 후속 통합 워크스페이스인 `Autonomous_Capstone-integrated`가 동시에 존재한다. 로컬 문서상 후자가 현재의 기준 구현(canonical workspace)이다.

# 2. 현재 Git 상태

## 확인된 사실

- Git root: `/home/autolab/autodrive_ws`
- 현재 branch: `main`
- 현재 HEAD: `6ffd1e9b4d03a12f544f92565df2421bf22d2a81`
- HEAD 요약: `Add vehicle config and fix steering motor control`
- 로컬 `origin/main`과 현재 `main`의 관계: ahead 0 / behind 0
- remote:
  - `origin https://github.com/chomi14/autodrive-capstone.git`
- staged 변경: 없음
- unstaged tracked 변경: 15개 파일, 총 232 insertions / 86 deletions
- untracked: 축약 경로 기준 10개, 실제 파일 기준 18개
- 변경은 주로 `Autonomous_Capstone-integrated` 아래에 있으며, Git root에는 `h-mobility-class_7233.zip`도 untracked 상태로 존재한다.
- `git diff --check`와 `git diff --cached --check`에서 whitespace 오류는 발견되지 않았다.

## 중요 해석

로컬 `main`과 로컬에 저장된 `origin/main` 참조가 같은 commit을 가리키지만, 이번 조사에서는 `git fetch`를 하지 않았다. 따라서 GitHub의 실제 최신 `origin/main`과 동일하다는 뜻은 아니다. 확인하려면 사용자가 다음을 실행해야 한다.

```bash
cd /home/autolab/autodrive_ws
git fetch origin
git status --short --branch
git log --oneline --decorate --graph --all -n 20
```

## 최근 commit history

| 순서 | Commit | 작성 시각 | 작성자 | 메시지 |
|---|---|---|---|---|
| 1 | `6ffd1e9` | 2026-09-19 15:18 KST | chomi14 | Add vehicle config and fix steering motor control |
| 2 | `90cdec5` | 2026-09-18 | zlyoon | Replace h-mobility-class with project files |
| 3 | `a83c821` | 2026-09-18 | zlyoon | Add h-mobility-class code |
| 4 | `8ccf79a` | 2026-09-18 | - | merge commit |
| 5 | `b6d6ee3` | 이전 | - | integrated workspace 초기 추가 |
| 6 | `6531e2d` | 이전 | - | repository 초기 commit |

## 최근 commit에서 변경된 영역

`6ffd1e9`는 `h-mobility-class` 아래 12개 파일을 변경했다. 주요 영역은 다음과 같다.

- 카메라 인식 노드 및 라이브러리 로딩 방식
- `driving.ino`의 조향 모터 제어
- path/motion planning 코드
- `launch_pkg`의 `CMakeLists.txt`, YAML 및 main launch
- 구형 serial sender

특히 `h-mobility-class` 쪽 카메라 라이브러리의 `__init__.py`는 일반 Python import 방식으로 정리됐지만, 같은 문제가 있는 `Autonomous_Capstone-integrated` 쪽 패키지에는 해당 수정이 반영되지 않았다.

# 3. 전체 디렉터리 구조

```text
/home/autolab/autodrive_ws/                 # 실제 Git root
├── README.md                               # 저장소 최상위 소개(현재 한 줄 수준)
├── h-mobility-class/                       # 현재 작업 위치, 과거/교육용 ROS 2 workspace
│   ├── src/                                # 7개 ROS 2 package 및 Arduino 관련 코드
│   ├── build/ install/ log/                # 기존 colcon 산출물
│   └── PROJECT_HANDOVER_ANALYSIS.md        # 본 인수인계 문서
├── Autonomous_Capstone-integrated/         # 문서상 현재 기준 통합 workspace
│   ├── README_INTEGRATION_KR.md             # 통합 구성 및 안전 운용 설명
│   ├── docs/                               # 신규 안전/운용 문서(untracked 포함)
│   ├── tools/                              # 장치 확인, alias, 의존성 설치, 테스트 도구
│   ├── tests/                              # host-side firmware safety test 등
│   ├── src/
│   │   ├── interfaces_pkg/                 # custom ROS message 16종
│   │   ├── sensor_bringup_pkg/             # 카메라/LiDAR 입력
│   │   ├── vehicle_io_pkg/                 # serial bridge 및 arm UI
│   │   ├── manual_drive_pkg/               # 수동 주행/데이터 수집
│   │   ├── skku_track_drive_pkg/           # 트랙 자율주행 단일 pipeline
│   │   ├── vehicle_bringup_pkg/             # 기준 launch/config
│   │   ├── camera_perception_pkg/           # mission 카메라 인식
│   │   ├── lidar_perception_pkg/            # mission LiDAR 인식
│   │   ├── decision_making_pkg/             # mission 판단/계획
│   │   ├── debug_pkg/                       # 시각화/디버깅
│   │   ├── launch_pkg/                      # 과거 통합 launch, 일부 compatibility 경로
│   │   ├── serial_communication_pkg/        # 구형 serial 통신
│   │   └── control/                         # Arduino firmware/sketch
│   └── build/ install/ log/                # 기존 colcon 산출물(현재 source보다 오래됨)
└── h-mobility-class_7233.zip               # untracked 백업 추정 파일
```

## 확인된 workspace 구분

`Autonomous_Capstone-integrated/README_INTEGRATION_KR.md` 및 관련 문서가 명시하는 기준 조합은 다음과 같다.

- ROS launch/config: `vehicle_bringup_pkg`
- PC↔Arduino bridge: `vehicle_io_pkg`
- Arduino firmware: `src/control/driving_user_pins/driving_user_pins.ino`
- 공유 설정: `src/vehicle_bringup_pkg/config/vehicle.yaml`

문서가 legacy 또는 위험 경로로 구분한 항목은 다음과 같다.

- `launch_pkg/main_launch.py`
- `launch_pkg/parking_launch.py`
- `serial_communication_pkg`
- 과거 `data_collection` 경로

다만 문서 일부는 현재 코드와 값이 맞지 않아, 문서를 단독 진실 공급원으로 사용하면 안 된다.

# 4. 주요 파일별 역할

## 통합 workspace의 핵심 파일

| 파일/경로 | 역할 |
|---|---|
| `Autonomous_Capstone-integrated/README_INTEGRATION_KR.md` | 기준 workspace, 안전 gate, launch 종류, 하드웨어 운용 개요 |
| `src/interfaces_pkg/msg/MotionCommand.msg` | 조향·좌/우 모터 명령(`int32 steering`, `left_speed`, `right_speed`) 정의 |
| `src/sensor_bringup_pkg/sensor_bringup_pkg/camera_publisher.py` | V4L2 카메라 프레임을 ROS Image로 발행 |
| `src/sensor_bringup_pkg/sensor_bringup_pkg/lidar_publisher.py` | RPLidar 데이터를 LaserScan으로 발행하고 TF 제공 |
| `src/manual_drive_pkg/manual_drive_pkg/manual_drive_capture.py` | 키보드 수동 주행, 명령 발행, 이미지/명령 데이터셋 저장 |
| `src/vehicle_io_pkg/vehicle_io_pkg/serial_sender_node.py` | `MotionCommand`를 Arduino serial protocol로 변환하고 calibration/arm gate 관리 |
| `src/vehicle_io_pkg/vehicle_io_pkg/drive_arm_node.py` | W/X/Space/Q 기반 arm/disarm UI |
| `src/skku_track_drive_pkg/skku_track_drive_pkg/track_controller_node.py` | 트랙 인식부터 계획, 조향 명령 생성까지 연결하는 상위 ROS 2 node |
| `src/skku_track_drive_pkg/.../yolo_detector.py` | YOLO 기반 차선/표식 검출 |
| `src/skku_track_drive_pkg/.../lane_info_extractor.py` | 검출 결과에서 차선 정보를 추출 |
| `src/skku_track_drive_pkg/.../path_planner.py` | CubicSpline 기반 주행 경로 생성 |
| `src/skku_track_drive_pkg/.../motion_planner.py` | Stanley 방식으로 조향값 및 속도 명령 계산 |
| `src/vehicle_bringup_pkg/config/vehicle.yaml` | 장치 경로, baudrate, 카메라, LiDAR, 조향 calibration 공유 설정 |
| `src/vehicle_bringup_pkg/launch/bench_capture.launch.py` | serial 없이 카메라/데이터 수집을 시험하는 bench launch |
| `src/vehicle_bringup_pkg/launch/sensor_check.launch.py` | 카메라와 LiDAR bring-up 점검 |
| `src/vehicle_bringup_pkg/launch/manual_capture.launch.py` | 센서, serial bridge, 수동 운전/수집 node 연결 |
| `src/vehicle_bringup_pkg/launch/track_drive.launch.py` | 트랙 자율주행 pipeline, serial bridge, arm UI 연결 |
| `src/launch_pkg/launch/mission_launch.py` | mission 인식/판단/계획과 gated serial bridge를 연결하는 호환 launch |
| `src/control/driving_user_pins/driving_user_pins.ino` | 현재 기준 Arduino 조향/구동 firmware |
| `src/control/steering_limit_calibration/steering_limit_calibration.ino` | 조향 좌/우 limit 및 center 측정용 sketch |
| `src/control/driving/driving.ino` | watchdog가 없는 과거 firmware 경로 |
| `tools/install_runtime.sh` | ROS/system/Python runtime 의존성 설치 보조 스크립트 |
| `tools/list_devices.sh` | 카메라/serial/LiDAR 장치 확인 |
| `tools/create_serial_alias.sh` | `/dev/arduino`, `/dev/lidar` alias 설정 보조 |
| `tests/firmware_safety_host_test.cpp` | Arduino parser/control logic의 host-side 안전성 검사 |

## 패키지 메타데이터

각 ROS 2 패키지의 `package.xml`, `setup.py` 또는 `CMakeLists.txt`가 빌드 및 설치 대상을 선언한다. 그러나 실제 Python import 의존성과 manifest 선언이 완전히 일치하지 않는 부분이 있으므로 `colcon build` 성공만으로 runtime 정상 여부를 판단할 수 없다.

별도 `requirements.txt`, `pyproject.toml`, Docker 설정, `.repos` 파일은 발견되지 않았다. 의존성 정보는 각 `package.xml`, Python setup, 설치 스크립트에 분산돼 있다.

# 5. 코드/노드 관계

## 기준 트랙 주행 경로

```mermaid
graph TD
    CAM[CameraPublisherNode<br/>/camera/front/image_raw] --> TRACK[TrackControllerNode]
    TRACK --> YOLO[YoloDetector]
    YOLO --> LANE[LaneInfoExtractor]
    LANE --> PATH[PathPlanner<br/>CubicSpline]
    PATH --> MOTION[MotionPlanner<br/>Stanley controller]
    MOTION --> CMD[/topic_control_signal<br/>MotionCommand]
    CMD --> SERIAL[SerialSenderNode]
    ARM[DriveArmNode<br/>/vehicle/armed] --> SERIAL
    SERIAL --> ARDUINO[Arduino Mega 2560 추정<br/>driving_user_pins.ino]
    ARDUINO --> STEER[조향 DC motor]
    ARDUINO --> DRIVE[좌/우 구동 motor]
    POT[Steering potentiometer A2] --> ARDUINO
```

`TrackControllerNode`는 내부에서 `YoloDetector → LaneInfoExtractor → PathPlanner → MotionPlanner`를 직접 호출한다. 생성된 `MotionCommand`는 `/topic_control_signal`로 발행되며, `SerialSenderNode`가 calibration-ready와 arm 상태를 모두 확인한 후 Arduino 명령 문자열로 변환한다.

`track_drive.launch.py`는 LiDAR node도 시작하지만, 현재 `TrackControllerNode`는 LiDAR topic을 subscribe하지 않는다. 따라서 트랙 주행 판단에는 LiDAR가 실제로 연결돼 있지 않다.

## 수동 주행/데이터 수집 경로

```mermaid
graph TD
    KEY[키보드 W/A/S/D/X/Space] --> MANUAL[ManualDriveCaptureNode]
    CAMERA[/camera/front/image_raw] --> MANUAL
    MANUAL --> DATA[이미지 + 명령 데이터셋]
    MANUAL --> CMD[/topic_control_signal]
    CMD --> SERIAL[SerialSenderNode]
    ARM[arm topic] --> SERIAL
    SERIAL --> FW[driving_user_pins.ino]
```

`ManualDriveCaptureNode`는 키 입력으로 속도와 조향을 조절하고, 카메라 프레임과 현재 명령을 함께 저장한다. `dry_run`을 사용하면 serial/motor 없이 데이터 경로를 점검할 수 있다.

## mission 경로

```mermaid
graph TD
    CAMERA[CameraPublisherNode] --> YOLO[YOLO detection]
    YOLO --> TL[Traffic-light processing]
    YOLO --> LP[Lane processing]
    LIDAR[LidarPublisherNode] --> LPROC[Lidar processor]
    LPROC --> OBS[Obstacle detector]
    TL --> MM[Mission manager]
    LP --> MM
    OBS --> MM
    MM --> PP[Path planner]
    PP --> MP[Motion planner]
    MP --> CMD[/topic_control_signal]
    CMD --> GATE[SerialSenderNode + arm gate]
    GATE --> ARDUINO[Arduino firmware]
```

이 구조는 launch와 topic 설계상 존재하지만, 현재는 일부 perception/decision library import가 실패하므로 실제 실행 가능한 완성 경로로 볼 수 없다.

## 주요 topic 및 service

| Topic/service | 생산자 → 소비자 | 의미 |
|---|---|---|
| `/camera/front/image_raw` | front camera → track/manual/mission | 전방 영상 |
| `/camera/aux/image_raw` | aux camera → 필요 node | 보조 영상 |
| `/lidar_raw` | lidar publisher → lidar processor | 원본 LaserScan |
| `/lidar_processed` | lidar processor → obstacle detector | 전처리 scan |
| `/lidar_obstacle_info` | obstacle detector → mission manager | 장애물 정보 |
| `/detections` | YOLO node → 후속 perception | 객체 검출 결과 |
| `/yolov8_traffic_light_info` | traffic-light node → mission | 신호등 판단 |
| `/mission_target_lane` | mission manager → lane/path logic | 목표 차선 |
| `/yolov8_lane_info` | lane node → path/mission | 차선 정보 |
| `/path_planning_result` | path planner → motion planner | 경로 결과 |
| `/mission_force_stop` | mission manager → motion planner | 강제 정지 |
| `/mission_speed_limit` | mission manager → motion planner | 속도 제한 |
| `/mission_state` | mission manager → 모니터링 | mission 상태 |
| `/topic_control_signal` | manual/track/mission → serial bridge | 최종 차량 명령 |
| `/vehicle/armed` | arm UI → serial bridge | 출력 허용 요청 |
| `/vehicle/calibration_ready` | serial bridge → arm UI/monitor | calibration 준비 상태 |
| `/vehicle/calibration_status` | serial bridge → monitor | calibration 상태 문자열 |
| `~/set_enabled` | TrackControllerNode, SetBool | 트랙 제어 활성화/비활성화 |
| YOLO SetBool service | YOLO node | 검출 활성화/비활성화 |

ROS action은 발견되지 않았다.

## PC↔Arduino serial protocol

`SerialSenderNode`와 `driving_user_pins.ino` 사이의 주요 protocol은 다음과 같다.

| 명령 | 역할 |
|---|---|
| `?` | firmware의 현재 steering config 조회 |
| `C` | calibration 측정 시작 |
| `Kleft,center,right` | 측정한 calibration 적용 |
| `X` | 즉시 정지 및 출력 비활성화 |
| `s{steer}l{left}r{right}\n` | 정상 조향/좌·우 모터 명령 |

정상 차량 명령은 `ready && armed`일 때만 Arduino로 전달된다. serial node 기본값인 `auto_calibrate=false`에서는 firmware가 유효한 config를 반환하면 READY 상태로 진입한다.

# 6. 전체 데이터 흐름

## 트랙 자율주행

```text
전방 카메라
→ CameraPublisherNode
→ YOLO 차선/표식 검출
→ LaneInfoExtractor
→ CubicSpline 경로 생성
→ Stanley 조향 계산
→ MotionCommand
→ calibration/arm safety gate
→ serial protocol
→ Arduino
→ potentiometer feedback를 이용한 조향 motor 제어 + 좌/우 구동 motor PWM
```

## mission 주행

```text
카메라 + LiDAR
→ 객체/신호등/차선/장애물 인식
→ MissionManager 판단
→ PathPlanner
→ MotionPlanner
→ MotionCommand
→ safety gate
→ Arduino
→ actuator
```

현재 mission 흐름은 Python library import 오류 때문에 중간에서 시작되지 못할 가능성이 확정적으로 높다.

## 최종 명령 생성 위치

- 트랙: `skku_track_drive_pkg`의 `MotionPlanner`가 steering/speed를 계산하고 `TrackControllerNode`가 `MotionCommand`를 발행한다.
- 수동: `ManualDriveCaptureNode`가 키 입력을 직접 `MotionCommand`로 변환한다.
- mission: `decision_making_pkg`의 motion planner가 최종 제어 신호를 생성하도록 설계돼 있다.
- actuator 적용: `vehicle_io_pkg/SerialSenderNode`가 gate를 거쳐 문자열을 보내고 `driving_user_pins.ino`가 실제 PWM/direction pin을 제어한다.

# 7. 실행 방법

> 아래 명령은 repository에 있는 launch 및 문서를 기준으로 정리했다. 조사 중 실제 차량 구동 launch는 실행하지 않았다.

## 7.1 기준 workspace 빌드

```bash
source /opt/ros/humble/setup.bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
colcon build --symlink-install
source install/setup.bash
```

현재 source 변경 시각이 기존 `build/install/log`보다 새롭기 때문에, 기존 산출물은 현재 working tree를 검증하지 않는다. 위 빌드는 새 산출물을 기록하므로 이번 읽기 전용 조사에서는 수행하지 않았다.

## 7.2 dependency 및 권한

설치 스크립트는 다음과 같이 존재하지만 이번 조사에서는 실행하지 않았다.

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
./tools/install_runtime.sh
```

serial 장치 사용자는 일반적으로 `dialout` group이 필요하다.

```bash
sudo usermod -aG dialout "$USER"
```

적용하려면 로그아웃/로그인이 필요하다. 현재 사용자는 조사 시점에 `dialout` group에 속하지 않았다.

장치 확인/alias 도구:

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
./tools/list_devices.sh
./tools/create_serial_alias.sh
```

두 번째 명령은 시스템 장치 설정을 변경할 수 있으므로 스크립트 내용을 재확인한 뒤 사용해야 한다.

## 7.3 motor 없이 bench capture

```bash
source /opt/ros/humble/setup.bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
source install/setup.bash
ros2 launch vehicle_bringup_pkg bench_capture.launch.py \
  camera_device:=/dev/video0 \
  output_dir:=/tmp/autodrive_capture
```

이 launch는 serial bridge를 시작하지 않는 dry-run/데이터 수집 경로다. 다만 실제 카메라 장치를 연다.

## 7.4 센서 확인

```bash
ros2 launch vehicle_bringup_pkg sensor_check.launch.py \
  front_camera_device:=/dev/video0 \
  aux_camera_device:=/dev/video2 \
  lidar_port:=/dev/lidar
```

현재 조사 환경에는 `/dev/video*`, `/dev/arduino`, `/dev/lidar`, `/dev/ttyACM*`, `/dev/ttyUSB*`가 발견되지 않았다. 이 상태에서는 정상 실행되지 않는다.

## 7.5 수동 주행/데이터 수집

```bash
ros2 launch vehicle_bringup_pkg manual_capture.launch.py \
  camera_device:=/dev/video0 \
  arduino_port:=/dev/arduino \
  output_dir:=/path/to/dataset \
  auto_calibrate:=false
```

- W/S: 속도 증가/감소
- A/D: 조향
- X 또는 Space: disarm/정지
- C: 즉시 capture
- R: auto capture toggle
- Q: 종료

실차에서 사용하기 전에 바퀴를 지면에서 띄우고 모터 방향, watchdog, steering limit을 검증해야 한다.

## 7.6 트랙 자율주행

```bash
ros2 launch vehicle_bringup_pkg track_drive.launch.py \
  camera_device:=/dev/video0 \
  arduino_port:=/dev/arduino \
  device:=cpu \
  speed:=80 \
  auto_calibrate:=false
```

launch에서 track controller는 활성 상태로 시작하지만 serial bridge는 READY 이후에도 W 키 arm이 있어야 실제 명령을 전달한다. 현재 GPU/CUDA를 사용할 수 없는 환경이므로 `device:=cpu`가 필요하다.

## 7.7 mission launch

설계상 실행 명령은 다음 형태다.

```bash
ros2 launch launch_pkg mission_launch.py
```

그러나 현재 `camera_perception_pkg`, `lidar_perception_pkg`, `decision_making_pkg`의 library import가 실패하므로 실행 전에 해당 문제를 먼저 해결해야 한다. 현재 상태에서는 실행 가능 명령으로 간주하면 안 된다.

## 7.8 Arduino firmware

기준 firmware:

```text
Autonomous_Capstone-integrated/src/control/driving_user_pins/driving_user_pins.ino
```

조향 limit 측정용 firmware:

```text
Autonomous_Capstone-integrated/src/control/steering_limit_calibration/steering_limit_calibration.ino
```

문서 및 pin 구성상 Arduino Mega 2560을 의도한 것으로 보이지만, 실제 장착 보드 모델은 현물 확인이 필요하다. repository에는 확정된 `arduino-cli` upload 명령이 없고 현재 시스템에도 `arduino-cli`가 설치돼 있지 않아, 정확한 CLI 업로드 명령은 확인 필요다.

`steering_limit_calibration.ino`의 `c` 명령과 serial bridge의 `auto_calibrate:=true`는 조향 모터를 물리적으로 움직일 수 있다. 안전 확보 없이 실행하면 안 된다.

## 7.9 과거 `h-mobility-class` workspace

과거 경로는 일반적으로 다음 형태로 빌드/실행하도록 구성돼 있다.

```bash
source /opt/ros/humble/setup.bash
cd /home/autolab/autodrive_ws/h-mobility-class
colcon build --symlink-install
source install/setup.bash
ros2 launch launch_pkg main_launch.py
```

그러나 이 launch는 구형 serial node/firmware 조합, 7초 후 자동 serial 시작, 안전 arm gate 부재, watchdog 부재 등의 위험이 있다. 인수 후 기준 실행 경로로 사용하지 않는 것이 안전하다.

# 8. 현재 구현된 기능

## 정상 구현으로 확인되는 기능

- ROS 2 custom message 16종 정의
- V4L2 카메라 입력 및 ROS Image 발행
- RPLidar 입력 및 LaserScan/TF 발행
- 키보드 수동 조향/가감속 및 데이터셋 저장
- 트랙 YOLO 검출, 차선 정보 추출, spline path planning, Stanley steering 계산
- `MotionCommand` 기반 PC→Arduino serial bridge
- calibration query/apply protocol
- READY + arm의 이중 출력 gate
- Arduino 명령 parser의 길이/형식 검증
- 유효 명령 기준 500 ms watchdog
- 조향 potentiometer feedback 기반 좌/우 방향 및 PWM 제어
- 좌/우 drive motor PWM 제어
- host-side Python 안전 테스트 일부
- track controller 활성화용 SetBool service

## 실제로 동작할 것으로 보이는 범위

- 하드웨어와 의존성이 갖춰지면 `bench_capture`는 카메라/데이터 수집까지 동작할 가능성이 높다.
- manual 경로는 camera + serial + Arduino가 연결되고 사용자가 W로 arm하면 수동 주행과 데이터 저장까지 설계돼 있다.
- track 경로는 bundled model을 CPU로 로드해 영상 기반 주행 명령을 만들 수 있다. 단, lane-loss 안전성과 실제 calibration 값 문제를 먼저 확인해야 한다.
- mission 경로는 현재 import 오류로 정상 시작이 어렵다.
- 현재 조사 machine에서는 카메라·LiDAR·Arduino 장치가 없으므로 hardware 포함 동작은 불가능하다.

# 9. 현재 미완성 / 문제 가능성이 있는 부분

## Critical

### 1. 두 workspace가 동시에 존재해 기준 코드가 불명확함

로컬 문서는 `Autonomous_Capstone-integrated`를 기준으로 명시하지만 현재 작업 디렉터리는 `h-mobility-class`다. 팀 합의 없이 어느 쪽을 수정해야 하는지 판단하면 변경이 분산될 수 있다.

### 2. 기준 workspace의 핵심 변경이 commit되지 않음

`vehicle.yaml`, `configuration.py`, 안전 문서와 테스트 등 필요한 파일 일부가 untracked다. tracked 파일도 다수 수정된 상태다. 다른 clone에서는 동일한 실행 구성을 재현할 수 없다.

### 3. steering calibration 값이 서로 불일치함

- 현재 `driving_user_pins.ino`: `LEFT=440`, `CENTER=355`, `RIGHT=270`
- `vehicle.yaml` 및 일부 문서/테스트: `LEFT=600`, `CENTER=522`, `RIGHT=445`

이 값 차이는 조향 방향, 중심, limit 계산과 안전성에 직접 영향을 준다. 실제 차량에서 ADC를 재측정하고 단일 값으로 통일해야 한다.

### 4. mission package import가 실제로 실패함

다음 `lib/__init__.py`들은 인접 `.py`를 import하지 않고 존재하지 않는 절대 `.pyc` 경로를 `marshal/exec`로 읽으려 한다.

- `camera_perception_pkg/lib/__init__.py`
- `lidar_perception_pkg/lib/__init__.py`
- `decision_making_pkg/lib/__init__.py`

계산되는 잘못된 경로 예:

```text
/home/autolab/autodrive_ws/src/build/build/lib/*.cpython-310.pyc
```

안전한 import 검사에서 lane extractor, traffic-light detector, lidar processor, obstacle detector, decision motion planner가 `FileNotFoundError`로 실패했다. 또한 decision motion planner는 `decision_making_func_lib`를 import하지만 source 파일명은 `decision_making_function_lib.py`여서 별도 이름 불일치도 존재한다.

### 5. 구형 workspace는 실차 실행 위험이 있음

`h-mobility-class`의 기존 main launch/serial/Arduino 조합은 arm gate와 command watchdog가 부족하다. 조사 목적으로도 실차에서 실행하지 않는 편이 안전하다.

## Important

### 1. 트랙에서 차선을 잃어도 직진 속도가 남을 수 있음

영상 callback은 계속 들어오지만 path detection이 없을 때 `MotionPlanner`가 조향 0과 기본 속도를 반환할 수 있다. 즉 차선 인식 실패가 즉시 정지로 연결되지 않는다.

### 2. 트랙 launch의 LiDAR가 제어에 연결되지 않음

LiDAR node는 시작되지만 `TrackControllerNode`가 해당 topic을 subscribe하지 않는다. 장애물 회피/정지 안전 계층이 없는 영상 단독 주행이다.

### 3. mission의 기본 GPU 설정이 현재 환경과 맞지 않음

mission YOLO 기본값은 `cuda:0`이지만 현재 `torch.cuda.is_available()`은 false였고 `nvidia-smi`도 driver와 통신하지 못했다.

### 4. model 경로와 provenance가 불명확함

track package의 `models/best.pt`와 repository의 다른 `best.pt`는 크기와 hash가 다르다. mission은 상대 경로 `best.pt`를 기본 사용하므로 실행 위치에 따라 다른 파일을 찾거나 실패할 수 있다.

### 5. dependency version 충돌 경고

현재 환경은 NumPy 1.26.4, SciPy 1.8.0이며 SciPy가 요구하는 NumPy 범위와 맞지 않는다는 warning이 발생했다. track import는 됐지만 수치 처리 runtime 안정성을 보장하지 않는다.

### 6. command publisher 간 arbitration 부재

manual/track/mission이 같은 `/topic_control_signal`을 발행할 수 있다. 둘 이상 동시에 실행하면 마지막으로 도착한 명령이 적용될 가능성이 있다. source별 mux/ownership이 필요하다.

### 7. arm은 요청 상태이지 실제 actuator 상태 feedback이 아님

`/vehicle/armed`는 UI 요청이고, Arduino 출력이 실제로 활성화됐는지까지 폐루프 확인하지 않는다.

### 8. manifest만으로 runtime dependency가 완전히 표현되지 않음

YOLO, OpenCV, serial, scientific Python 등의 실제 import와 `package.xml` 선언이 충분히 일치하는지 재점검이 필요하다.

## Later / 정리 대상

- 중복 firmware 및 중복 node
- backup/prototype 파일
- 여러 `.pt` model의 용도 불명확
- tracked IDE metadata(`.vs` 등)
- root의 대형 ZIP 파일
- TODO/FIXME 및 placeholder package metadata
- 주석 처리된 legacy 코드와 문서상 이미 폐기된 launch
- `motor_test/motor_test.ino`의 UNO/Nano 가정과 기준 Mega pin 구성의 충돌

# 10. 최근 변경사항

## 마지막 commit `6ffd1e9`

확인된 범위에서 마지막 commit은 과거 `h-mobility-class` workspace의 vehicle config 및 steering motor control을 손봤다. 카메라 perception library loading, planners, launch 설정, serial sender, `driving.ino`가 함께 변경됐다.

## 현재 working tree

현재 미commit 변경은 통합 workspace에 집중돼 있다.

- `.gitignore`와 통합 README 보강
- `driving.ino`, `driving_user_pins.ino`, calibration sketch 수정
- mission launch에 새 serial safety gate 연결
- manual node 변경
- `vehicle_bringup_pkg` launch/package/setup 변경
- `vehicle_io_pkg` serial sender 변경
- 안전/운용 문서 다수 추가
- `vehicle.yaml`과 Python config loader 추가
- firmware test/tool 추가
- 별도 `motor_test` sketch 추가

즉 최근 팀원 작업은 단순 기능 추가라기보다, 차량 출력 경로를 `vehicle_io_pkg + driving_user_pins.ino`로 통합하고 calibration/arm/watchdog를 강화하는 방향으로 보인다. 이는 파일 내용에 근거한 해석이며, 팀원의 실제 의도와 완료 여부는 직접 확인해야 한다.

# 11. 중요한 파라미터

## Arduino 기준 firmware

파일: `Autonomous_Capstone-integrated/src/control/driving_user_pins/driving_user_pins.ino`

| 항목 | 값 | 사용 위치/역할 |
|---|---:|---|
| steering direction/PWM pins | D3 / D2 | 조향 모터 방향/PWM |
| right motor direction/PWM | D4 / D5 | 우측 구동 모터 |
| left motor direction/PWM | D7 / D6 | 좌측 구동 모터 |
| steering potentiometer | A2 | 조향 위치 feedback |
| default left ADC | 440 | 조향 좌 limit |
| default center ADC | 355 | 조향 중심 |
| default right ADC | 270 | 조향 우 limit |
| maximum steering command | 7 | ROS/serial 조향 범위 제한 |
| steering PWM | 128 | 평상시 조향 motor 출력 |
| calibration PWM | 90 | calibration 이동 출력 |
| drive motor range | -255..255 | 좌/우 속도 PWM |
| command watchdog | 500 ms | 유효 명령 미수신 시 정지 |
| control period | 30 ms | 조향/구동 갱신 주기 |
| status period | 500 ms | 상태 출력 주기 |
| calibration sample period | 30 ms | ADC sample 주기 |
| potentiometer change threshold | 1 | 정지/변화 판정 임계값 |
| endpoint still time | 350 ms | limit 도달 판정 |
| minimum movement time | 300 ms | 너무 이른 endpoint 판정 방지 |
| stable time | 450 ms | 안정화 판정 |
| center timeout | 5000 ms | 중심 이동 timeout |
| side timeout | 6500 ms | 좌/우 limit 탐색 timeout |
| center deadband | 3 ADC | 중심 정지 허용 오차 |
| minimum calibration span | 80 ADC | calibration 유효성 검사 |
| serial baudrate | 115200 | PC↔Arduino 통신 |

`X`는 PWM을 끄며 자동으로 center로 복귀시키지 않는다. watchdog timestamp는 parser를 통과한 유효 명령만 갱신한다.

## 공유 vehicle YAML

파일: `Autonomous_Capstone-integrated/src/vehicle_bringup_pkg/config/vehicle.yaml`

| 항목 | 값 | 역할 |
|---|---|---|
| Arduino port | `/dev/arduino` | serial alias |
| baudrate | 115200 | serial 속도 |
| front camera | `/dev/video0` | 전방 카메라 |
| front topic | `/camera/front/image_raw` | 전방 영상 topic |
| aux camera | `/dev/video2` | 보조 카메라 |
| aux topic | `/camera/aux/image_raw` | 보조 영상 topic |
| LiDAR port | `/dev/lidar` | LiDAR serial alias |
| LiDAR topic | `/lidar_raw` | 원본 scan topic |
| LiDAR rotation | 180° | scan 방향 보정 |
| steering left/center/right | 600 / 522 / 445 | 문서화된 calibration 값, firmware와 불일치 |
| max steering | 7 | 제어 명령 범위 |
| auto calibration | false | startup 자동 물리 이동 방지 |
| calibration tolerance | 35 | config 비교 허용 오차 |

## serial bridge

파일: `vehicle_io_pkg/vehicle_io_pkg/serial_sender_node.py`

| 항목 | 값 | 역할 |
|---|---:|---|
| serial startup wait | 1.2 s | Arduino reset 대기 |
| calibration tolerance | 35 | 기대값과 firmware config 비교 |
| minimum span | 80 | invalid calibration 거부 |
| calibration timeout | 22 s | calibration 전체 timeout |
| config retry | 1 s | `?` 재전송 주기 |
| serial write timeout | 0.05 s | write block 제한 |

## 수동 주행

파일: `manual_drive_pkg/manual_drive_pkg/manual_drive_capture.py`

| 항목 | 기본값 | 역할 |
|---|---:|---|
| speed step | 20 | W/S 증감 폭 |
| steering step | 1 | A/D 증감 폭 |
| max speed | 255 | 모터 명령 제한 |
| max steering | 7 | 조향 제한 |
| left speed sign | -1 | 좌측 모터 장착 방향 보정 |
| auto capture interval | 0.20 s | 자동 데이터 저장 주기 |
| JPEG quality | 95 | 이미지 저장 품질 |
| command republish | 20 Hz | 명령 유지 주기 |

## 트랙 제어

| 항목 | 값 | 역할 |
|---|---:|---|
| node default speed | 100 | 코드 기본 속도 |
| launch default speed | 80 | 실제 launch 기본값 |
| inference device | CPU 권장 | 현재 CUDA 불가 |
| confidence | 0.5 | YOLO threshold |
| max steering | 7 | 최종 조향 제한 |
| max steering angle | 50° | Stanley 계산의 물리각 스케일 |
| Stanley gain | 0.02 | 횡오차 gain |
| softening | 0.001 | 저속 분모 안정화 |
| lookahead index | 10 | path 목표점 |
| heading step | 3 | heading 계산 간격 |
| heading gain | 0.6 | heading 오차 반영 |
| vehicle center | `(320, 179)` | 영상 좌표상 차량 기준점 |
| BEV top shift | -8 | 원근 변환 보정 |
| ROI | 300 | 차선 관심 영역 관련 값 |
| look shift | 50 | 탐색 위치 이동 |
| EMA alpha | 0.3 | 차선 smoothing |
| virtual lane width | 300 | 한쪽 차선 손실 시 가상 차선 |
| padding | 250 | path/차선 보조 범위 |

## mission 관련 주요 값

| 항목 | 값 | 역할 |
|---|---:|---|
| normal speed | 150 | 일반 주행 |
| avoidance speed | 90 | 회피 구간 |
| stop speed | 0 | 정지 |
| motion planner max/min speed | 250 / 250 | 현재 동일해 곡률 기반 감속이 사실상 무효일 가능성 |
| steering bias | -2 | 조향 offset |
| max steering step | 7 | 조향 제한 |
| theta | 75 | 제어 계산 parameter |
| alpha | 0.3 | smoothing |
| steering slew | 1 | cycle별 조향 변화 제한 |
| obstacle angular sector | 0–30° | 전방 장애물 검색 범위 |
| obstacle distance | 0.5–2.0 m | 장애물 유효 거리 |
| consecutive detection | 5 | 장애물 확정 frame 수 |
| parking speed | 90 | parking launch |
| parking exit speed | 100 | parking 탈출 |
| parking steering | 6 | parking 조향 |

## 과거 workspace 주요 값

| 항목 | 값 |
|---|---:|
| PID Kp/Ki/Kd | 0.05 / 0 / 0.005 |
| default speed | 255 |
| max steering | 7 |
| steering offset | 10 |
| stop y threshold | 160 |
| old firmware endpoints | 425 / 278 |
| old steering PWM | 50 |

# 12. 내가 다음으로 확인해야 할 것

## Critical

1. 팀원에게 공식 기준 workspace가 `Autonomous_Capstone-integrated`가 맞는지 확인한다.
2. 현재 unstaged/untracked 파일이 의도된 인수인계 결과인지, 누락된 commit인지 확인한다.
3. 실제 차량에서 조향 potentiometer의 left/center/right ADC를 안전하게 재측정한다.
4. firmware, `vehicle.yaml`, 문서, host test의 calibration 값을 하나로 통일한다.
5. mission package의 비정상 `.pyc` loader와 module naming 문제를 해결하기 전 mission을 실행하지 않는다.
6. 바퀴를 띄운 bench 상태에서 `X`, 500 ms watchdog, motor direction, steering limit을 검증한다.

## Important

1. 깨끗한 clone 또는 별도 검증 환경에서 기준 workspace 전체를 다시 build/test한다.
2. `/dev/arduino`, `/dev/lidar`, camera 장치 번호와 udev alias를 실제 하드웨어에서 확인한다.
3. 사용자를 `dialout` group에 추가하고 serial 권한을 확인한다.
4. Python/ROS 의존성 버전을 고정하고 NumPy/SciPy 조합을 정리한다.
5. lane-loss 시 즉시 감속/정지 정책을 추가하고 실차 전에 시험한다.
6. track 주행에 LiDAR 안전 정지 계층을 연결할지 결정한다.
7. 각 `.pt` model의 학습 대상, class map, 정확한 사용 package를 기록한다.
8. `/topic_control_signal`에 source arbitration 또는 mux를 도입한다.
9. `package.xml`과 실제 runtime import 의존성을 일치시킨다.

## Later

1. 두 workspace를 하나의 공식 구조로 정리하고 legacy를 archive한다.
2. 중복 Arduino sketch, prototype, backup, IDE metadata를 정리한다.
3. 대형 model/ZIP 파일의 Git LFS 또는 artifact 관리 정책을 정한다.
4. serial bridge의 arm 요청뿐 아니라 firmware의 실제 출력 상태 feedback도 설계한다.
5. 안전 테스트, launch test, hardware-in-the-loop 절차를 문서화한다.

# 이 프로젝트를 처음 보는 사람이 알아야 하는 핵심 10가지

1. 실제 Git root는 현재 디렉터리가 아니라 `/home/autolab/autodrive_ws`다.
2. repository 안에 ROS 2 workspace가 두 개 있으며, 문서상 현재 기준은 `Autonomous_Capstone-integrated`다.
3. 현재 branch/HEAD는 `main`/`6ffd1e9`이고 staged 파일은 없지만 통합 workspace에는 많은 미commit 변경이 있다.
4. 로컬 `origin/main`과 HEAD는 같지만 `fetch`하지 않았으므로 원격 최신 여부는 확인되지 않았다.
5. 기준 차량 출력 경로는 `vehicle_bringup_pkg → vehicle_io_pkg/SerialSenderNode → driving_user_pins.ino`다.
6. serial 출력은 calibration READY와 사용자의 arm이 모두 만족돼야 전달된다.
7. firmware에는 500 ms watchdog가 있지만 legacy firmware/launch에는 같은 안전성이 없다.
8. steering calibration 값이 firmware와 YAML/문서/테스트에서 크게 다르므로 실차 실행 전 반드시 실제 ADC를 확인해야 한다.
9. 트랙 주행은 LiDAR를 사용하지 않으며 차선 손실 시 직진 명령이 남을 가능성이 있다.
10. mission pipeline은 현재 Python library import 오류 때문에 실행 불가능한 상태로 판단된다.

---

## 조사 중 수행한 비파괴 검사

- Git status/log/diff/branch/remote 조회
- Python source 90개 AST parse: syntax error 0개
- Python offline safety test: 5개 통과
- 일부 package import 검사
- host C++ firmware safety test compile

host C++ test는 compile에는 성공했지만 `runtime_center == 532` assertion에서 실패했다. 이는 테스트가 과거 calibration 값 `600/522/445`를 기대하는 반면 현재 firmware 기준값이 `440/355/270`으로 바뀌었기 때문이다.

조사 중 repository 파일은 수정하지 않았고, test 임시 산출물과 ROS log는 `/tmp`에만 생성했다. 단, 사용자의 후속 요청에 따라 이 보고서 파일 자체는 새로 추가했다.
