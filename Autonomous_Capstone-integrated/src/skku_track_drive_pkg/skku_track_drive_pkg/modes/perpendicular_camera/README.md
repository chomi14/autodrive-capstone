# 카메라 기반 수직주차

전방 주차 구역 추종과 후방 중심선·종료선 기반 후진 주차를 제공한다.
기존 `perpendicular_parking.launch.py`의 시간 기반 주차와 독립적인 실행 모드다.
주차 전용 학습 가중치를 기본 경로로 사용한다. BEV·조향·정지 위치 설정은
**예시이며 실측값이 아니다**. 실차 카메라 보정이 필요하다.

## 파일과 실행 연결

| 위치 | 역할 |
| --- | --- |
| 이 폴더의 `observations.py` | 한 원본 프레임에서 나온 관측 및 좌표 규약 |
| `perception.py` | BEV, 전방 경계와 평행주차선 후보, 후방 중심선, 종료·출차선 계산 |
| `model.py` | Ultralytics segment 모델과 실제 class 이름 대응 |
| `perception_node.py` | 카메라 하나당 인식 프로세스 하나 |
| `core.py` | ROS가 없는 상태 전이와 조향 계산 |
| `controller_node.py` | 20 Hz 명령 발행, 관측 검사, W/S 상태, reset/save |
| `viewer_node.py`, `tuning_panel.py` | 전·후방 6개 영상, 상태·BEV 기준표시, 설정 편집과 저장 |
| `configuration.py` | 이 모드 전용 설정 검증·원자적 저장 |
| `src/vehicle_bringup_pkg/config/perpendicular_camera.yaml` | 제어·카메라·모델·BEV 전체 설정 |
| `src/vehicle_bringup_pkg/vehicle_bringup_pkg/modes/perpendicular_camera/launch.py` | 이 모드 전용 실행 조립 |
| `tools/tests/perpendicular_camera/` | 장비 없는 상태 전이·좌표·어댑터 테스트 |

입력은 기존 `sensor_bringup_pkg/camera_publisher_node` 두 개 또는 외부 Image
토픽이다. 전·후방 인식 결과는 `/parking_camera/{front,rear}/observation`의
JSON으로 전송하고, **제어기 하나만** 기존 `interfaces_pkg/MotionCommand`를 발행한다.
관측의 `stamp_ns`, `frame_id`, valid 및 모든 geometry는 같은 이미지에서 나온다.
기존 메시지 패키지를 수정하거나 원본 패키지를 중복 복사하지 않는다.

실차 출력 연결은 기존 `topic_control_signal` →
`vehicle_io_pkg/serial_sender_node_v2` → `s<steering>l<left>r<right>\n` → Arduino다.
`src/control/driving/driving.ino`와 `driving_user_pins/driving_user_pins.ino`의
명령 파서는 조향 ±7, PWM ±255를 사용한다. **차량에 실제 업로드된 펌웨어와
배선은 여기서 확인할 수 없으며, 이 작업은 펌웨어를 변경하지 않는다.**
기존 arm gate와 controller/tuner heartbeat 경로를 그대로 이용한다.

## 참조와 이용 조건

- 대상 기준: `chomi14/autodrive-capstone`의 `1030`,
  `56395e0c9930bf88bf616c061b86faf0ebff1d65`.
- 알고리즘 참고: [ksk192830/ROS2_AutonomousVehicle](https://github.com/ksk192830/ROS2_AutonomousVehicle)
  `9c657d6007354e9bad319eb2f55f44d75a1dc43c`, `projects/4th_skku_contest`.
- 확인 파일: `parking_front_detect.py`, `parking_rear_detect.py`,
  `motion_parking.py`, `launch/parking.py`, `yolo_seg.py`, 주차 메시지 정의.
- 원본 플랫폼 출처: 성균관대학교 자동화연구실의
  [ros2_autonomous_vehicle_book](https://github.com/SKKUAutoLab/ros2_autonomous_vehicle_book).

원본 FSM과 영상 처리 아이디어를 대상 인터페이스에 맞춰 재구현했다.
원본 800줄 인식 노드나 전역 패키지를 통째로 복사한 것이 아니며,
기존 결과와의 수치적 동등성·동일 주차 성능을 주장하지 않는다.
원본 `docs/license-and-credits.md`는 프로젝트 전체에 단일 MIT를 적용하지
않으며 교재 코드 GPL-3.0 등 파일별 조건 검토를 요구한다.
참고한 `camera_pkg/package.xml`의 라이선스는 `TODO: License declaration`이다.
따라서 이식 부분 전체의 이용·재배포 조건을 원본에서 확정했다고 볼 수 없다.
출처를 보존했고 원본 가중치·논문·사진·시뮬레이션 에셋은 포함하지 않았다.

## 필요한 모델과 클래스

원본은 전방 `parking_front.engine`, 후방 `parking_rear.pt`를 참조하나
해당 주차 가중치를 공개하지 않는다. 대상의 `best_v2.pt`, `best_v3.pt`는
주행 클래스, `mission/best_m.pt`는 lane1/lane2/obstacle/traffic_light 메타데이터를
갖는다. 이 모델들을 주차 모델로 자동 대체하지 않는다.

대신 제공된 `parking_front.v2i.yolov8.zip`, `parking_rear.v5i.yolov8.zip`으로
각각 학습한 YOLO11n segmentation 모델을 사용한다. YOLOv8은 여기서 데이터셋
라벨 형식이며, YOLO11도 같은 polygon 형식을 사용한다.

| 역할 | 포함 가중치 (`skku_track_drive_pkg/models/parking/`) | 실제 ID → 클래스 |
| --- | --- | --- |
| 전방 | `parking_front.pt` | 0 → out_line, 1 → parking_lot, 2 → parking_space |
| 후방 | `parking_rear.pt` | 0 → end_line, 1 → parking_lot, 2 → parking_space |

후방의 `parking_lot`도 삭제·재번호 부여 없이 학습한다. 후방 제어에는
`parking_space`, `end_line`만 사용하고 `parking_lot` 결과는 제외한다.
YAML의 `package://skku_track_drive_pkg/models/parking/...` 경로는 설치된 ROS
패키지 share에서 해석하므로 실행 디렉터리나 사용자 홈 이름에 의존하지 않는다.
CLI의 `front_model`, `rear_model`로 별도 파일을 지정할 수도 있다.
학습 설정·분할 중복 제거·평가 수치는 저장소의
`reports/parking_training_2026-11-13/`와 `models/parking/README.md`를 확인한다.

| 카메라 | 이 모드의 의미 키 | 원본 class_name | 원본 출력 | 이 구현 |
| --- | --- | --- | --- | --- |
| 전방 | parking_lot | parking_lot | ParkingLot: found, x 정규화, yaw rad | 합친 ROI의 왼쪽 경계 |
| 전방 | parking_space | parking_space | ParkingSpace: 탐색에서 found 사용 | 연속 미검출 횟수 계산 |
| 전방 | out_line | out_line | OutLine: x/y 정규화, yaw rad | 출차선 x/y 정규화, yaw rad |
| 후방 | parking_space | parking_space | ParkingSpace: 하단 x 정규화, yaw rad | ROI 안 영상 선으로 중심선 계산 |
| 후방 | end_line | end_line | EndLine: 원본 x/y는 pixel, yaw rad | x/y 정규화, yaw rad |

고정 class ID는 가정하지 않는다. `classes`의 값은 **확보한 모델의 실제 클래스 이름**이다.
시작 시 `model.names`로 필수 클래스 존재를 검사하고 ID를 연결한다.
Ultralytics 호환 segment `.pt`, `.onnx`, `.engine`을 지원하는 어댑터이며,
각 backend의 설치 의존성과 실제 추론은 가중치를 확보한 환경에서 확인해야 한다.
engine은 해당 GPU·TensorRT 환경에 맞게 준비한다. 박스 전용 모델은 사용할 수 없다.
추론 polygon은 영상 회전/반전 후 이미지의 pixel 좌표로 처리한다.

전방 가중치와 데이터셋에는 평행주차선을 가리키는 별도 클래스가 없다.
실제 테스트 영상에서 `out_line`은 바닥의 OUT 글자 앞 출차 표식에 붙으며
평행주차 구획선은 `out_line` 없이도 보인다. 제한선 후보는 전방 BEV의
지정 범위 안에서 흰색 도색의 가로 성분을 찾고, 검출된 출차선 마스크 영역을
제외해 계산한다. 가장 가까운 후보의 y를 정지 기준과 비교한다.
이 검출기는 지정 범위에 들어온 다른 흰색 선도 후보로 고를 수 있다.

## 설치와 모터 없는 확인

ROS 2 Humble/Ubuntu 22.04 작업공간에서 기존 의존성에 더해 `ultralytics`,
NumPy, OpenCV, PyYAML이 필요하다. GUI는 Tk와 Pillow ImageTk를 사용한다.
ROS의 `cv_bridge`와 호환되는 Python·NumPy
환경을 사용한다. 이 변경만을 위해 NumPy/OpenCV 시스템 버전을 임의로 올리지 않는다.

아래 명령은 `Autonomous_Capstone-integrated`에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash
mkdir -p ~/.config/autodrive
cp src/vehicle_bringup_pkg/config/perpendicular_camera.yaml ~/.config/autodrive/perpendicular_camera.yaml
```

복사한 YAML의 기본 모델 경로는 포함 가중치를 가리킨다.
장치, 회전/반전, BEV는 차량에 맞춘다.
기본 입력은 전·후방 모두 1280×720이며 일반 가로 장착 기준으로
전·후방 `rotate_deg: 0`을 사용한다. 영상의 위아래가 정상인 상태로 추론한다.
후방 학습 이미지는 720×1280 세로 시야이므로 일반 가로 장착에서의 인식 성능은
실제 영상으로 확인해야 한다. 가로 영상을 단순히 90도 회전하거나 세로로
늘리는 것으로 학습 당시의 시야를 재현할 수 없다.
물리적으로 회전해 장착한 카메라만 preview에서 확인하여 `rotate_deg`를 조정한다.
이미 방향이 보정된 영상 토픽이나 학습 이미지는 다시 회전하지 않는다.
전·후방 장치는 가능한 `/dev/v4l/by-id/...`로 지정한다.
`model_device: cpu`도 가능하지만 추론 시간이 `observation_timeout_s`를 넘으면 정지한다.
지연 예산은 실제 처리 시간을 측정해 설정한다.

```bash
ros2 launch vehicle_bringup_pkg perpendicular_camera_preview.launch.py \
  config_file:=$HOME/.config/autodrive/perpendicular_camera.yaml
```

preview는 Arduino/arm gate를 시작하지 않으며 최종 명령도
`/sensors_only/topic_control_signal`로 격리한다. W는 **예상 명령 시뮬레이션**을
시작하고 S는 멈춘다. 화면에는 원본 방향 보정 영상·segmentation·BEV가
전방/후방 각 3개씩 나타난다. 장치 단절을 저장 이미지 fallback으로 숨기지 않는다.
preview의 관측·화면·상태 토픽과 reset/save 서비스도 `/parking_camera_preview/`
아래로 분리하여 실차 제어기에 preview 관측이나 reset이 들어가지 않게 했다.
모델이 없으면 명확한 오류로 시작을 차단한다. 모델 없이 가능한 검증은 아래
단위 테스트이며, 영상 인식이 성공한 것으로 대체하지 않는다.

임시 override도 가능하며 P 저장 시 유효 모델 경로와 장치까지 보존한다.

```bash
ros2 launch vehicle_bringup_pkg perpendicular_camera_preview.launch.py \
  config_file:=$HOME/.config/autodrive/perpendicular_camera.yaml \
  front_camera_device:=/dev/video0 rear_camera_device:=/dev/video2 \
  front_model:=/absolute/path/parking_front.pt rear_model:=/absolute/path/parking_rear.pt
```

## 수직주차에서 평행주차까지 이어서 실행

전·후방 11월 25일 데이터로 학습한 공용 평행주차 세그멘테이션 가중치를
`parallel_model:=/absolute/path/parallel_parking.pt`로 지정한다. 또는 같은 YAML의
`parallel.model_path`에 경로를 저장한다. 경로가 비어 있으면 기존 수직주차와
시간제 출차로 동작한다. 두 카메라 노드는 시작할 때 새 모델을 준비하며
출차 첫 직진이 끝나면 이를 사용한다. 분류 ID는 모델의 이름으로 찾아 사용한다.
필수 클래스는 `parallel_roadside_line`, `parallel_divider`, `parallel_row_start`,
`parallel_row_end`, `parallel_far_line`, `parallel_parking_space`, `vehicle`이다.

```bash
ros2 launch vehicle_bringup_pkg perpendicular_camera_preview.launch.py \
  config_file:=$HOME/.config/autodrive/perpendicular_camera.yaml \
  front_camera_device:=/dev/video0 rear_camera_device:=/dev/video4 \
  parallel_model:=/absolute/path/parallel_parking.pt
```

출차 우조향 중 통로 쪽 긴 선이 BEV에서 조향에 쓸 수 있게 잡히면 같은 제어
주기에 선 추종으로 넘어간다. `exit_turn_s`는 최대 우조향 시간이다.
`parallel_target_x` 기본값 0.35는 **전방 BEV 하단에서 본 긴 선의 목표 x**다.
통로선 미검출이면 선을 찾으며 직진하고, 목표 칸과 후진 기준이 없으면 기존
출차 직진 시간(`exit_straight_s`)이 끝나는 때 정지한다.

`parallel_row_start`와 `parallel_row_end`에서 칸 사이의 `parallel_divider`를 세고,
`vehicle` 접점으로 점유를 판단한다. `parallel_parking_space`는 통로선이
정의한 줄 안에서만 목표 칸 후보로 사용한다. 뒤쪽에서 목표 칸의 접근측
경계를 잡고 후방 BEV의 후진 시작 y에 도달하면 전후진 전환 대기를 적용한다.

GUI의 **평행주차** 탭에서 전·후방 BEV, PWM, 통로선 목표 x, 후진 시작 y,
진입 각도, 안쪽 선 추종 기준과 종료 깊이, 조향 게인을 조절한다.
`far_gap`은 후방 차량 기준점에서 `parallel_far_line`까지 남은 거리를
주차 줄 폭으로 나눈 값이다. 1은 통로 쪽 선, 0은 안쪽 선에 해당한다.
좌조향 후진으로 각도를 만들고 필요하면 직진 후진으로 깊이를 늘린다.
`far_gap <= parallel_far_track_gap`(기본 0.80)이면 안쪽 선 각도와 거리로
후진 조향한다. 목표 깊이(기본 0.55)와 정렬 각도를 만족하면 정지한다.
이 값들과 BEV는 preview에서 차량과 주차 줄에 맞춰 보정한다.

외부 영상 토픽을 쓰려면 preview에서 `source:=topics`를 사용한다.
rosbag 재생은 명시적으로 `source:=replay use_sim_time:=true`를 사용한다.
이때 카메라 장치는 열지 않으며, 별도 터미널에서 실제 녹화 bag을
`ros2 bag play /absolute/path/to/bag --clock`로 재생한다.
bag의 Image 토픽 이름은 YAML의 `image_topic`에 맞춰야 한다.
bag 재시작으로 시계가 뒤로 이동하면 S/R로 reset한다.
live launch에서는 replay와 simulated clock을 허용하지 않는다.

## 설정 변경과 저장

미리보기에서 **Stop(S/Esc)** 후 **Front / Rear / Control** 탭을 편집한다.
Front/Rear는 시계 방향 회전, 수평 반전, 모델 신뢰도, BEV 원본·목적 네 점,
마스크와 주차선 검출 설정을 다룬다. 영상과 BEV의 네 점은 같은 번호로 표시한다.
Front 탭의 제한선 검출 범위(x/y), 도색 밝기, 최소 가로 폭을 조절한다.
Front BEV에는 그 범위, 초록색 검출 후보와 `front_limit_stop_y` 기준선이
표시된다. Control 탭에서 접근 정지 y, 제한선 미검출 시 좌회전 전진 시간,
고정 우조향 후진 시간, 탐색 미검출 횟수, 조향 게인·부호, 단계별 PWM,
종료선 y, 출차 우조향 전진·직진 시간과 대기시간을 조절한다.
요약에는 PWM 0을 낸 실제 분기 이유와 최근 상태 전이가 한글로 표시된다.

**Apply**는 비활성 상태에서 검증된 값을 함께 적용하고 FSM을 초기화한다.
회전·반전·BEV·신뢰도와 제어값은 다음 프레임부터 반영된다.
장치 경로·해상도·FPS는 **Save YAML(P)** 후 그 파일로 다시 실행하면 반영된다.
**Reload**는 편집 중인 값을 버리고 현재 적용된 설정을 다시 읽는다.
모델이나 영상 토픽은 YAML 또는 launch 인자로 지정한다.
제어 파라미터는 정지 상태에서 ROS 명령으로도 수정할 수 있다.

```bash
ros2 param set /parking_camera_preview/perpendicular_camera_controller rear_position_gain 8.0
ros2 service call /parking_camera_preview/perpendicular_camera_controller/save std_srvs/srv/Trigger '{}'
ros2 service call /parking_camera_preview/perpendicular_camera_controller/reset std_srvs/srv/Trigger '{}'
```

화면 R/P도 같은 reset/save 서비스다. 저장 경로는 YAML의 `save_path`이며
기본은 `~/.config/autodrive/perpendicular_camera.yaml`이다. 모델·장치·회전·BEV
문자열/배열과 제어값을 함께 저장하며 설치된 share 파일을 덮어쓰지 않는다.
이 파일을 `config_file:=...`로 명시해서 다시 불러온다.
실차 launch의 제어기 이름은 namespace 없는 `/perpendicular_camera_controller`다.

## 좌표·조향·상태 조건

BEV는 +x가 화면 오른쪽, +y가 아래이며 yaw는 `atan2(dy, dx)` rad다.
중심선 x는 화면 하단에서의 교점/너비다. 종료선 y는 마스크 하단점/높이다.
방향 없는 선이므로 `yaw`와 `yaw+pi`를 같은 방향으로 처리한다.
수직선 방향 오차는 pi 주기로 wrap한 `yaw-pi/2`, 가로 종료선은 `yaw-0`이다.
회전은 시계 방향 0/90/180/270도, 그다음 수평 반전이며 추론보다 먼저 적용한다.

후진 조향은 `-rear_position_gain*(x-rear_target_x) - rear_angle_gain*yaw_error`다.
출력 단위는 조향 단계이며 기본 부호에서 +는 바퀴 오른쪽이다.
`steering_sign`은 설치 방향을 반영한다. deadband·포화를 거친 뒤 마지막에
정수로 반올림하며 ±7 이내다. 좌우 PWM은 동일하며 ±255 이내다.
이 소프트웨어 부호는 테스트했지만 실제 카메라 설치/조향 구동 부호를
측정한 것은 아니다.

| 상태 | 움직임에 필요한 정보·전이 |
| --- | --- |
| SEARCHING_SPACE | 전방 경계로 전진. 전방 칸이 정상 관측 5회 연속 미검출되면 좌회전 전진으로 전환. 경계선 유무와 별개로 횟수를 세며 칸이 검출되면 초기화 |
| ALIGN_TO_SPACE | 최신 전방 영상으로 제한선 접근을 판단하며 좌조향 전진. 후보 y가 기준에 닿거나, 설정한 전진 시간 경과 시 후보가 미검출이면 정지 |
| DIRECTION_PAUSE | 정상 후방 영상이 들어오면 기존 정지시간 적용. 칸 중심선 없이도 우조향 후진 시작 |
| REVERSE_RIGHT | 후방 칸 중심선이 일찍 잡혀도 `reverse_right_s`(기본 3초) 동안 고정 우조향 후진. 시간이 끝난 주기에 PARKING_IN으로 전환 |
| PARKING_IN | 후방 중심선으로 후진. 종료선이 지정 y 이후면 종료선 방향 사용. 정상 후방 영상에서 두 조향 기준을 모두 잃으면 직전 조향·PWM 유지. 정지 y 도달 시 같은 주기에 0 출력 |
| STOPPED | 3초간 정지한 뒤 자동 출차. Control 탭에서 자동 출차를 끄면 계속 정지 |
| ESCAPE | 정상 전방 영상으로 첫 직진. `escape_s`(기본 1.5초) 동안 조향 0과 `escape_pwm`(기본 200)을 사용 |
| TURN_RIGHT | 정상 전방 영상으로 최대 우조향 전진. 평행주차 모델이 통로선을 잡으면 바로 추종으로 전환. 선이 늦게 잡혀도 `exit_turn_s`(기본 10초)에서 우조향 종료 |
| FINAL_STRAIGHT | 평행주차 모델 미설정 시 조향 0 유지. `exit_straight_s`(기본 20초) 이후 정지 |
| PARALLEL_APPROACH | 전방 통로선의 x=0.35 추종. 후방 목표 칸 경계가 후진 시작 기준에 도달하면 전환 대기 |
| PARALLEL_PAUSE | 전진→후진 전환 대기 후 좌조향 후진 |
| PARALLEL_REVERSE_LEFT / STRAIGHT | 좌조향 후진으로 각도를 만들고 필요하면 직진 후진. 안쪽 선이 추종 거리에 도달하면 정렬 |
| PARALLEL_FAR_ALIGN | 안쪽 긴 선 각도와 거리로 후진 조향. 잠깐 놓치면 통로선 또는 직전 명령 유지. 정렬과 목표 깊이가 확인되면 정지 |
| FINISHED / FAULT_STOP | 정지. fault 재시작은 S/R 필요 |

원본 `allowed_states`와 `/motion_state` 의존성은 **전·후방 연속 추론**으로
명시적으로 대체했다. 프로세스는 두 개지만 차량 명령 producer는 하나다.
GPU 비용을 줄이는 상태별 추론 중단은 현재 하지 않는다.
카메라 전환/W/resume 직후에는 해당 시점보다 나중에 촬영된 이미지가 필요하다.
같은 카메라를 쓰는 즉시 전이는 확인에 사용한 영상을 유효 시간 동안 재사용한다.
같은 source stamp를 반복 읽어도 미검출 횟수가 늘지 않는다.
원본의 이미지/검출 approximate sync 0.3~1초 대신 같은 배열에서 추론과
geometry를 계산하므로 서로 다른 프레임을 결합하지 않는다.
원본 stamp 기준 age와 수신 monotonic age를 함께 검사한다.

필요 관측이 없거나 오래되면 정지하고 기동 타이머를 멈춘다.
별도 관측 상실 watchdog이 만료되면 FAULT_STOP으로 간다.
우조향 후진에는 상태 제한 시간을 추가하지 않았다. 카메라 영상 신선도와
관측 상실 watchdog은 적용된다. 기존 기동 상태의 제한 시간은 유지한다.
주차 정지 상태는 필수 관측이 없으며 S 중에는 주차 대기시간도 멈춘다.
고정 우조향 후진 시간은 명령이 나가는 동안 누적한다. 추종으로 전환할 때
또는 추종 중에 영상은 정상인데 선이 잠시 안 잡히면 직전 후진 명령을
유지하고 그 시간도 기존 상태 제한 시간에 포함한다. 추종 중 영상 문제나
S로 정지한 뒤에는 현재 영상에서 추종에 쓸 선을 확보한 후 후진한다.
자동 출차는 각 단계의 시간으로 전환한다. 첫 직진 뒤 우조향 전진과 마지막
직진은 공통으로 `exit_pwm`(기본 110)을 사용한다. 전방 출차선 검출 여부는
전환에 사용하지 않는다. 영상 문제 또는 S로 정지한 시간은 출차 시간에서
제외된다. 기존 YAML에 새 시간 설정이 없으면 우조향 10초와 마지막 직진
20초가 적용된다.

원본의 탐색 카운터와 좌조향 전진을 따르고, 전진 종료는 후방 yaw 대신
전방 평행주차선 접근으로 판단한다. 방향 전환 후 고정 우조향 후진을 먼저 수행한다.
종료선 기준은 원본 1000px와 1250px를 원본 영상 높이 1280px로 나눈 값으로
시작한다. 카메라 설치와 BEV에 맞는 정지 위치는 현장에서 보정한다.

## 실차 보정 순서와 한계

1. 전·후방 장치를 구분하고 카메라 위치·높이·방향·영상 회전/반전을 고정한다.
2. 확보한 모델의 클래스와 현장 segmentation을 확인한다.
3. 평평한 바닥의 실제 주차선으로 각 카메라 BEV 네 점을 맞춘다.
   화면 비율 정규화는 실제 거리 보정이 아니다. 해상도 **종횡비** 변경 시에도 재확인한다.
4. 후방 양쪽 주차선에서 중심선이 가운데에 오는지, 한쪽만 보일 때
   `parking_width_ratio`가 실제 BEV 폭에 맞는지 확인한다.
   기본 0.40은 원본 528px를 환산한 측정값이 아니다.
5. preview에서 좌우 위치·차체 방향을 바꿔 예상 조향 부호와 포화를 확인한다.
6. Front BEV에서 실제 평행주차선이 선택되도록 검출 범위를 맞추고 범퍼 앞
   여유 위치의 `front_limit_stop_y`를 확인한다. Rear BEV에서
   `end_steering_y`/`end_stop_y`와 후방 종료선 y를 비교한다.
7. 조향/카메라 부호와 구동 PWM을 확인하고 낮은 속도부터 gain을 조정한다.
8. 확인 후에만 calibration_confirmed를 true로 설정하고 live launch를 사용한다.
9. 주차만 검증할 때는 Control 탭의 Automatic exit를 끈다.

전방 영상이 정상이고 제한선 후보가 보이지 않으면 좌조향 전진 시간이
`align_fallback_s`(기본 5초)에 도달했을 때 정지한다. 후보가 검출되고
접근 기준에 도달하면 즉시 정지한다. S 또는 영상 문제로 정지한 시간은
전진 시간에 포함되지 않는다. 두 경우 모두 기존 전후진 전환 대기를 거친다.
시간 기준은 실제 범퍼와 제한선 사이의 거리를 측정하지 않는다.
다중 주차칸에서 같은 목표를 계속 고르는 tracking은 없으며 가장 큰 ROI를 쓴다.
목표칸 선택과 실제 검출 정확도는 현장 데이터로 추가 검증해야 한다.
이 모드는 LiDAR를 충돌 감시에 연결하지 않으며 후방 충돌 회피를 보장하지 않는다.
기존 시간 기반 주차의 geometry_confirmed를 켜거나 가짜 scan을 넣지 않는다.

```bash
ros2 launch vehicle_bringup_pkg perpendicular_camera_parking.launch.py \
  config_file:=$HOME/.config/autodrive/perpendicular_camera.yaml \
  arduino_port:=/dev/arduino
```

실제 운용은 위 보정과 모델 준비 이후에만 가능하다. 이 작업에서는 모터를 구동하지 않았다.

## 오프라인 검증

모델·ROS 없는 환경에서도 FSM과 합성 영상 geometry 테스트는 실행 가능하다.
Python, pytest, PyYAML, NumPy, OpenCV가 필요하다.

```bash
PYTHONPATH=src/skku_track_drive_pkg:src/lidar_perception_pkg:src/vehicle_bringup_pkg \
  python3 -m pytest -q tools/tests/perpendicular_camera \
  src/skku_track_drive_pkg/test/test_mission_parking.py \
  src/skku_track_drive_pkg/test/test_lane_processing.py \
  src/skku_track_drive_pkg/test/test_bbox_size_avoidance.py \
  src/skku_track_drive_pkg/test/test_bev_roi_separation.py \
  src/skku_track_drive_pkg/test/test_lidar_camera_fusion.py
```

ROS가 없으면 `test_ros_launch.py`는 skip된다. adapter 테스트는 실제 adapter의
메서드를 in-memory transport double로 검사한다. 이 변경에서는 별도로
격리된 ROS preview 토픽에 합성 관측을 보내 상태·PWM·정지 사유를 확인하고,
실제 전방 가중치로 저장 영상의 마스크와 제한선 후보를 확인했다.
실차 카메라 배치에서의 검출 정확도, 정지 여유 거리와 실차 주차는 현장 확인이 필요하다.
