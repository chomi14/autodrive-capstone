# 카메라 기반 수직주차

전방 주차 구역 추종과 후방 중심선·종료선 기반 후진 주차를 제공한다.
기존 `perpendicular_parking.launch.py`의 시간 기반 주차와 독립적인 실행 모드다.
기본 설정은 **예시이며 실측값이 아니다**. 전용 가중치와 카메라 보정이 필요하다.

## 파일과 실행 연결

| 위치 | 역할 |
| --- | --- |
| 이 폴더의 `observations.py` | 한 원본 프레임에서 나온 관측 및 좌표 규약 |
| `perception.py` | BEV, 전방 경계, 후방 Hough 중심선, 종료·출차선 계산 |
| `model.py` | Ultralytics segment 모델과 실제 class 이름 대응 |
| `perception_node.py` | 카메라 하나당 인식 프로세스 하나 |
| `core.py` | ROS가 없는 상태 전이와 조향 계산 |
| `controller_node.py` | 20 Hz 명령 발행, 관측 검사, W/S 상태, reset/save |
| `viewer_node.py` | 전·후방 6개 영상, 상태 표시, W/S/R/P |
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

| 카메라 | 이 모드의 의미 키 | 원본 class_name | 원본 출력 | 이 구현 |
| --- | --- | --- | --- | --- |
| 전방 | parking_lot | parking_lot | ParkingLot: found, x 정규화, yaw rad | 합친 ROI의 왼쪽 경계 |
| 전방 | parking_space | parking_space | ParkingSpace: 탐색에서 found 사용 | 주차공간 검출 이력 |
| 전방 | out_line | out_line | OutLine: x/y 정규화, yaw rad | 출차선 x/y 정규화, yaw rad |
| 후방 | parking_space | parking_space | ParkingSpace: 하단 x 정규화, yaw rad | ROI 안 영상 선으로 중심선 계산 |
| 후방 | end_line | end_line | EndLine: 원본 x/y는 pixel, yaw rad | x/y 정규화, yaw rad |

고정 class ID는 가정하지 않는다. `classes`의 값은 **확보한 모델의 실제 클래스 이름**이다.
시작 시 `model.names`로 필수 클래스 존재를 검사하고 ID를 연결한다.
Ultralytics 호환 segment `.pt`, `.onnx`, `.engine`을 지원하는 어댑터이며,
각 backend의 설치 의존성과 실제 추론은 가중치를 확보한 환경에서 확인해야 한다.
engine은 해당 GPU·TensorRT 환경에 맞게 준비한다. 박스 전용 모델은 사용할 수 없다.
추론 polygon은 영상 회전/반전 후 이미지의 pixel 좌표로 처리한다.

## 설치와 모터 없는 확인

ROS 2 Humble/Ubuntu 22.04 작업공간에서 기존 의존성에 더해 `ultralytics`,
NumPy, OpenCV, PyYAML이 필요하다. ROS의 `cv_bridge`와 호환되는 Python·NumPy
환경을 사용한다. 이 변경만을 위해 NumPy/OpenCV 시스템 버전을 임의로 올리지 않는다.

아래 명령은 `Autonomous_Capstone-integrated`에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash
mkdir -p ~/.config/autodrive
cp src/vehicle_bringup_pkg/config/perpendicular_camera.yaml ~/.config/autodrive/perpendicular_camera.yaml
```

복사한 YAML에서 모델 경로, 장치, 회전/반전, BEV를 설정한다.
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

외부 영상 토픽을 쓰려면 preview에서 `source:=topics`를 사용한다.
rosbag 재생은 명시적으로 `source:=replay use_sim_time:=true`를 사용한다.
이때 카메라 장치는 열지 않으며, 별도 터미널에서 실제 녹화 bag을
`ros2 bag play /absolute/path/to/bag --clock`로 재생한다.
bag의 Image 토픽 이름은 YAML의 `image_topic`에 맞춰야 한다.
bag 재시작으로 시계가 뒤로 이동하면 S/R로 reset한다.
live launch에서는 replay와 simulated clock을 허용하지 않는다.

## 설정 변경과 저장

기존 숫자형 주차 튜너와 별도의 YAML 스키마다. 카메라·모델·BEV 변경에는
재시작이 필요하다. 제어 파라미터는 정지 후 ROS 파라미터로 수정할 수 있으며
수정하면 FSM이 초기화된다. 타입을 유지해야 한다.

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
| SEARCHING_SPACE | 새 전방 경계가 있어야 전진. 주차공간을 confirm_frames번 본 뒤 confirm_frames번 사라져야 진입 |
| ALIGN_TO_SPACE | 최신 후방 중심선이 있어야 전진 회전. 각도 오차 절댓값이 허용치 이내인 새 프레임이 누적되면 정지 |
| DIRECTION_PAUSE | 후방 중심선을 유지하며 전후진 전환 정지시간 확보 |
| PARKING_IN | 후방 중심선으로 후진. 종료선이 지정 y 이후면 종료선 방향 사용. 정지 y 도달 시 같은 주기에 0 출력 |
| STOPPED | 주차 상태 유지. 출차는 기본 비활성화 |
| ESCAPE | 출차 활성화 시 최신 전방 관측과 함께 제한 시간 직진 |
| TURN_RIGHT | 최신 전방 출차선을 따라 진행. 출차선 상실 시 정지 |
| FINAL_STRAIGHT | 최신 전방 관측과 함께 제한 시간 직진 후 FINISHED |
| FINISHED / FAULT_STOP | 정지. fault 재시작은 S/R 필요 |

원본 `allowed_states`와 `/motion_state` 의존성은 **전·후방 연속 추론**으로
명시적으로 대체했다. 프로세스는 두 개지만 차량 명령 producer는 하나다.
GPU 비용을 줄이는 상태별 추론 중단은 현재 하지 않는다.
상태 변경/W/resume 직후에는 해당 시점보다 나중에 촬영된 이미지가 필요하다.
같은 source stamp를 반복 수신해도 confirmation 횟수가 늘지 않는다.
원본의 이미지/검출 approximate sync 0.3~1초 대신 같은 배열에서 추론과
geometry를 계산하므로 서로 다른 프레임을 결합하지 않는다.
원본 stamp 기준 age와 수신 monotonic age를 함께 검사한다.

필요 관측이 없거나 오래되면 정지하고 기동 타이머를 멈춘다.
별도 관측 상실 watchdog이 만료되면 FAULT_STOP으로 간다.
기동 상태는 유효 관측이 계속 들어와도 상태별 timeout을 초과하면 fault다.
주차 정지 상태는 필수 관측이 없으며 S 중에는 주차 대기시간도 멈춘다.

원본에서 수정한 부분은 초기 미검출 진입, timer 기반 중복 확인,
방향 부호가 뒤집히는 fitLine, 미초기화 중심값, 종료선 판정 후 한 주기 후진,
인식 상실 시 전진 복구, 전후진 즉시 전환, 제한 없는 출차다.
원본의 ALIGN yaw `>=0.65`와 반대 의미 로그는 폐기하고
**수직 목표 방향 오차의 절댓값**으로 조건을 정의했다.

## 실차 보정 순서와 한계

1. 전·후방 장치를 구분하고 카메라 위치·높이·방향·영상 회전/반전을 고정한다.
2. 확보한 모델의 클래스와 현장 segmentation을 확인한다.
3. 평평한 바닥의 실제 주차선으로 각 카메라 BEV 네 점을 맞춘다.
   화면 비율 정규화는 실제 거리 보정이 아니다. 해상도 **종횡비** 변경 시에도 재확인한다.
4. 후방 양쪽 주차선에서 중심선이 가운데에 오는지, 한쪽만 보일 때
   `parking_width_ratio`가 실제 BEV 폭에 맞는지 확인한다.
   기본 0.40은 원본 528px를 환산한 측정값이 아니다.
5. preview에서 좌우 위치·차체 방향을 바꿔 예상 조향 부호와 포화를 확인한다.
6. 실제 정지시킬 범퍼 위치에서 종료선 y를 측정해 end_steering_y/end_stop_y를 설정한다.
   원본 1000/1250px는 그대로 이식하지 않았다.
7. 조향/카메라 부호와 구동 PWM을 확인하고 낮은 속도부터 gain을 조정한다.
8. 확인 후에만 calibration_confirmed를 true로 설정하고 live launch를 사용한다.
9. 출차는 주차 검증 후 별도로 exit_enabled를 켜서 확인한다.

ALIGN 진입 시 후방 선이 아직 안 보이면 자동으로 전진하며 찾지 않고 정지한다.
이 경우 후방 카메라 시야·진입 위치를 먼저 맞춰야 한다. 영상 없이 진행하는
탐색 회전이나 복구를 별도로 구현한 것처럼 취급하지 않는다.
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
메서드를 in-memory transport double로 검사하며 ROS 통신을 검증한 것은 아니다.
합성 영상은 geometry 계산을 검증하는 fixture이며 현장 영상이나 모델 정확도의
증거가 아니다. 실제 가중치 추론, colcon build, ROS launch, 카메라 연결, 지연과
프레임률, 실차 주차는 이 환경에서 검증하지 못했다.
