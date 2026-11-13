# 평행주차 라벨링용 전후방 수동 촬영

`manual_capture.launch.py`에서 전후방 카메라를 함께 띄우고, 수동으로 운전하면서
두 원본 JPEG를 같은 번호의 쌍으로 저장할 수 있다. 추론 모델은 실행하지 않는다.

## 빌드

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sensor_bringup_pkg manual_drive_pkg vehicle_bringup_pkg
source install/setup.bash
```

## 카메라 장치 확인

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
```

아래 예시는 현재 실행에서 확인한 **전방 `/dev/video2`, 후방 `/dev/video4`**인 경우다.
USB 연결에 따라 번호가 바뀔 수 있으므로 실제 장치에 맞게 바꾼다.
한 카메라가 제공하는 영상/메타데이터 장치 두 개를 서로 다른 카메라로 지정하지 않는다.
안정적인 장치 경로가 있으면 `/dev/v4l/by-id/...-video-index0`을 사용할 수 있다.

## 실행

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vehicle_bringup_pkg manual_capture.launch.py \
  camera_device:=/dev/video2 \
  use_rear_camera:=true \
  rear_camera_device:=/dev/video4 \
  arduino_port:=/dev/arduino \
  auto_calibrate:=False \
  auto_capture:=true \
  auto_interval:=0.20 \
  output_dir:="$PWD/datasets/parallel_parking/$(date +%Y%m%d_%H%M%S)"
```

카메라 창에 포커스를 두고 조작한다. 왼쪽은 `FRONT`, 오른쪽은 `REAR`다.
전진/후진 모두 두 카메라를 저장하며 방향에 따라 한쪽만 선택하지 않는다.
시작 시 자동 촬영은 켜져 있지만, **bridge의 주행 승인이 있고 속도 명령이
0이 아닐 때만** 기본 0.20초마다 한 쌍을 저장한다(초당 최대 5쌍, JPEG 10장).
`auto_calibrate:=False`는 시작 시 자동 조향 보정을 생략하는 기존 옵션이다.
실제 출발은 창에 `CAL=READY`가 표시된 후 W/S로 요청한다.

| 키 | 동작 |
| --- | --- |
| W | 속도 명령 +20 누적: 후진 감속 → 0 → 전진 가속 |
| S | 속도 명령 -20 누적: 전진 감속 → 0 → 후진 가속 |
| A / D | 조향 왼쪽 / 오른쪽 한 단계 누적(최대 ±7). 정지 상태에서도 구동 PWM 0으로 조향 승인 요청 |
| X / Space | 속도·조향 명령 0, 정지 및 disarm. 모든 모터 출력 OFF. 자동 촬영은 다음 출발까지 대기 |
| C | 정지 중에도 전후방 한 쌍 저장 |
| R | 자동 촬영 ON/OFF |
| Q / 창 닫기 | 정지 및 수동조작 노드 종료. 남은 launch 프로세스는 터미널 Ctrl+C로 종료 |

주차 완료 상태, 정지선 앞 대기 등 정지 장면은 C로 추가한다.
촬영 간격을 늘리려면 `auto_interval:=0.5` 등으로 변경한다.

처음 실행하거나 X/Space로 정지한 뒤에도 A/D로 제자리 조향할 수 있다.
주행 승인 전의 속도 요청은 제자리 조향 시 0으로 초기화한다.
X/Space는 조향 모터 출력도 끄므로 바퀴를 자동으로 중앙까지 돌리지는 않는다.

조향 문제를 확인하려면 별도 터미널에서 다음을 실행한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /vehicle/command_status
```

최상위 `steering`은 PC가 시리얼에 쓴 조향 명령이다.
`firmware.target`과 `firmware.pot_adc`는 STATUS를 지원하는 아두이노가 보고한 값이며,
`firmware_age_sec`로 수신 경과 시간을 확인한다. STATUS가 없는 펌웨어에서는
`firmware`가 null이다. `serial_rx`에는 최근 아두이노 응답이 표시된다.

## 저장 구조와 라벨링

```text
datasets/parallel_parking/<실행 시각>/
├── images/
│   ├── front/00000001_<시각ns>_front.jpg
│   └── rear/00000001_<시각ns>_rear.jpg
└── pairs.csv
```

원본에는 화면에 보이는 조향·속도 글씨를 그리지 않으며 반전/회전도 적용하지 않는다.
두 파일은 공통 번호·시각으로 연결되고, 접미사가 달라 한 폴더로 모아도 이름이 겹치지 않는다.
라벨링 도구에는 `images/front/`, `images/rear/`의 JPEG를 가져와 객체/폴리곤을 지정한다.
`pairs.csv`는 객체 라벨이 아니라 촬영 당시의 제어 명령과 영상 쌍을 기록한다.

CSV 필드:
`timestamp, front_filename, rear_filename, steering, left_speed, right_speed,
capture_mode, last_key, front_stamp_ns, rear_stamp_ns, pair_delta_ms`.
파일명은 `images/` 기준 상대 경로이며, 속도는 실제 측정 속도가 아닌 PWM 명령이다.

두 USB 카메라의 최신 프레임을 묶는 방식이며 하드웨어 동기 촬영은 아니다.
기본적으로 한쪽 수신이 0.50초 넘게 끊기거나 영상 헤더 시각 차이가 0.10초를
넘으면 저장을 건너뛰고 경고한다. 자동 촬영은 양쪽 모두 새 프레임이 들어와야
다음 쌍을 저장한다. 필요하면 `max_frame_age`, `max_pair_skew`를 초 단위로 지정한다.
창의 `WAIT / STALE` 표시로 수신 상태를 확인할 수 있다.

후방 카메라를 사용하지 않는 기존 명령은 그대로 쓸 수 있다.
`use_rear_camera:=false`가 기본이며 이때는 `images/*.jpg`와 기존 `labels.csv`를 쓴다.
`auto_capture` 기본값도 `false`이므로 생략하면 R로 자동 촬영을 켠다.
카메라 전송 형식은 USB 대역폭을 줄이도록 `MJPG`를 요청하며, 장치가 지원하지 않으면
`camera_fourcc:=YUYV`로 변경할 수 있다.
