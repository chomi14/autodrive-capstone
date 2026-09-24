# contest_ws 통합 운용

이 워크스페이스는 `Autonomous_Capstone-integrated`를 underlay로 사용한다.
contest의 segmentation/차선 판단은 유지하고, 실제 차량 출력은 기준 구현의
`vehicle_io_pkg`와 `driving_user_pins.ino`를 사용한다.

## 현재 차량 하드웨어 계약

| 항목 | 값 |
|---|---|
| Steering | IN1 D3, IN2 D2 |
| Right motor | IN1 D4, IN2 D5 |
| Left motor | IN1 D7, IN2 D6 |
| Steering potentiometer | A2 |
| Arduino serial | `/dev/arduino`, 115200 baud |
| Steering command | -7 .. +7 |

`src/control_pkg/arduino/ultrasonic_control.ino`는 별도 초음파 보드용 예제다.
구동 Arduino에 업로드하면 핀이 충돌하므로 사용하지 않는다.

구동 Arduino에는 다음 중 하나를 업로드한다. 두 파일은 같은 기준 펌웨어를 가리킨다.

```text
driving/driving.ino
../../Autonomous_Capstone-integrated/src/control/driving_user_pins/driving_user_pins.ino
```

ADC endpoint/center 값은 핀 번호와 별개인 차량별 보정값이다. 차륜을 띄우고
`steering_limit_calibration.ino`로 측정하기 전에는 임의로 바꾸지 않는다.

## 빌드

상위 Git 저장소 전체에서 `colcon build`를 실행하지 않는다. 같은 이름의 과거 패키지가
여러 워크스페이스에 있기 때문이다. 각 워크스페이스를 분리해서 빌드한다.

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install

cd /home/autolab/autodrive_ws/contest_ws-main/contest_ws-main
./tools/build.sh
```

## 실행

기본값은 전방 카메라 한 대와 YOLO 한 인스턴스만 사용한다. 지연을 줄이기 위해
카메라 큐는 1장, perception QoS는 best-effort이고 디버그 영상 발행은 꺼져 있다.

```bash
./tools/run_drive.sh \
  front_camera:=/dev/video2 \
  arduino_port:=/dev/arduino \
  device:=cuda:0 \
  max_speed:=80
```

launch 후 `READY`를 확인하고 `drive_arm_node` 창/터미널에서 `W`를 눌러야 실제 출력이
허용된다. `X` 또는 Space는 즉시 DISARM한다.

보조 카메라 신호등 인식을 함께 켜려면 다음을 추가한다.

```bash
use_aux:=true aux_camera:=/dev/video4
```

GPU 여유가 없으면 `use_aux:=false`를 유지한다. 디버그 마스크/BEV가 필요할 때만
`publish_debug:=true`를 사용한다.

조향 방향이 반대로 확인되면 배선을 바꾸기 전에 저속·차륜 부상 상태에서 다음을 시험한다.

```bash
steering_sign:=-1 max_speed:=40
```

장치 번호는 재부팅 때 바뀔 수 있으므로 가능한 경우 `/dev/v4l/by-id/...`와
`/dev/serial/by-id/...` 또는 기존 `/dev/arduino` alias를 사용한다.
