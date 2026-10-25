# 실차 주행 코드 안내

주행 모드별 시작점은 아래 표를 따른다. **트랙 주행의 기존 런치·제어기·GUI·공통 센서/통신 코드는 그대로 유지했다.**
이 문서의 경로가 현재 구조의 기준이며, 과거 보고서에 적힌 이전 경로는 개발 기록이다.

| 모드 | 기존 실행 명령 뒤에 붙일 런치 이름 | 설정을 수정할 파일 | 판단 로직 |
| --- | --- | --- | --- |
| 트랙 주행 | `track_drive_tuning.launch.py` | [기존 트랙 런치](src/vehicle_bringup_pkg/launch/track_drive_tuning.launch.py), [track_tuning.yaml](src/vehicle_bringup_pkg/config/track_tuning.yaml) | [기존 track_controller_node.py](src/skku_track_drive_pkg/skku_track_drive_pkg/track_controller_node.py) |
| bbox 장애물 회피 + 신호등 | `mission_bbox_only.launch.py` | [bbox 전용 launch.py](src/vehicle_bringup_pkg/vehicle_bringup_pkg/modes/bbox_mission/launch.py) 상단 `EDIT HERE` | [bbox_mission/](src/skku_track_drive_pkg/skku_track_drive_pkg/modes/bbox_mission/) |
| bbox 장애물 회피 + 신호등 (기존 size 실행 이름) | `mission_bbox_size.launch.py` | [size_launch.py](src/vehicle_bringup_pkg/vehicle_bringup_pkg/modes/bbox_mission/size_launch.py) 상단 `SIZE_DEFAULTS` | 위 bbox 미션과 같은 Y 기준 전환 |
| 수직주차 | `perpendicular_parking.launch.py` | [수직주차 launch.py](src/vehicle_bringup_pkg/vehicle_bringup_pkg/modes/perpendicular_parking/launch.py), [perpendicular_parking.yaml](src/vehicle_bringup_pkg/config/perpendicular_parking.yaml) | [수직주차 상태 전이](src/skku_track_drive_pkg/skku_track_drive_pkg/modes/perpendicular_parking/state_machine.py) |
| 평행주차 | `parallel_parking.launch.py` | [평행주차 launch.py](src/vehicle_bringup_pkg/vehicle_bringup_pkg/modes/parallel_parking/launch.py), [parallel_parking.yaml](src/vehicle_bringup_pkg/config/parallel_parking.yaml) | [평행주차 상태 전이](src/skku_track_drive_pkg/skku_track_drive_pkg/modes/parallel_parking/state_machine.py) |

각 런치는 `ros2 launch vehicle_bringup_pkg <런치 이름>`으로 실행한다. 모드는 한 번에 하나씩 실행한다.
카메라와 Arduino 포트는 실제 차량에 맞춰 `camera_device:=...`, `arduino_port:=...`로 지정한다.
기존 W 출발 승인과 정지 처리는 유지된다.

```text
src/
├── vehicle_bringup_pkg/
│   ├── launch/                      # 기존 ROS 실행 이름을 유지하는 진입점
│   │   ├── track_drive_tuning.launch.py  # 기존 코드 유지
│   │   └── ...                      # 다른 모드의 짧은 연결 파일
│   └── vehicle_bringup_pkg/
│       ├── modes/
│       │   ├── bbox_mission/         # bbox·신호등 런치 및 조정값
│       │   ├── perpendicular_parking/
│       │   └── parallel_parking/
│       └── development/             # 실험·보정·센서 점검 런치
├── skku_track_drive_pkg/
│   └── skku_track_drive_pkg/
│       ├── track_controller_node.py # 기존 트랙 제어기 유지
│       ├── track_tuner_node.py      # 기존 트랙 GUI 유지
│       ├── modes/
│       │   ├── bbox_mission/        # 장애물·신호등 판단과 제어기
│       │   ├── perpendicular_parking/ # 수직주차 상태 전이
│       │   ├── parallel_parking/    # 평행주차 상태 전이
│       │   └── parking_common/      # 공통 주차 제어기·검증·LiDAR 계산
│       └── development/            # 보정·LiDAR/카메라 융합 실험
├── sensor_bringup_pkg/              # 공통 센서 입력
├── vehicle_io_pkg/                  # 공통 출발 승인·시리얼 출력
├── interfaces_pkg/                  # 공통 ROS 메시지
├── manual_drive_pkg/                # 트랙의 선택적 주행 중 캡처에서도 사용
├── lidar_perception_pkg/            # 주차 공통 시간표 parking_timing을 제공
└── control/                        # 기존 펌웨어 경로 유지
development/
└── legacy/                         # 과거 파이프라인 6개, 기본 빌드에서 제외
tools/                              # 기존 개발·분석·검증 도구
```

주차의 공통 정지 조건·입력 검증·위치 추정은 `modes/parking_common/`에 있고,
각 모드의 상태 전이는 `perpendicular_parking/`, `parallel_parking/`에 있다.
공통 GUI와 파라미터 정의는 기존 `mode_tuner_node.py`, `mode_parameters.py`를 공유한다.
bbox 미션은 기존 트랙 제어기를 상속하므로 트랙의 차선·경로·Stanley 계산도 공유한다.

이전 Python 모듈 경로에는 짧은 호환 연결 파일을 남겼다.
예를 들어 기존 `parking_core.py`는 새 `modes/parking_common/parking_core.py`를 불러온다.
실제 로직 수정은 표에 연결된 새 파일에서 한다. 알고리즘을 두 벌로 복사하지 않았다.

## 모델 파일

- 트랙 모델 선택과 기존 `models/` 파일은 유지했다.
- bbox 미션의 기본 모델은 `src/skku_track_drive_pkg/models/mission/best_m.pt`다.
- 다른 미션 모델은 실행 인자 `model_path:=/절대경로/모델.pt`로 지정할 수 있다.
- 미션 모델은 `lane1`, `lane2`, `obstacle`(또는 `obstacles`), `traffic_light` 클래스가 있는 segmentation 모델이어야 한다.

## 빌드와 실행

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash

# 아래 중 필요한 모드 하나를 실행
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py
ros2 launch vehicle_bringup_pkg mission_bbox_only.launch.py
ros2 launch vehicle_bringup_pkg mission_bbox_size.launch.py
ros2 launch vehicle_bringup_pkg perpendicular_parking.launch.py
ros2 launch vehicle_bringup_pkg parallel_parking.launch.py
```

보정·실험·과거 코드의 위치와 기존 명령 호환 범위는 [개발 코드 안내](development/README.md)를 참고한다.
트랙의 선택적 녹화·분석·캡처와 관련된 기존 파일도 유지했다.

두 bbox 실행 모두 현재 목표 차선의 관측 마스크 위 장애물 bbox 하단이
`obstacle_near_y` 이상이면 즉시 반대 차선으로 전환한다. 장애물로 인한 정지·감속,
면적/연속 검출/반대 차선 확인 조건은 사용하지 않는다.
전환 후 5초간은 추가 자동 회피를 막고 새 목표 차선을 유지한다. 쿨다운 때문에 정지하지 않는다.
미션 GUI 또는 디버그 창에 포커스를 두고 **L**을 누르면 목표 차선을 `1↔2`로
강제 전환한다. L은 쿨다운 중에도 동작하며 누른 시점부터 다시 5초간 자동 회피를 막는다.
목표 차선과 남은 쿨다운은 GUI/디버그에 표시된다. `obstacle_near_y`는
`Mission Tuner Obstacle` 창에서 조절한다. 이전 YAML의 `lane_change_cooldown_s`와
무관하게 현재 bbox 미션의 쿨다운은 5초로 고정한다.
새 목표 차선 경로는 다음 영상 프레임에서 계산하며, 전환 중에는 기존 주행 명령을 유지한다.
신호등 빨강 정지·노랑 감속과 W/S·통신 정지는 유지한다.
과거 면적 회피 이식 내역은 [반영 기록](docs/SANGMINV2_OBSTACLE_IMPORT_KR.md)에 정리했다.
