# 1009 Drive Tuning + bbox 장애물 회피

기존 `skku_track_drive_pkg`의 TrackController 주행과 BEV/ROI 튜닝을 그대로 사용하며,
옵션을 켰을 때만 bbox 미션 컨트롤러와 obstacle 클래스가 있는 미션 모델을 사용합니다.
LiDAR 융합이 아닌 카메라 bbox 하단 기준 회피입니다.

## 설치 및 실행 (차량의 Ubuntu / ROS 2 Humble)

```bash
git switch 1009
git pull --ff-only origin 1009
cd Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash

# 먼저 모터 없이 영상/튜닝 확인
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py bbox_obstacle_avoidance:=true sensors_only:=true camera_device:=/dev/video2 device:=cuda:0 speed:=30

# 실제 주행: 장치 경로 확인 후 실행
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py bbox_obstacle_avoidance:=true camera_device:=/dev/video2 arduino_port:=/dev/arduino device:=cuda:0 speed:=30 auto_calibrate:=false
```

CPU 사용 시 `device:=cpu`. 원래 주행은 `bbox_obstacle_avoidance` 옵션을 빼면 됩니다.
기본 PWM은 기존 1009와 같은 250이므로 초기 시험에는 반드시 낮은 `speed`를 지정하세요.
기존 W 출발 게이트와 정지/조향 보정 흐름을 유지합니다. 실제 차량 시험은 별도 필요합니다.

## 튜닝과 회피

- 조정값은 `src/vehicle_bringup_pkg/launch/mission_bbox_only.launch.py` 상단에 모았습니다.
- 기존 `~/.config/autodrive/track_tuning.yaml`의 주행/BEV/ROI 값을 읽고, 실행 인자가 최종 우선입니다. 원래 저장 파일은 수정하지 않습니다.
- GUI P 저장은 별도 `~/.config/autodrive/mission_bbox_only_tuning.yaml`입니다. 저장된 bbox 설정으로 재시작하려면 `ros2 launch vehicle_bringup_pkg mission_bbox_only.launch.py load_saved_tuning:=true`를 사용하세요 (장치 인자 추가).
- 현재 경로와 겹치며 bbox 하단이 `obstacle_near_y`를 넘은 obstacle만 판단합니다. 연속 확인 후 옆 차선이 보이고 비어 있을 때 전환하고, 안전한 대체 차선이 없으면 정지합니다.
- 핵심 값: `obstacle_near_y`, `path_margin_px`, `obstacle_confirm_frames`, `obstacle_clear_frames`, `avoid_speed`, `avoid_hold_s`. 신호등 로직도 기존 미션 기능을 사용합니다.
- BEV 좌표를 GUI에서 바꾸면 장애물 경로 판정도 동일한 변환을 즉시 사용합니다.

## 회귀 검사

ROS 환경 및 위 빌드 후:

```bash
python3 tools/tests/test_bbox_launch_assembly.py
python3 -m pytest -q src/skku_track_drive_pkg/test/test_mission_parking.py src/skku_track_drive_pkg/test/test_lane_processing.py tools/tests/test_bev_source_tuning.py tools/tests/test_bbox_1009_integration.py
```
