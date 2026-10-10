# bbox 기준 장애물 회피 주행

기존 트랙의 YOLO 차선 처리 → BEV → 경로 생성 → Stanley 조향 계산을 사용하는 MissionControllerNode를 상속한 bbox_mission_controller_node를 추가했습니다. 기존 MissionCore의 장애물 회피와 신호등 판단을 그대로 사용합니다.

## 추가 코드

- Autonomous_Capstone-integrated/src/skku_track_drive_pkg/skku_track_drive_pkg/bbox_mission_controller_node.py
- Autonomous_Capstone-integrated/src/vehicle_bringup_pkg/launch/mission_bbox_only.launch.py
- 실행 등록: skku_track_drive_pkg/setup.py에 bbox_mission_controller_node 항목 추가

launch 상단 TEST_SPEED와 BBOX_DEFAULTS에 초기 속도 및 bbox 회피 조정값을 모았습니다. obstacle_near_y=300, path_margin_px=35, obstacle_confirm_frames=3, obstacle_clear_frames=5, alternate_min_area_px=250, avoid_hold_s=2.0, avoid_speed=80입니다. saved/explicit YAML 및 명시 launch 인자가 BBOX_DEFAULTS보다 우선합니다. speed의 기본 launch 인자는 30입니다.

## 판단 흐름

1. obstacle bbox 하단 y가 근접 기준선을 넘었는지 확인합니다(480px 기준; 실제 영상 높이에 맞춰 비례 적용).
2. 현재 BEV 주행 경로를 카메라 영상에 역투영한 선이 bbox 폭+여유 영역에 겹치는지 확인합니다.
3. 경로를 막는 물체를 발견하면 확인 프레임을 쌓는 동안 정지합니다.
4. 연속 확인 후 반대 차선 마스크가 보이고 장애물로 막히지 않았으면 선택 차선을 lane1↔lane2로 변경합니다. 변경한 차선에서 기존 경로·Stanley가 조향합니다.
5. 대체 차선이 없거나 막혔으면 정지합니다. 회피 후 선택한 차선을 유지합니다.
6. 기존 빨간불 정지 latch, 확인된 초록불 재개, 노란불 속도 제한을 함께 사용합니다.

이 모드는 camera bbox 하단 위치로 근접을 판단합니다. bbox 면적 증가율 또는 LiDAR 거리 기반 판정은 추가하지 않았습니다. 모델은 lane1, lane2, obstacle, traffic_light를 가진 기존 미션 segmentation 모델을 선택합니다.

## 빌드

Ubuntu/WSL 터미널에서 실행하세요. 현재 PC의 공유 저장소 경로는 다음과 같습니다.

```bash
cd "/mnt/c/Users/82106/OneDrive - 성균관대학교/바탕 화면/성대/26-2/자캡디/autodrive-capstone/Autonomous_Capstone-integrated"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

다른 Ubuntu 차량 PC에서는 그 PC에 있는 Autonomous_Capstone-integrated 폴더로 이동합니다.

## 카메라/판단만 확인

```bash
ros2 launch vehicle_bringup_pkg mission_bbox_only.launch.py \\
  sensors_only:=true camera_device:=/dev/video2 device:=cuda:0
```

이 실행은 실제 카메라와 GUI만 사용하며 차량 serial/arm gate를 실행하지 않습니다. 영상에서 경로·bbox, 상태에서 path_blocked와 target_lane을 확인하세요.

## 실제 주행

센서 확인 실행을 Ctrl+C로 종료하고 아래를 실행합니다.

```bash
ros2 launch vehicle_bringup_pkg mission_bbox_only.launch.py \\
  camera_device:=/dev/video2 arduino_port:=/dev/arduino \\
  device:=cuda:0 speed:=30 auto_calibrate:=false
```

/dev/video2, /dev/arduino는 실제 장치명으로 바꿉니다. CUDA가 없는 경우 device:=cpu를 사용합니다.

Mission Tuner에서 W=출발 요청, S/X/Space=정지, P=승인된 튜닝값 저장, D=영상, B=BEV 표시 전환입니다. GUI 승인 로그 이후 P로 저장합니다. GUI의 bbox 하단 y, 경로 여유, 확인/해제 프레임, 회피 유지시간·속도를 조정할 수 있습니다. 기본 주행·신호등 파라미터도 같은 GUI에서 조정합니다.

전용 저장 파일: ~/.config/autodrive/mission_bbox_only_tuning.yaml

예: 근접 기준선을 바꿔 시작하려면 실행 명령에 obstacle_near_y:=330 path_margin_px:=40을 추가합니다.

## 검증

ROS 2 Humble launch 구성 검사 통과: dry_run, sensors_only, 실제 주행 설정에서 bbox controller 단일 실행, LiDAR publisher 미실행, 모터 preview 격리 확인.

기존 미션/주차 회귀 테스트 18개 통과. 새 코드 문법 검사 및 git diff --check 통과.

실제 카메라 검출 및 차량 주행은 아직 검증하지 않았습니다. 이 코드는 sangmin 브랜치에서 사용할 수 있습니다.
