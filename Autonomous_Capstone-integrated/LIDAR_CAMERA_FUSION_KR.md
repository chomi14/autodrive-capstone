# 상민: LiDAR 테스트·보정 및 카메라+LiDAR 장애물 주행

기존 트랙/미션 controller와 launch는 그대로 두고 별도 실행 모드를 추가했습니다. 기존 파일 변경은 skku_track_drive_pkg/setup.py의 실행 프로그램 등록 2줄뿐입니다. 새 모드는 동일한 차선 처리·경로·Stanley·신호등·W/S gate를 재사용합니다.

## 파일과 조정 위치

- src/skku_track_drive_pkg/skku_track_drive_pkg/lidar_camera_fusion.py: 새 조정값 전체를 상단 SETTINGS에 모았습니다. scan topic, 저장 경로, GUI 주기도 상단에 있습니다.
- lidar_calibration_node.py: 슬라이더 범위/간격 SLIDERS, 화면 설정은 상단.
- fusion_mission_core.py: bbox 경로 판단 + 거리 확인 + 기존 미션 FSM 연결.
- fusion_mission_controller_node.py: timestamp로 가까운 scan 선택, 센서 watchdog, 영상 거리 표시.
- src/vehicle_bringup_pkg/launch/lidar_calibration.launch.py: LiDAR + 화면 전용; 모터 노드 없음.
- src/vehicle_bringup_pkg/launch/mission_lidar_camera.launch.py: 별도 융합 미션. 초기 PWM TEST_SPEED=30.

기본 주행 조정은 기존 mission_tuning.yaml / Mission Tuner를 사용합니다. 새 주행 모드의 P 저장은 ~/.config/autodrive/mission_lidar_camera_tuning.yaml로 분리했습니다.

## 실행 환경 및 빌드

이 프로젝트는 Ubuntu ROS 2 Humble용입니다. 아래 명령은 Windows PowerShell이 아니라 차량 Ubuntu 또는 ROS가 설치된 WSL Ubuntu 터미널에서 실행합니다. WSL에서는 카메라·LiDAR USB 장치가 Ubuntu에 전달되어 있어야 합니다.

현재 PC의 WSL에서는 다음 경로를 사용할 수 있습니다. 다른 차량 PC에서는 복사한 저장소의 Autonomous_Capstone-integrated 폴더로 이동합니다.

```bash
cd '/mnt/c/Users/82106/OneDrive - 성균관대학교/바탕 화면/성대/26-2/자캡디/autodrive-capstone/Autonomous_Capstone-integrated'
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

새 터미널마다 ROS setup 및 install/setup.bash를 source합니다. /dev/lidar가 없으면 lidar_port:=/dev/ttyUSB0처럼 실제 장치명을 지정합니다.

## 1. LiDAR 켜고 화면·각도 보정

```bash
ros2 launch vehicle_bringup_pkg lidar_calibration.launch.py \\
  lidar_port:=/dev/lidar lidar_rotation:=180.0
```

- 검정 top-view 화면: 위=차량 전방, 왼쪽=차량 좌측. 녹색=현재 유효 반사점, 붉은 점=LiDAR 위치, 파란 선=카메라 수평 시야. 입력이 끊기면 STALE 표시와 흐린 점이 나옵니다.
- Controls 창의 슬라이더를 마우스로 드래그하면 화면과 융합 설정에 즉시 반영됩니다.
- 전방에 장애물을 놓고 yaw_deg를 조절해 화면 위쪽에 맞춥니다. 왼쪽 실제 물체가 오른쪽에 표시되면 angle_sign을 바꿉니다.
- driver의 lidar_rotation은 upstream 회전입니다. yaw_deg에는 그 이후 남은 오차만 입력해 회전을 중복 적용하지 않습니다.
- lidar_x_m/lidar_y_m는 카메라 원점 대비 LiDAR 위치입니다. +x=전방, +y=좌측. 주차 코드의 뒷차축 기준 값과 구분해야 합니다.
- P: 승인된 보정값 JSON 저장. Q/Esc 또는 창 닫기: 종료 및 실행 중 confirmed=0 발행. 파일에 저장된 값은 유지됩니다. 전체 launch는 Ctrl+C로 종료합니다.

## 2. 모터 없이 카메라+LiDAR 확인

먼저 위 LiDAR 전용 launch를 Ctrl+C로 종료합니다. 두 launch를 동시에 실행하면 같은 serial LiDAR를 중복 사용합니다.

```bash
ros2 launch vehicle_bringup_pkg mission_lidar_camera.launch.py \\
  sensors_only:=true camera_device:=/dev/video2 \\
  lidar_port:=/dev/lidar lidar_rotation:=180.0 device:=cuda:0
```

카메라 Debug 화면의 obstacle bbox에는 거리, 매칭 점 수 n, 경로 겹침 path가 표시됩니다. Calibration Controls에서 camera_cx_px, camera_fx_px, camera_yaw_deg를 조절하면서 카메라 중앙·좌·우에 놓은 물체의 bbox가 실제 LiDAR 거리와 일치하는지 확인합니다. 카메라 물체가 bbox로 검출되어야 화면에 거리 결과가 나옵니다.

confirmed=0 상태에서는 CALIBRATION_NOT_CONFIRMED 정지입니다. 실제 전방·좌우 물체와 거리를 확인한 뒤 confirmed를 1로 드래그하고 P로 저장합니다. GUI 슬라이더로 실시간 변경한 값은 코드 기본값보다 우선하며, 다음 실행은 저장 JSON을 먼저 읽습니다. 잘못된 거리 순서는 적용을 거부하고 confirmed를 0으로 만듭니다.

## 3. 저속 융합 미션 실행

센서 전용 launch를 종료한 뒤 실행합니다.

```bash
ros2 launch vehicle_bringup_pkg mission_lidar_camera.launch.py \\
  camera_device:=/dev/video2 lidar_port:=/dev/lidar \\
  lidar_rotation:=180.0 arduino_port:=/dev/arduino \\
  device:=cuda:0 speed:=30 auto_calibrate:=false
```

Mission Tuner에서 W=출발 요청, S/X/Space=정지, P=기본 주행·미션 YAML 저장입니다. LiDAR Calibration 창의 P는 LiDAR JSON 저장입니다. 두 저장 파일은 서로 다릅니다. 차량 READY 및 GUI heartbeat 등 기존 출발 조건을 충족해야 실제 구동합니다. 조정은 S로 정지한 상태에서 진행합니다.

GPU가 없으면 device:=cpu를 사용할 수 있지만 추론이 image_timeout_s보다 오래 걸리면 새 융합 모드는 정지합니다. 관측한 지연에 맞춰 값을 조정합니다.

## 장애물 판단 동작

1. bbox 하단 높이에서 기존 역투영 주행 경로와 bbox 폭이 겹치는지 검사합니다. 이 새 모드는 근접 여부를 픽셀 y 대신 LiDAR 거리로 판단하므로 기존 obstacle_near_y는 융합 판정에서 사용하지 않습니다.
2. LiDAR 점을 camera_fx/cx/yaw와 장착 위치로 카메라 수평 픽셀에 연결합니다. bbox 폭+margin 안의 점만 모읍니다.
3. 서로 떨어진 거리 군집을 분리합니다. min_points 이상인 군집 하나만 있을 때 거리의 20% 백분위 값을 사용합니다. 여러 거리 군집이면 AMBIGUOUS_DEPTH, 점이 부족하면 NO_MATCH입니다.
4. 현재 경로 장애물이 near_m 이내이면 기존 연속 확인·대체 차선 검사·차선 전환을 사용합니다. stop_m 이내이면 차선 전환보다 정지를 우선합니다. 경로 밖 가까운 물체는 현재 경로 회피를 유발하지 않으며 대체 차선 점유 검사에는 사용됩니다.
5. 현재 경로 장애물의 매칭 실패, scan/image 지연, timestamp 불일치, 미보정 상태는 정지합니다. 유효 scan 점이 전혀 없어도 정지합니다. 일반 LiDAR 근접점만으로 경로 밖 물체에 회피를 걸지 않습니다.
6. 기존 신호등 HSV·빨간불 latch·초록 확인·노란불 감속을 재사용합니다.

## 자주 조정하는 값

| 값 | 초기값 | 의미 |
|---|---:|---|
| yaw_deg / angle_sign | 0 / +1 | LiDAR 방향·좌우 보정 |
| camera_fx_px / camera_cx_px | 554 / 320 | 640px 기준 카메라 수평 정렬, 초기 추정값 |
| camera_yaw_deg | 0 | 카메라 광축 방향 |
| lidar_x_m / lidar_y_m | 0 / 0 | 카메라 대비 LiDAR 실측 위치 |
| near_m / stop_m | 1.5 / 0.45 | 회피 판단 거리 / 즉시 정지 거리 |
| min_points | 3 | 유효 거리 군집 최소 점 수 |
| margin_px | 8 | bbox 수평 매칭 여유 |
| max_cluster_gap_m | 0.20 | 거리 군집 분리 기준 |
| scan_timeout_s / image_timeout_s | 0.40 / 0.75 | 입력 지연 정지 기준 |
| max_pair_delta_s | 0.25 | 영상/scan timestamp 최대 차이 |
| confirmed | 0 | 실제 정렬 확인 후 1 |

## 검증 상태와 한계

합성 scan·bbox 테스트 10개 통과: 방향 회전, 거리 매칭, 점 부족, 여러 거리 군집, 경로 밖 장애물, 먼 경로 장애물, 가까운 경로 장애물 차선 전환, 미매칭/위험 근접 정지, 빨간불 latch 유지, 잘못된 파라미터 거부.

WSL ROS 2 Humble에서 launch 조립 테스트 통과: dry_run / sensors_only / 실제 모드 각각의 LiDAR 실행 여부, 센서 전용 모드에서 serial 미실행, 융합 controller 단일 실행, 초기 PWM 30을 확인했습니다. 이 검사는 노드를 실제 실행하거나 USB 장치·모터를 열지 않습니다.

실제 LiDAR·카메라 화면, USB 연결, 모터 주행은 아직 검증하지 않았습니다. 현재 매칭은 2D LiDAR의 수평 투영입니다. 물체 높이·카메라 pitch·가림을 해결하는 완전한 3D 외부 보정은 포함하지 않았으며, 같은 방향의 서로 다른 물체가 한 거리 군집으로 합쳐지면 오매칭할 수 있습니다. 전방/좌우·여러 거리의 실제 물체로 먼저 확인해야 합니다.
