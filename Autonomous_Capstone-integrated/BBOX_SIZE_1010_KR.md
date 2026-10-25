# 최신 1010 주행 + bbox 면적 기준 차선 변경

기준 커밋: 1010의 ba0f401 (2026-10-17).
ROS 차선/BEV 추출, ROI 변경 시 차량 기준점 유지, CubicSpline 및 Stanley 계산은 수정하지 않았습니다.
기존 기본 주행/하단 bbox 실행은 유지하고, 새 mission_bbox_size.launch.py에서만 면적 모드를 켭니다.
Arduino 펌웨어나 C:/LaneProject의 0713 파일은 이 작업에서 수정하지 않았습니다.

## 책 코드에서 참고한 부분

ros2_autonomous_vehicle_book의 decision_making_pkg/motion_planner_node.py는
target_lane을 lane1 ↔ lane2로 바꾸고 변경 후 감속·쿨다운을 적용합니다.
그 파일의 전환 대상은 traffic_light bbox이며, LiDAR 장애물은 정지하는 방식입니다.
장애물 bbox 면적 회피가 원래 구현돼 있던 것은 아닙니다.

이번 구현은 전환/감속/쿨다운 구조만 참고하고, 최신 1010의 Stanley 주행을 사용합니다.
책의 10초 쿨다운·20초 저속 수치를 그대로 옮기지 않고 2초 재전환 제한·기존 회피 유지 시간을 사용합니다.
쿨다운 중 현재 차선도 막히면 정지합니다.

## 회피 조건

1. obstacle 또는 obstacles 클래스의 bbox가 현재 경로 또는 현재 차선의 원본 마스크와 겹침.
2. 화면 안에 보이는 bbox 면적 / 전체 영상 면적이 0.02 이상.
   640x480 기준 6144px²이며, 거리(m) 추정이나 이전 프레임 대비 증가율은 아닙니다.
3. bbox 하단이 y=300 이상 (480px 높이 기준, 해상도에 비례).
4. 반대 차선 마스크가 현재 프레임에 보이고 그 차선과 겹치는 다른 장애물이 없음.

위 조건 충족 첫 프레임에 반대 차선을 요청합니다.
요청에 따라 면적 모드 기본값은 obstacle_stop_enabled=0입니다. 전환 프레임·반대 차선 미검출·
변경 경로 미생성·쿨다운으로는 정지하지 않고 기존 주행 명령을 회피 속도 상한 안에서 사용합니다.
반대 차선이 안 보이거나 막혔으면 전환을 하지 않은 채 계속 움직일 수 있습니다.
이는 장애물과 충돌할 수 있는 동작입니다. 새 경로가 없을 때에도 이전 조향을 사용할 수 있으므로
차체를 고정한 상태에서 먼저 확인하세요. 1로 설정하면 기존 장애물 정지 조건을 복원합니다.
빨간불/초록불 래치와 수동 정지, 송신기의 통신/출발/인지 안전 게이트는 해제하지 않습니다.
전환은 lane2→lane1, lane1→lane2 모두 지원하며 회피 후 원래 차선으로 무조건 복귀하지 않습니다.
원본 화면 반대편 절반을 고정해서 선택하지 않고 현재 target_lane의 반대 클래스를 선택합니다.
이 판단은 픽셀 관측 휴리스틱이며 실제 빈 공간/충돌 여유를 보증하지 않습니다.

## Ubuntu 실행

변경 코드가 있는 ROS 작업공간에서:

    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
    source install/setup.bash

먼저 모터 없이 카메라·GUI 확인:

    ros2 launch vehicle_bringup_pkg mission_bbox_size.launch.py sensors_only:=true camera_device:=/dev/video2 device:=cuda:0 speed:=30

차량 장치 경로/펌웨어 확인 후 실제 주행:

    ros2 launch vehicle_bringup_pkg mission_bbox_size.launch.py camera_device:=/dev/video2 arduino_port:=/dev/arduino device:=cuda:0 speed:=30 auto_calibrate:=false

기존 GUI와 W 출발/정지 gate를 유지합니다. 별도 기본 주행 노드를 동시에 실행하지 마세요.
LiDAR 노드는 실행하지 않습니다. 초기 차량 시험은 바퀴를 띄워 방향·정지를 확인하세요.
ROS sender는 기존 저장소의 정수 조향/115200 펌웨어용입니다. 로컬에서 수정한
0707 float 펌웨어와 동일한 프로그램/프로토콜이라고 가정하지 마세요.

## 모델 주의

기본 best_v2.pt는 lane2/traffic_light만 있어서 이 회피에 사용할 수 없습니다.
기본 미션 모델 models/mission/best.pt를 사용합니다. lane1/lane2/obstacle(또는 obstacles)/traffic_light
클래스가 있는 segmentation 모델이면 model_path 인자로 교체할 수 있습니다.
따라서 주행 계산 코드는 같아도 트랙 모델과 검출 결과까지 동일하지는 않습니다.

## 튜닝

새 launch 파일 상단 SIZE_DEFAULTS에 주요 값을 모았습니다:

- obstacle_min_area_ratio=0.02: 작게 하면 더 작은/먼 bbox에서도 반응.
- initial_target_lane=2: 출발 차선 번호. lane1에서 출발하면 1로 지정.
- obstacle_near_y=300: 작게 하면 하단 근접 조건을 일찍 충족. 0은 명시적으로 하단 조건 해제.
- lane_change_cooldown_s=2.0: 재전환 최소 간격. 최초 전환에는 대기 없음.
- avoid_speed=80: 회피 PWM 상한. 기본 speed보다 높이지 않음.
- obstacle_trigger_mode=1: 면적+하단 모드. 0이면 기존 하단/확인 프레임 방식.
- obstacle_stop_enabled=0: 장애물 미션 자동 정지만 해제. 1이면 기존 정지 조건 유지.

실행 시 같은 이름의 인자가 최종 우선입니다. 예:

    ros2 launch vehicle_bringup_pkg mission_bbox_size.launch.py sensors_only:=true obstacle_min_area_ratio:=0.03 obstacle_near_y:=320 speed:=30

최신 track_tuning.yaml 또는 기존 track GUI 저장값의 공통 주행·BEV/ROI 설정을 읽습니다.
면적 모드의 GUI 저장값은 별도 ~/.config/autodrive/mission_bbox_size_tuning.yaml입니다.
재사용은 load_saved_tuning:=true이며, 새 실행 파일에 명시된 기본값은 실행 인자처럼 우선 적용됩니다.
주행 설정 보존/우선순위는 기존 mission_bbox_only.launch.py의 규칙을 따릅니다.
디버그 bbox에 area/near/path를 표시하고 /mission/status에 임계값과 전환 이유를 발행합니다.

## 오프라인 검증 (실제 차량을 실행하지 않음)

    python3 tools/tests/test_bbox_launch_assembly.py
    python3 -m pytest -q src/skku_track_drive_pkg/test tools/tests

실제 차량의 카메라 장착, 주행 속도, bbox 크기 임계값, 반대 차선 경로 품질은 별도 시험이 필요합니다.
