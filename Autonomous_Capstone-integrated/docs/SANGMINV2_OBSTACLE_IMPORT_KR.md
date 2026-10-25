# sangminv2 장애물 회피 반영

## 현재 동작 (2026-10-25 변경)

`mission_bbox_only.launch.py`, `mission_bbox_size.launch.py` 모두 현재 목표 차선의
관측 마스크와 장애물 bbox 하단이 겹치고 `obstacle_near_y` 이상이면 첫 프레임에
반대 차선으로 전환한다 (`lane2→lane1`, `lane1→lane2`). 판정은 원본 카메라 영상에서
수행하며 ROI/주행 경로 유효성이나 bbox 면적을 전환 조건으로 사용하지 않는다.
`obstacle_near_y`는 480px 영상 높이 기준이며 해상도에 비례해 적용한다.
차선 겹침은 bbox 하단 주변 7픽셀 높이의 실제 폭으로 확인하며 좌우 margin을 추가하지 않는다.

장애물 확인 대기·전환 프레임 정지·회피 경로 미검출 정지·반대 차선 확인 대기·
해제 확인 대기·회피 속도 제한을 제거했다. 새 차선 경로는 다음 영상 프레임에
계산하고 그동안 기존 주행 명령을 유지한다. 반대 차선이 보이지 않거나 막혀 있어도
전환한다. 전환 후 5초간 추가 자동 회피를 막아 목표 차선이 바로 되돌아가는 것을 방지한다.
쿨다운 동안 속도를 0으로 보내지 않으며 원래 주행을 유지한다. 이 시간은 프레임 수가
아닌 monotonic 시계의 실제 경과 시간으로 계산한다. 최초 회피에는 대기가 없다.
신호등 정지/감속, W/S, 통신/제어기 오류 처리는 별도 기능으로 유지한다.

미션 GUI(조절 창 포함) 또는 디버그 창에 포커스를 두고 `L`/`l`을 누르면
쿨다운 중에도 목표 차선을 `1↔2`로 즉시 반전하고, 그 시점부터 5초를 다시 유지한다.
이 동작은 출발 승인을 변경하거나 모터 명령을 직접 발행하지 않는다.
`/mission_controller_node/toggle_target_lane` (`std_srvs/srv/Trigger`) 서비스로 연결하며,
새 목표 차선과 남은 쿨다운을 GUI/디버그에 표시한다.
`obstacle_near_y` 슬라이더는 별도 `Mission Tuner Obstacle` 창에 표시한다.

과거 면적/확인 프레임/유지시간/쿨다운/회피 PWM 등의 파라미터는 기존 YAML과
실행 인자를 읽기 위해 남겨두지만 bbox 회피에 사용하지 않으며 GUI에서 숨긴다.
현재 쿨다운은 이전 `lane_change_cooldown_s` 설정과 무관하게 5초로 고정한다.
개발용 LiDAR 융합 모드는 기존 판단을 `development/mission_core.py`에 보존했다.

아래는 최초 이식 당시 기록이며, 현재 회피 동작은 위 설명을 따른다.

원본: [chomi14/autodrive-capstone, sangminv2](https://github.com/chomi14/autodrive-capstone/tree/sangminv2).
가져온 변경: [0ea1a82add42f50e766c170d798bd414c477123c](https://github.com/chomi14/autodrive-capstone/commit/0ea1a82add42f50e766c170d798bd414c477123c),
`Add bbox-area obstacle lane change on latest 1010 driving pipeline`.

전체 브랜치를 병합하지 않고 `git fetch origin sangminv2`로 받은 커밋의 장애물 회피 변경만 이식했다.
기본 트랙 주행 파일·모델·차선/BEV/Stanley 계산·주차·펌웨어·통신 코드는 이번 반영에서 수정하지 않았다.
기존 `modes/`·`development/` 구조와 실행 이름을 유지했다.

## 반영한 동작

- 새 `mission_bbox_size.launch.py`로 실행하면 bbox 면적 모드가 켜진다.
- 현재 경로 또는 현재 차선의 관측 마스크와 겹치는 장애물에 대해,
  영상 안에 보이는 bbox 면적이 전체 영상의 2% 이상이고 bbox 하단이 480px 기준 y=300 이상이면 회피를 요청한다.
- 현재 프레임에서 반대 차선 마스크가 보이고 그 차선과 겹치는 장애물이 없어야 차선을 전환한다.
  반대 차선 검사에는 현재 차선의 최소 면적 조건을 적용하지 않는다.
- 첫 조건 충족 프레임에 반대 차선을 요청하고, 전환 프레임에는 정지한다.
  다음 프레임의 새 차선 경로가 유효하면 기존 Stanley 계산과 회피 PWM 상한을 사용한다.
- 재전환 제한은 2초, 회피 PWM 상한은 80이다. 기본 주행 속도를 상한보다 높이지 않는다.
- `lane2→lane1`, `lane1→lane2`를 지원한다. `obstacle`, `obstacles` 클래스명을 모두 허용한다.
- 기존 신호등 빨강 정지·초록 재개·노랑 감속 판단은 유지했다.
- 기존 `mission_bbox_only.launch.py`는 기존 하단/연속 프레임 방식으로 실행된다.

설정은 [size_launch.py](../src/vehicle_bringup_pkg/vehicle_bringup_pkg/modes/bbox_mission/size_launch.py)의
`SIZE_DEFAULTS`에서 수정한다. 공통 bbox 설정은 같은 폴더의 `launch.py`에 있다.
현재 미션 모델 `models/mission/best_m.pt`를 사용하며 `model_path:=...`로 바꿀 수 있다.
면적 모드 GUI 저장 경로는 `~/.config/autodrive/mission_bbox_size_tuning.yaml`이다.
실행 인자가 저장된 설정이나 코드 기본값보다 우선한다.

## 실행

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash

# 카메라와 판단 GUI 확인
ros2 launch vehicle_bringup_pkg mission_bbox_size.launch.py \
  sensors_only:=true camera_device:=/dev/video2 device:=cuda:0 speed:=30

# 위 실행을 종료한 후 실제 주행 시 사용하는 명령
ros2 launch vehicle_bringup_pkg mission_bbox_size.launch.py \
  camera_device:=/dev/video2 arduino_port:=/dev/arduino \
  device:=cuda:0 speed:=30 auto_calibrate:=false
```

카메라/Arduino 포트는 실제 장치에 맞춘다. 기존 W 출발 승인과 정지 처리를 사용한다.
기존 트랙 명령 `track_drive_tuning.launch.py`는 그대로 사용한다.
새 면적 모드를 실행할 때는 위 새 런치 이름을 사용한다.

## 소스 위치와 검증 범위

- `src/skku_track_drive_pkg/skku_track_drive_pkg/modes/bbox_mission/mission_core.py`: 원본 면적/차선 전환 로직.
- 같은 폴더의 `mission_controller_node.py`: 장애물 클래스명 호환과 bbox 면적 디버그 표시.
- `mode_parameters.py`: 미션용 파라미터 세 개만 추가.
- `src/vehicle_bringup_pkg/vehicle_bringup_pkg/modes/bbox_mission/`: 기존 bbox 런치의 파라미터 연결과 새 면적 런치.
- `src/vehicle_bringup_pkg/launch/mission_bbox_size.launch.py`: 기존 구조에 맞춘 짧은 실행 연결 파일.
- 원본의 `test_bbox_size_avoidance.py`, `test_bbox_size_launch.py`도 함께 가져왔다.

검증 결과: 전체 Python 회귀 검사 248개 통과, 7개 패키지 빌드 성공.
트랙·기존 bbox·새 면적 bbox·수직주차·평행주차의 설치된 런치 인자 조회도 확인했다.
기존 소스 파일 279개가 동일하고, 원본 면적 회피 구현과 이식된 구현의 실행 코드는 동일함을 확인했다.
신호등 판단 블록도 변경 전후 동일하다.
실차의 임계값·회피 궤적은 이번 자동 검증에 포함되지 않는다.
