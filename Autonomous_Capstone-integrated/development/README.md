# 개발·실험·과거 코드

현재 주행 시작점과 소스 위치는 [루트 README](../README.md)에 정리했다.

## 과거 파이프라인

`legacy/`에는 기존 폴더 내용을 그대로 옮겼다. 이 폴더의 상위 `COLCON_IGNORE`로
루트에서 빌드할 때 과거 패키지를 자동 검색하지 않는다. 새 주행에 필요한 7개 ROS 패키지는 `src/`에 남아 있다.

| 이전 경로 | 현재 경로 | 용도 |
| --- | --- | --- |
| `src/camera_perception_pkg` | [legacy/camera_perception_pkg](legacy/camera_perception_pkg/) | 과거 분리형 영상·YOLO·신호등 파이프라인 |
| `src/decision_making_pkg` | [legacy/decision_making_pkg](legacy/decision_making_pkg/) | 과거 판단·경로·미션 노드 |
| `src/debug_pkg` | [legacy/debug_pkg](legacy/debug_pkg/) | 과거 시각화 노드 |
| `src/launch_pkg` | [legacy/launch_pkg](legacy/launch_pkg/) | 과거 main/mission/parking 런치 |
| `src/serial_communication_pkg` | [legacy/serial_communication_pkg](legacy/serial_communication_pkg/) | 과거 통신·검증 코드 |
| `src/data_collection` | [legacy/data_collection](legacy/data_collection/) | 과거 수집 스크립트 |

과거 `ros2 launch launch_pkg ...`는 현재 기본 빌드의 실행 경로에서 제외했다.
기존 `build/`·`install/`에 과거 패키지의 결과물이 남아 있을 수 있으므로, 현재 실행 이름은 루트 README의 표를 따른다.
`lidar_perception_pkg`는 현재 주차 파라미터가 `parking_timing.py`를 불러오므로 `src/`에 유지했다.

## 현재 패키지 안의 보조 코드

ROS 설치와 기존 실행 이름을 유지하기 위해 아래 코드는 각 Python 패키지 안의 `development/`로 분리했다.
원래 모듈과 런치 경로에는 짧은 연결 파일이 있다.

- [vehicle_bringup_pkg/development](../src/vehicle_bringup_pkg/vehicle_bringup_pkg/development/):
  카메라 수집, 수동 수집, 센서 점검, 주차/LiDAR 보정, LiDAR·카메라 융합 미션,
  비교용 기존 `mission_drive_tuning` 런치.
- [skku_track_drive_pkg/development](../src/skku_track_drive_pkg/skku_track_drive_pkg/development/):
  주차 보정 제어기/GUI, LiDAR 보정 GUI, LiDAR·카메라 융합 제어기·판단·설정.
- [tools/](../tools/): 기존 분석·측정·검증 스크립트 및 [회귀 테스트](../tools/tests/).
- [reports/](../reports/), [docs/](../docs/): 과거 검증 결과와 기술 문서.
- `datasets/`: 학습·평가 자료. 모델과 데이터는 이번 코드 분리에서 이동하지 않았다.

트랙 관련 파일 유지 조건에 따라 `track_video_replay.launch.py`, `tuning_replay.launch.py`,
`tuning_bag_replay_node.py`, `tuning_recorder_node.py`, `tuning_analysis.py` 등 기존 트랙 재생·분석 연결도 유지했다.
실차 센서/통신, 캡처, 펌웨어도 원래 위치를 유지했다.

## 하드웨어 없이 검증

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/autodrive-tests python3 -m pytest -q \
  src/skku_track_drive_pkg/test tools/tests
```

실행 인자만 확인하려면 `ros2 launch vehicle_bringup_pkg <런치 이름> --show-args`를 사용한다.
