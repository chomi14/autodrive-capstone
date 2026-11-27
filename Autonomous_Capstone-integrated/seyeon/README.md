# 수직주차에서 평행주차까지 이어지는 미션 2

한 번 실행해서 `1125`의 수직주차 제어를 그대로 사용한다. 기존 제어기가 `STOPPED`에 들어가면 이 폴더의 코드가 같은 명령 발행기를 이어서 사용한다. 원본 수직주차의 `ESCAPE` 출차는 이 실행 모드에서만 끈다. 주차 중 프로그램 교체나 추가 `W`는 없다. 첫 `W`만 기존 GUI에서 누른다.

| 상태 | 동작 |
|---|---|
| `PERP_HOLD` | 현재 차체가 수직 2번 또는 3번에 있는지 지도에서 확인하고 최소 5초간 정차한다. 위치가 움직이거나 관측이 끊기면 정차 시간을 다시 센다. |
| `EXIT_RIGHT` | 최대 우조향 전진을 내되 조향 궤적과 정지 여유가 측정한 통로 안에 있을 때만 가능하다. 후방 카메라에서 두 기준 차량을 찾아 칸을 선택한다. |
| `ENTRY_PAUSE` | 목표가 좌측 후방에 들어오면 정지한다. 같은 칸을 계속 확인하고 정지가 안정된 뒤 후진 경로를 계산한다. |
| `PARALLEL` | 좌조향 후진으로 시작해 차를 집어넣고 반대 조향으로 펴며, 필요하면 짧은 전·후진으로 위치를 맞춘다. 실제 위치와 장면에서 금지 영역을 재확인한다. |
| `PARALLEL_HOLD` | 선택한 칸 안에서 차체가 평행한 상태를 유지하고 최소 5초 정차한다. |
| `PARALLEL_DONE` | 평행주차까지만 완료한 상태다. 전체 경기 성공 조건인 OUT 선 도달은 이 코드의 다음 단계다. |

평행 배치는 규정집 PDF 5페이지 도면에서 평행칸 내부 길이 1.50 m, 안쪽 폭 0.85 m, 칸 피치 1.55 m를 사용한다. 폭 0.95 m는 선을 포함한 외곽 치수다. 수직 쪽은 내부 폭 0.95 m, 깊이 1.50 m, 칸 피치 1.00 m를 사용한다. 기준 차량이 평행 1·3에 보이면 2번, 2·4에 보이면 3번이다. 끝의 빈 칸이나 차량 한 대만 보이는 경우에는 목표를 확정하지 않는다. 1125의 후방 주차선 모델과 기준 차량 검출 모델은 역할이 다르다. 흰색 모형에서 두 기준 차량을 일관되게 검출할 수 있음을 별도로 확인해야 한다.

## 실행

`1125`의 워크스페이스를 평소처럼 빌드한 뒤 해당 `install/setup.bash`를 source한다. 이 폴더는 **로컬 Seyeon 소스 트리의 `Autonomous_Capstone-integrated/seyeon`**에 둔다. 두 워크스페이스를 겹쳐 source하지 않는다. 아래 경로와 카메라 장치는 실제 위치로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
source /path/to/1125/Autonomous_Capstone-integrated/install/setup.bash
ros2 launch /path/to/Seyeon/Autonomous_Capstone-integrated/seyeon/mission2_parking.launch.py \
  config_file:="$HOME/.config/autodrive/perpendicular_camera.yaml" \
  mission2_config:="$HOME/.config/autodrive/mission2_measured.json" \
  front_camera_device:=/dev/video0 rear_camera_device:=/dev/video2
```

기본은 미리보기이며 기존 `1125` 화면의 `W`를 사용한다. 상태는 터미널과 `/seyeon/mission2/status`에 표시된다. 실제 구동은 `preview:=false`, 두 실측 설정의 승인, 기존 시리얼 주행 허가가 모두 필요하다. 원본 수직주차 YAML은 읽기만 하며 이 실행에서 원본 수직주차 노드의 별도 launch 명령을 동시에 실행하지 않는다.

## 한 번 필요한 현장 설정

기본 키는 상위 `tools/parallel_camera_measured.template.json`과 `tools/parallel_camera_live.md`의 형식을 쓴다. `slot`과 `obstacles`는 이 실행에서 아래 값으로 생성된다. 나머지는 측정해 둔 값으로 한 번 구성한다. 주차칸의 절대 원점, 카메라의 실측 좌표 변환, 정지거리와 스티어링 보정은 규정집 치수에서 알 수 없다. 위치 오인식 방지를 위해 진입점에서 측정한 시작 위치와 실제 트랙에 분명히 대응하는 흰 모서리를 등록한다. 동일 모양이 반복돼 영상 위치를 구분할 수 없는 구간에서는 장치가 정지하도록 하고, 통과 가능 여부는 미리보기에서 확인한다.

`mission2` 키 아래에 다음을 넣는다.

- `parallel_bay1_clear_corner_m`, `perpendicular_bay1_clear_corner_m`. 각각 1번 칸 내부의 최소 `[x, y]` 모서리. 현장 `+x`는 최종 평행주차 차량의 전방이다.
- `parallel_index_direction`, `perpendicular_index_direction`. 각 줄에서 1→4번이 `+x`나 `-x`인지 각각 `1` 또는 `-1`로 지정한다.
- `fixed_obstacles`. 다른 고정 장애물 사각형 목록. 각 줄의 개별 주차칸은 프로그램에서 추가한다. 경기에서 놓인 차량 외곽이 칸 밖으로 나오면 해당 구역도 포함해야 한다.
- `exit_right_limit_s`, `target_confirm_frames`. 최대 우조향으로 나가는 시간이 현장 검증 범위를 넘지 않도록 하며, 서로 다른 후방 이미지에서 두 차량 위치가 반복 확인되어야 한다.
- `rear_detector`. `model_path`는 이미 설치된 `.pt`, `classes`는 기준 차량 검출에 쓸 실제 클래스 이름의 목록, `device`는 추론 장치, `confidence`는 확인한 모델의 신뢰도 기준, `bay_edge_margin_m`는 모호한 경계 검출을 제외할 여유다. 자동 다운로드하지 않는다. 카메라 평면에 투영한 박스 하단을 이용하므로 차량 전체가 보이는지와 바퀴 접지점 오차를 현장에서 확인한다.

`1125` 설치본은 GitHub `4af80cef7cb5e56b66dff1849238fac4003bcd9f` 기준이다. 평행 진입 전에는 2→2, 2→3, 3→2, 3→3 조합을 확인하고, 먼저 차를 움직이지 않는 미리보기에서 신호와 정지 동작을 확인한다.
