# 전·후방 카메라로 이어서 수행하는 미션 2

기존 `1125` 수직주차를 같은 프로그램에서 실행한다. 수직주차 제어가 `STOPPED`에 들어오면 기존 자동 출차 대신 아래 순서를 수행한다. 새 실측 파일·`mission2` 설정 블록·트랙 좌표·조향각 실측표는 사용하지 않는다.

| 상태 | 카메라로 확인하는 동작 |
|---|---|
| `PERP_HOLD` | 수직주차 직후 전·후방 영상의 변화가 작아지는 것을 확인하고 5초간 정차한다. |
| `EXIT_RIGHT` | 1125가 사용하는 우조향 부호 그대로 최대 우조향으로 전진한다. 기존 출차·후진 시간보다 오래 탐색하지 않으며, 전방 근접 후보나 영상 지연 시 0 명령을 낸다. |
| `BEFORE_REVERSE` | 후방에서 두 기준 차량과 그 사이의 주차선이 연속으로 확인되면 정차한다. 1·3 사이에 있든 2·4 사이에 있든 두 차량 사이의 칸을 고른다. 숫자 자체를 읽지는 않는다. |
| `REVERSE_IN` | 기존 후방 BEV와 수직주차의 조향 계수를 재사용하여 주차선 방향과 화면상 중심으로 후진 조향을 보정한다. 처음 잡은 칸을 짧게 추적하되 정지선 확인이 끊기면 후진하지 않는다. |
| `BEFORE_ADJUST` / `FORWARD_ADJUST` | 끝 경계에 접근했는데 각도가 어긋나면 일시 정지하고, 전방 근접 후보를 확인한 후 짧게 전진 보정한다. 최대 두 번이며 다시 정차 후 후진한다. |
| `PARALLEL_HOLD` | 화면의 각도·중심·정지선 조건을 만족하고 정차 상태가 5초 유지되면 0 명령 상태로 끝난다. |

카메라 입력은 1125 `perpendicular_camera.yaml`의 회전·반전·투시변환을 사용한다. 기준 차량은 1125 설치본의 `models/mission/best_m_s_curve.pt`에서 `obstacle`·`obstacles`·`car` 클래스 중 존재하는 것을 사용해 찾는다. 새 모델을 받거나 모델 경로를 입력하지 않는다. 흰색 모형 차량 검출 재현율, 양방향 시야, 실제 정차·충돌 여유는 실차에서 아직 확인하지 않았다. 영상의 픽셀 기준 근접 판정은 실제 범퍼 거리나 전 차체의 사각지대를 측정하지 않는다. `PARALLEL_DONE`은 코드의 영상 조건 완료이며, 규정집의 바퀴 접지면 판정과 `OUT` 선 도달까지 확인한 결과는 아니다. `OUT` 주행은 포함하지 않는다.

## 실행

`1125`에서 기존 수직주차를 돌렸던 것과 같은 빌드를 source한다. `seyeon` 코드는 1125 패키지를 가져와 같은 노드에서 두 단계를 잇는다. 별도의 수직주차 launch를 동시에 실행하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/autodrive_ws/Autonomous_Capstone-integrated/install/setup.bash"
ros2 launch "$HOME/autodrive_ws/mission2-seyeon/Autonomous_Capstone-integrated/seyeon/mission2_parking.launch.py" \
  config_file:="$HOME/.config/autodrive/perpendicular_camera.yaml" \
  front_camera_device:=/dev/video0 rear_camera_device:=/dev/video2 \
  preview:=true
```

기본은 모터 없는 미리보기다. 기존 창의 `W`로 수직주차부터 시작할 수 있고 자동 인계 시 `W`를 한 번 더 누르지 않는다. 단계와 정지 이유는 터미널과 `/seyeon/mission2/status`에서 본다. 실제 송신 경로는 `preview:=false`일 때만 열리며 1125 기존 수직주차의 `calibration_confirmed` 및 시리얼 주행 허가를 그대로 적용한다. 원본 수직주차 YAML은 읽기만 하고, `P`로 저장하면 세션 임시 사본에만 들어간다.
