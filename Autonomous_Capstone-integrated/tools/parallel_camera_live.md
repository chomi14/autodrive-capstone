# 실시간 카메라 평행주차

ROS 2의 `parallel_camera_parking.launch.py`는 별도 모드다. 전·후방 영상에서 **이미 그려진 흰색 주차선의 코너**를 찾아 차체 위치와 방향을 갱신한다. 시작 기준 위치는 측정해 지정한다. 시야에 들어올 실제 코너를 카메라별로 등록한다. ArUco 표식, 저장 영상, 이미지 학습은 필요 없다.

첫 구간은 좌조향 후진이다. 경로를 따라가며 후진 진입을 하고, 차 뒤가 주차칸에 들어간 뒤 반대 조향과 짧은 전진·후진을 허용한다. 카메라 위치로 경로 진행도를 계산하고, 조향을 보정한다. 진행 방향을 바꾸기 전에 바퀴 PWM을 0으로 유지해 정지 상태를 확인한다. 마지막에는 주차칸 안에 차체 전체가 들어가고 방향 오차가 허용 범위 이내인지 검사한다.

**통제된 고정 트랙 배치 전용이다.** 앞차·뒤차 위치와 통행 경계는 실측한 사각형으로 입력한다. 코드가 실시간으로 추정하는 것은 우리 차의 위치다. 앞뒤 차량의 위치를 화면에서 재검출하는 기능은 없다. 배치가 바뀌거나 사람이 진입하면 멈추고 새로 확인한다. 사용 중 차량이나 사람을 관찰할 수 없는 환경에는 적용하지 않는다.

## 실행과 키

설정 없이 두 카메라를 바로 확인하려면 `python3 tools/parallel_camera_preview.py --inspect /dev/video0 /dev/video2`를 실행한다. 노란색은 영상에서 검출한 선분 후보이며 `Q`로 닫는다. 이 화면에는 거리 추정이나 구동 명령 출력이 없다.

ROS 2 Humble을 로드하고 워크스페이스에서 빌드·source한 뒤 실행한다. 장치 번호와 설정 경로는 예시다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch vehicle_bringup_pkg parallel_camera_parking.launch.py \
  config_file:="$HOME/.config/autodrive/parallel_camera_measured.json" \
  front_camera_device:=/dev/video0 rear_camera_device:=/dev/video2
```

기본은 시리얼을 생성하지 않는 미리보기다. 카메라 화면에서 정지 상태로 `P`를 눌러 경로를 생성하고 `W`로 시작한다. 미리보기에서는 차량을 수동으로 옮겨 영상에 따라 조향 명령이 바뀌는지 확인할 수 있다. 움직임 명령은 `/parallel_camera/preview/motion`으로 분리된다. `S`로 정지하고, 정지 뒤 `R`로 측정한 시작 위치를 다시 설정할 수 있다. 관측 상실, 경로 이탈, 카메라 지연, 구간 제한 시간 초과, 정지 여유 부족이 발생하면 현재 경로를 폐기하므로 새 `P`와 `W`가 필요하다.

실차 경로는 동일 명령에 `live:=true`를 명시했을 때만 연결된다. 그 전에 보정, 시리얼 조향 매핑, 정지 여유, 고정 배치, 미리보기 방향을 확인한 설정에 `commissioned: true`를 넣어야 한다. 실제 구동은 기존 시리얼 주행 허가와 `W`가 모두 필요하다. 시작 전 별도의 비상 정지 수단을 준비한다. 아래 값은 현장 실측 없이는 결정할 수 없다.

## JSON에 넣을 값

`parallel_camera_measured.template.json`을 복사해 측정값으로 채운다. 평행주차칸을 따라 차가 최종적으로 바라볼 방향이 현장 좌표 `+x`, 왼쪽이 `+y`다. 차량 좌표는 후륜 차축 중앙이 원점이다. 위치와 길이는 m, 각도는 도 단위다. GUI의 `MAP LONGITUDINAL GAPS`는 지도에서 같은 가로 범위에 들어온 장애물까지의 종방향 간격이며 `--`는 그 방향에 해당하는 표시값이 없다는 뜻이다. 실제 경로 검사는 양쪽 간격 숫자와 별개로 차체 전체를 사용한다.

| 필드 | 실측·설정 내용 |
|---|---|
| `start_pose_m_deg` | 진입 직전 후륜 차축 중앙의 현장 좌표 `[x, y, yaw]`. 이 모드는 해당 위치에서 주차칸이 차량 좌측 후방인 경우에 사용한다. |
| `vehicle` | `wheelbase`, `front_overhang`, `rear_overhang`, `width`, `steer_left_deg`, `steer_right_deg`. 좌·우 한계는 각각 양의 크기다. |
| `slot`, `bounds`, `obstacles` | 각각 주차칸, 통행 가능 전체 바닥 범위, 앞·뒤 차량을 포함한 고정 장애물 목록. 사각형 하나는 `[xmin, ymin, xmax, ymax]`. 차체 돌출부까지 장애물에 포함한다. |
| `margin_m`, `segment_m`, `heading_tolerance_deg` | 계획 차체 여유, 짧은 경로 구간 길이, 완료 방향 오차 한계. 여유는 최소한 `worst_stop_travel_m + max_path_error_m + pose_error_m` 이상으로 정한다. |
| `front`, `rear` | 각각 `image_size` `[width,height]`, `pixel_points` `[[u,v],...]`, `vehicle_points_m` `[[x,y],...]`, `corners_world_m` `[[x,y],...]`. 첫 두 점 목록은 같은 바닥 점을 이미지와 차량 좌표로 대응시킨다. 카메라당 네 개 이상의 서로 일직선이 아닌 점이 필요하며, 주차 단계별로 두 카메라 모두에 등록된 페인트 모서리가 보이고 합쳐서 서로 다른 코너가 세 개 이상 보여야 한다. |
| `tracking`의 영상 기준 | `pose_error_m`, `corner_window_m`, `max_pose_jump_m`, `max_yaw_jump_deg`, `max_frame_age_s`, `max_frame_skew_s`, `paint_s_max`, `paint_v_min`. 페인트 모서리는 선의 중심이 아니라 영상에서 검출할 실제 모서리 위치를 측정한다. 영상 신선도는 ROS 캡처 시각으로 확인하며 `max_frame_age_s`의 상한은 0.25초다. |
| `tracking`의 제어 기준 | `max_path_error_m`, `max_path_error_deg`, `worst_stop_travel_m`, `settle_s`, `settle_drift_m`, `segment_timeout_s`, `forward_pwm`, `reverse_pwm`, `steering_deg_by_step_minus7_to_plus7`. 마지막은 송신 조향 -7부터 +7까지의 실제 앞바퀴 각도 15개이며 좌회전 각도를 양수로 적는다. `worst_stop_travel_m`은 영상·제어·송신 지연과 실제 정지 시까지 움직인 거리까지 포함해 더 불리한 전·후진 상황에서 잰 값이다. |
| `commissioned` | 최초에는 `false`. |

먼저 차량을 움직이지 않고 시작 위치, 좌후진 진입, 반대 조향, 마무리 정렬 위치에서 위치와 각도를 독립 측정값과 대조한다. 측정 코너를 가리거나 카메라 한 대를 끊는 상황에서 미리보기 명령이 0이 되는지 확인한다. 보정을 바꾸거나 주차칸 또는 앞뒤 차량이 이동하면 해당 설정을 다시 확인한다.
