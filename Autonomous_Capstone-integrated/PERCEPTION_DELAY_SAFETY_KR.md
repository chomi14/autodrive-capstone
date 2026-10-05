# 카메라·추론 중단 시 명령 유지와 정지 (2026-10-05)

작업 디렉터리: `/home/autolab/autodrive_ws/Autonomous_Capstone-integrated`.
기존 **20Hz 시리얼 송신 / 10Hz 생존 callback**을 유지했다.
실제 카메라 읽기와 CUDA 추론은 센서 전용 구성에서 수행했다.
모터 구동·차량 시리얼 연결·펌웨어 업로드는 수행하지 않았다.

## 측정 환경과 분포

연결된 장치는 `/dev/video2`, SunplusIT HD Webcam **노트북 내장 카메라**이다.
실차 전방 `/dev/video0`는 없었다. 640×480 MJPG, 요청 30fps, 실제 약 14.8fps,
RTX 4050 Laptop GPU / CUDA 0, 실제 트랙 YOLOv8s-seg 및 미션 YOLOv8n-seg를 사용했다.
사용자 저장 track/mission 튜닝 파일을 로딩했고 변경하거나 P로 덮어쓰지 않았다.
각 60초씩 프레임 수신·성공한 제어결과 완료를 측정하고, 수신 분리 후 각 60초 재측정했다.
첫 성공 결과 이후 5초 워밍업은 정상 분포에서 제외하고 원시 기록에는 보존했다.

아래 단위는 ms이다. 완료는 **YOLO + 차선/경로 + 미션/제어 계산 + 명령 발행 성공**이다.
추론 단독 소요 시간 및 순수 YOLO 완료 시각도 별도로 기록한다.

| 모드/측정 | 프레임/결과 수 | 프레임 median / p95 / p99 / max | 유효결과 완료 median / p95 / p99 / max |
|---|---:|---|---|
| 트랙 변경 전 60s | 887 / 887 | 67.854 / 81.383 / 86.482 / 104.904 | 68.139 / 89.699 / 98.464 / 133.405 |
| 트랙 수신 분리 후 60s | 890 / 889 | 67.457 / 81.851 / 86.109 / 94.698 | 68.304 / 89.787 / 101.364 / 115.443 |
| 미션 변경 전 60s | 888 / 888 | 67.854 / 81.398 / 86.339 / 93.483 | 68.724 / 91.637 / 105.453 / 115.677 |
| 미션 수신 분리 후 60s | 887 / 888 | 67.825 / 82.667 / 87.773 / 95.663 | 68.540 / 92.078 / 101.902 / 140.114 |

수신 분리 후 추론 단독 소요 시간 median/p99/max는 트랙 16.811/36.032/55.618ms,
미션 16.189/36.978/66.860ms였다. 프레임을 제어 노드가 받은 뒤 유효결과 완료까지의
시간 max는 각각 63.415/73.022ms였다. 입력/완료 시각이 다른 관찰창 경계에서는
프레임 수와 결과 수가 하나 다를 수 있다.
순수 YOLO 종료 시각도 추가로 각 30초 측정했다. 아래는 YOLO 처리 **완료 사이 간격**이며,
한 번의 추론에 걸린 시간과는 구분한다. 단위는 ms이다.

| 추가 측정 | 프레임 / 결과 수 | 프레임 median / p95 / p99 / max | 순수 YOLO 완료 간격 median / p95 / p99 / max |
|---|---:|---|---|
| 트랙 30s | 443 / 443 | 67.908 / 82.458 / 87.854 / 90.957 | 68.512 / 92.253 / 99.828 / 122.603 |
| 미션 30s | 445 / 446 | 67.597 / 82.313 / 87.487 / 95.467 | 68.424 / 89.387 / 95.461 / 110.869 |

추가 측정은 `yolo_epoch_track`, `yolo_epoch_mission`에 보존했다. 순수 YOLO 종료 시각은
진단 기록이며, 주행의 진행 인정은 실제 제어 명령까지 성공한 결과로 유지한다.

지속 기록은 `reports/perception_delay_2026-10-04/`에 있다. summary.json은 위 정상
구간의 분포이며 events.jsonl에는 워밍업을 포함한 전체 시간 기록이 있다. 이미지는 저장하지 않았다.
**실차 카메라/트랙 장면·실제 GUI·열/배터리/CPU 경쟁 부하의 최악값을 측정한 것은 아니다.**

## 기존 6초 정지 기준 유지

2026-10-05 요청에 따라 단계 감속을 제거했다. 기존에 최종 X/disarm을 결정하던
**유효결과 나이 6.0초**는 그대로 유지한다. 새로운 timeout이나 별도 timer는 추가하지 않았다.
앞선 측정에 근거해 사용하던 최종 정지 시점을 유지한 것이며, 위 내장 카메라 측정이
실차의 제동거리나 허용 주행거리를 검증했다는 뜻은 아니다.

기준 파일은 `src/vehicle_bringup_pkg/config/perception_delay_policy.yaml`이다.
트랙·미션 profile에는 `perception_stop_s: 6.0`만 사용한다. 기존 감속용
`perception_hold_s` / `perception_ramp_s` launch 인자와 sender 파라미터는 제거했다.
기존 최종 정지 나이를 한 이름으로 표현했으며 정지 기준값은 바꾸지 않았다.
설정 스키마는 2이다. 이전 스키마는 잘못된 감속 설정이 묵시적으로 적용되지 않도록 거부한다.
기존 사용자 차선/Stanley/속도 YAML과 분리하며 GUI P 저장은 이 파일을 변경하지 않는다.
`perception_stop_s`는 read-only ROS 파라미터이며 launch에서 설정하고 재실행한다.

## 현재 동작

카메라 수신 callback과 추론 callback은 분리되어 있다. 추론에는 최신 프레임 하나만
전달하고, 10Hz 생존 신호에는 카메라/성공 결과 횟수와 나이 및 worker 상태를 담는다.
반복 생존 신호나 새 입력만으로 성공 결과의 유효 시각을 갱신하지 않는다.

sender는 새 성공 sequence에서만 결과의 유효 시각을 갱신한다. 판단 나이는 마지막
완료 이후 시간과 **그 결과에 사용한 프레임의 수신 이후 시간 중 더 큰 값**이다.
오래된 프레임의 늦은 완료가 기존 정지 시점을 연장하지 않는다. 수신 시각은
제어 노드의 monotonic clock이며 실제 촬영/노출 시각까지 보장하지 않는다.

| 유효결과 나이 | 주행 명령 |
|---|---|
| <6.0s | 마지막 유효 조향·왼쪽/오른쪽 PWM을 그대로 20Hz 송신 |
| ≥6.0s | 같은 조향의 속도 0 프레임 후 X/disarm, 새 W 요구 |

지연 도중 PWM을 곱하거나 낮추는 로직, 감속 진입 조향 anchor, 감속 비율/단계는 없다.
미션 판단으로 이미 정지했다면 마지막 속도 0을 반복한다. 새로운 미션 속도 0 명령도
즉시 적용하며, 과거의 주행 PWM을 복원하지 않는다. 정상 결과가 계속 들어오는 신호등/
장애물 정지는 기존 MissionCore의 해제 조건으로 재개한다.

6초 전에 새 최근 프레임의 성공 결과가 들어오면 정상 처리가 이어진다. 최종 정지
기준은 새 메시지를 수용하기 전에 검사하므로 늦은 결과가 정지를 취소하지 못한다.
정지 뒤에는 새 성공 sequence와 유효 명령·생존·UI·READY 및 새 W가 필요하다.

**S와 Ctrl+C는 6초를 기다리지 않고 기존 X 정지 경로를 사용한다.**
제어기 Ctrl+C는 종료 생존 신호를 먼저 보낸다. SIGKILL/SIGSTOP은 기존 생존 만료로
정지한다. 기존 command/UI 생존 timeout과 firmware 500ms watchdog 값은 그대로다.
20Hz sender / 10Hz 생존 callback도 그대로다. 미션 `image_timeout_s=0.75`는 지연
진단 표시이며 별도 정지를 발행하지 않는다. 이미지 변환/인지 예외의 기존 zero 발행은 유지한다.

트랙/미션에만 결과 중단 조건을 적용한다. 주차와 수동 보정의 기존 LiDAR/geometry/
제어 조건은 변경하지 않았다. GUI는 실제 송신 요청 PWM/조향, ARM, 결과 나이와 정지
이유를 표시하며 감속 단계/비율은 표시하지 않는다. 이 표시값은 물리 PWM 계측이 아니다.

## 모의 검증

실제 ROS callback/executor와 production sender를 사용했다. 카메라는 합성 입력,
YOLO는 기다리는 detector, 시리얼은 메모리 포트로 대체한다(ROS_DOMAIN_ID=146).
모터 구동·차량 시리얼 연결·펌웨어 업로드는 수행하지 않는다.

| 시험 | 트랙 | 미션 |
|---|---|---|
| worker만 중단, 카메라/생존 정상 | PWM 100 유지→0→X, 약 5.937s | PWM 100 유지→0→X, 약 5.965s |
| 카메라 입력 중단 | PWM 100 유지→0→X, 약 6.001s | PWM 100 유지→0→X, 약 5.992s |
| 단일 추론 2초 지연 / 카메라 1초 공백 | 감속/정지 없음, 송신 최대 간격 52.2ms | 감속/정지 없음, 51.3ms |
| worker 중단 중 controller Ctrl+C | X 약 58.6ms | X 약 52.4ms |
| controller SIGKILL / SIGSTOP | X 약 701 / 702ms | X 약 703 / 704ms |
| 미션 의도적 정지 후 worker 중단 | 해당 없음 | 모든 motion PWM 0 유지 |

시간은 고장 주입 이후 관찰값이다. 기존 6초 나이 기준은 마지막 결과가 사용한 프레임
시각에서 계산하므로 주입 이후 정지 시간과 수십 ms 차이가 있다. OS/DDS/timer 지연을
포함한 관찰값이며 보장 시간이 아니다.

두 모드 모두 중간 PWM 감소 명령이 없고, 카메라/worker 중단 원인을 구분했다.
S는 지연 중 즉시 X를 쓰고, 늦은 결과가 와도 disarm을 유지했다. 중단 복구만으로는
재출발하지 않고 새 W가 있어야 출발했다. 미션의 정상 속도 0은 계속 유지했다.
회귀 검사 **125개**, **7개 패키지 빌드** 통과. 기존 SciPy/NumPy 경고 1개는 유지됐다.
5개 GUI의 실제 ROS 파라미터 서비스/임시 P 저장 검사도 통과했다. 트랙·미션은
4.5초 지연 상태를 입력하여 PWM 100/100과 결과 나이를 표시하고 감속 단계/비율을
표시하지 않는지 실제 그리기 callback으로 검사했다.

최신 모의 기록은 `reports/perception_hold_2026-10-05/`에 보존한다. 앞선 측정/감속
시험은 `reports/perception_delay_2026-10-04/`의 과거 기록으로 보존하며 현재 동작은
위 명령 유지 정책이다. 세부 측정값은 최신 `simulation_results.json` / `termination_results.json`을 참고한다.

## 실행과 재측정

```bash
cd ~/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash
```

기존 네 launch 명령을 유지한다. 트랙·미션의 6초 기준은 자동 연결되며 새 필수 인자는 없다.
실제 센서/GUI만 확인할 때 sender와 gate를 제외한다.

```bash
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py sensors_only:=true camera_device:=/dev/video2
ros2 launch vehicle_bringup_pkg mission_drive_tuning.launch.py sensors_only:=true camera_device:=/dev/video2
ros2 launch vehicle_bringup_pkg perpendicular_parking.launch.py sensors_only:=true
ros2 launch vehicle_bringup_pkg parallel_parking.launch.py sensors_only:=true
```

위 /dev/video2는 이전 노트북 측정용이다. 실차에서는 확인한 실제 전방 카메라 경로를 넣는다.
각 측정 명령은 자체적으로 센서 전용 launch를 시작하므로 기존 launch와 동시에 실행하지 않는다.

```bash
ROS_DOMAIN_ID=145 python3 tools/measure_perception_intervals.py --mode track --camera-device /dev/video0 --device cuda:0 --duration 60 --output /tmp/vehicle_timing_track
ROS_DOMAIN_ID=145 python3 tools/measure_perception_intervals.py --mode mission --camera-device /dev/video0 --device cuda:0 --duration 60 --output /tmp/vehicle_timing_mission
ROS_DOMAIN_ID=146 python3 tools/validate_perception_delay.py
ROS_DOMAIN_ID=144 python3 tools/validate_inference_liveness.py
python3 -m pytest -q tools/tests/test_perception_delay_guard.py tools/tests/test_controller_liveness.py tools/tests/test_drive_safety.py tools/tests/test_canonical_safety.py tools/tests/test_tuning_revision.py tools/tests/test_four_mode_launches.py src/skku_track_drive_pkg/test/test_mission_parking.py src/skku_track_drive_pkg/test/test_lane_processing.py
```

## 남는 실차 확인

- 기존 PWM을 유지하므로 인지 없이 최대 6초간 이동할 수 있다. 실제 이동거리/제동거리,
  조향 유지와 X의 물리 PWM 0은 실차 시험에서 확인해야 한다.
- 내장 카메라 측정은 실차 전방 카메라, 트랙 장면, 열/경쟁 부하의 최악값을 보장하지 않는다.
- 같은 정지 영상이 계속 새 메시지와 성공 결과로 발행되는 경우는 감지하지 않는다.
- GIL/OS/DDS/USB 중단으로 생존·시리얼도 막히면 기존 생존/UI/firmware 정지가 우선한다.
  멈춘 worker의 Python cleanup에는 강제 종료가 필요할 수 있지만 sender의 X 정지는
  worker가 완료될 때까지 기다리지 않는다.
