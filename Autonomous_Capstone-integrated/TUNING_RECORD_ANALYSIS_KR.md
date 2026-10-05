# 실차 튜닝 비교·기록·분석 (2026-10-05)

작업 디렉터리: `/home/autolab/autodrive_ws/Autonomous_Capstone-integrated`.
모터 구동, 실제 차량 센서/시리얼 연결, 펌웨어 업로드는 수행하지 않았다.
모델·사용자 주행 설정·주차 실측값은 자동 교체하지 않았다.
20Hz 송신, 10Hz 생존, S/Ctrl+C X 정지, 생존/통신 만료 정지, 새 W 요구,
감속 없이 기존 6초 인지 중단 기준에서 X/disarm하는 정책을 유지했다.

## 기존 기능 조사와 재사용

| 기존 기능 | 확인 결과 / 재사용 |
|---|---|
| `video_replay_node`, `track_video_replay.launch.py` | 영상 디코딩/크기/FPS/EOF 처리를 재사용. 기존 트랙 replay의 명령과 키/생존 토픽도 `/dry_run`으로 격리 |
| `manual_drive_capture_node`, `manual_capture`, `bench_capture` | JPEG + 촬영 당시 명령 CSV가 있으나 원본 header 시각/경로/미션 근거/LiDAR가 없음. 기존 도구 유지, 새 수동 명령 생산자는 만들지 않음 |
| 기존 controller `perception_timing`, profile | 성공 프레임의 시각/YOLO/제어 완료 시간을 확장. 계산 로직은 그대로 사용 |
| MissionCore status / debug | 목표 차선, 차단/대체 마스크, 색상/카운터가 이미 있었음. 판단 박스, 색상별 비율, 이유, 현재 경로를 추가 |
| 주차 `render`, calibration controller/tuner | 기존 화면과 W/S 보정 도구 확장. 새 주차 제어기/수동 모터 도구를 복제하지 않음 |
| ROSbag2 | 원본 Image/LaserScan/상태를 저장하는 기존 ROS 도구를 사용. 파일 쓰기는 별도 recorder 프로세스 |

기존 manual_capture의 S는 감속/후진이다. 이번 측정 workflow의 W 시작/S 정지는
`parking_calibration.launch.py`의 기존 공통 gate를 사용한다.

## 실제 모델과 설정

| 모델 | 실제 클래스 ID 순서 | task | SHA-256 |
|---|---|---|---|
| `src/skku_track_drive_pkg/models/best.pt` | 0 lane2, 1 traffic_light | segment | `27d7fedda0946ded2f03f9c7ff47023b3a1f859b08574ee1c323baee9ee25e03` |
| `src/skku_track_drive_pkg/models/mission/best.pt` | 0 lane1, 1 lane2, 2 obstacle, 3 traffic_light | segment | `377b6a225e05ce3da6209b2dfdf0e129df034e8837d12861511e54267d258f15` |
| 통합본 루트 `best.pt` | 0 lane1, 1 lane2, 2 obstacle, 3 traffic_light | segment | `377b6a225e05ce3da6209b2dfdf0e129df034e8837d12861511e54267d258f15` |

미션 모델과 루트 모델은 전체 파일 해시가 같아 동일 가중치 파일이다.

**미션 정상 주행 speed는 추가 요청에 따라 250으로 반영했다.**
패키지 `config/mission_tuning.yaml`과 사용자 `~/.config/autodrive/mission_tuning.yaml`의
정상 speed만 80→250으로 변경했다. speed 인자를 생략한 launch에서 실제 ROS 로딩값과
GUI 초기값/슬라이더 250을 확인했고, 유효 차선의 모의 검출 입력에서 좌우 PWM 250을 발행했다.
회피 80/노랑 60/신호등·장애물 정지 0과 조향·모델·정지 정책은 유지했다.
아래 기존 기록·부하 측정 당시의 미션 speed는 80이었다. 해당 기록의 설정 snapshot은
수정하지 않았으며, 과거 bag 재생에 그 snapshot을 지정하면 당시 speed 80을 재현한다.
트랙의 초기 speed 250도 변경하지 않았다.
운영자가 명시적으로 변경하려면 기존 launch `speed` 인자 또는 GUI speed 후 P 저장을 사용한다.

## 같은 영상에 대한 모델 비교

전체 홈/워크스페이스에서 사용 가능한 영상 파일을 조사했다. 예제
`h-mobility-class/.../Collected_Datasets/driving_simulation.mp4`가 있다.
`ros2_ws`의 같은 이름 파일도 해시가 같았다. 현재 차량/현재 카메라로 촬영됐다는
근거가 있는 별도 실차 주행 영상은 확인하지 못했다. 영상은 실내 트랙 장면이며
촬영 차량·설치 자세·주행 방식과 정답 라벨은 미확인이다. 내장 카메라/합성 영상으로
모델의 실차 성능을 평가하지 않았고, 아래 수치는 예제 영상의 **모델 간 결과 차이**다.

영상: 640×480, 30fps, 2949프레임(98.3초).
SHA-256: `cec954a516545fb21bfc2d6f010d258137e7cb7314744bfbaba831d8e654ecf1`.
실제 두 가중치를 CUDA 0에서 사용하고, 동일 프레임 전체를 같은 순서로 넣었다.
현재 트랙의 공통 저장 설정을 두 계산에 넣고 두 모델 모두 lane2 → 기존 BEV/ROI →
PathPlanner → Stanley를 실행했다. 비교용 미션 모델에서는 미션 개입을 제외했다.
이 설정은 비교 프로세스 내부에서만 사용했으며 운영 설정을 저장/교체하지 않았다.

| 항목 | 중앙값 | p95 | 최대 |
|---|---:|---:|---:|
| lane2 마스크 IoU (두 모델 일치도, 정확도 아님) | 0.8860 | 0.9115 | 0.9258 |
| 조향 차이 절댓값 (step) | 0 | 2 | 5 |
| 횡오차 차이 절댓값 (BEV ROI px) | 5.267 | 23.063 | 57.997 |
| 방향 오차 차이 절댓값 (deg) | 1.110 | 9.293 | 30.563 |
| 경로 x 차이의 평균 (공통 y 범위, px) | 6.173 | 14.359 | 52.323 |
| 경로 x 차이의 최대 (공통 y 범위, px) | 10 | 28 | 82 |

두 모델 모두 이 영상의 2949개 프레임에서 현재 마스크/경로/오차 reference가 유효했다.
트랙/미션 모델의 제어 완료 시간 중앙값은 각각 15.456/11.989ms, p95는
16.218/12.756ms였다. YOLO 단독 중앙값은 10.816/7.358ms였다.
영상/기준점/정답이 검증되지 않아 어느 모델이 실제 주행에서 더 정확하거나 안전하다고
결론내리지 않는다. 미션 정상 상태라도 모델 가중치가 다르면 경로와 조향이 달라질 수 있다.

결과: `reports/tuning_analysis_2026-10-05/model_comparison/`.
`summary.json`, `comparison.csv`, 전체 `frames.jsonl.gz`, 대표 입력/마스크 PNG,
`control_comparison.png`를 보존했다. CTE는 m가 아닌 픽셀이고 경로 y는 BEV의 ROI 내부 좌표다.

## 미션 판단 화면과 로그

기존 상태와 카운터에 아래 읽기 전용 근거를 추가했다.

- debug 영상의 자홍색 선: 현재 계산에 사용한 카메라 투영 경로.
  주황/빨간 장애물 박스: 전체 장애물 / 현재 경로 차단 장애물.
- `path_lane`: 현재 경로를 계산한 차선, `target_lane`: 다음 처리 목표 차선.
  전환 요청 프레임에는 둘이 다를 수 있다. 현재 경로 유효/현재 마스크 관측 여부도 구분한다.
- 대체 차선 번호, 마스크 면적/최소 면적 검사, 가시성/장애물 중첩/유효 여부.
  이 유효성은 기존 **마스크 면적+장애물 중첩** 검사이며 대체 BEV 경로를 미리 계획했다는 뜻이 아니다.
- `reason`, `reasons`, 장애물 확인 횟수/기준, 차선 전환 요청 여부.
- 신호등의 eligible/제외 박스와 제외 근거, R/G/Y HSV 비율, 박스별 색상,
  최종 관찰 색에 기여한 `used_for_color`, 선택 색 비율과 후보 최대 비율,
  빨강/초록 확인 횟수·기준, red latch와 최종 미션 상태.

비율의 분모는 각 신호등 bbox 전체 픽셀이다. S/V 필터와 HSV H 범위를 통과한
빨강/초록/노랑 픽셀 비율이며 RGB 채널의 평균값이 아니다. 기존 Red→Yellow→Green 우선순위와
Unknown이 빨강 latch를 해제하지 않는 규칙을 유지했다. GUI의 기존 HSV/색상 비율/박스
임계값 slider와 P 저장은 그대로이며 별도 감속 단계나 timeout을 추가하지 않았다.

모의 검증: 경로 밖 장애물, 경로 위 장애물, 대체 차선 없음/막힘,
Red→Unknown→Green 모두 production MissionController/PathPlanner/Stanley와 합성 검출로 통과했다.
21프레임 기록과 debug PNG를 `mission_scenarios/`에 저장했다.
전환 요청 프레임은 기존 정지 규칙으로 PWM 0이었다. 다음 lane1 계산 프레임에서
조향 0→-7, 경로 x 평균 변화 339.195px/최대 367px,
CTE 변화 286.453px, 방향 변화 -29.687deg가 관찰됐다.
**합성 차선 배치의 값**이며 실제 차량 전환량이 아니다. 이 큰 변화는 기록·표시했고
완화하려고 회피 알고리즘이나 조향 계산을 임의로 수정하지 않았다.

## 기록 형식과 시간 연결

네 launch와 수동 보정에 `record_dir`를 추가했다. 빈 문자열이면 기록하지 않는다.
새 세션 폴더를 지정하면 read-only 분석을 켜고 sidecar recorder 하나를 추가한다.
주행 controller에서 디스크에 쓰지 않는다. `analysis:=true`만 지정하면 분석 메시지만 발행한다.

| 저장 파일 | 내용 |
|---|---|
| `input_bag/` | 원본 Image/LaserScan, 인지 timing/근거, controller 명령, 미션/주차/보정 상태, serial 명령 상태, drive state/키/생존 및 parameter events |
| `control.csv` | 프레임 시각, 처리/추론 시간, 목표/현재 차선, 경로 유효, CTE, 방향 오차, 조향/좌우 PWM, 미션 상태/이유, 전환 flag와 전체 details JSON |
| `events.jsonl` | 제어결과·상태·serial 명령을 ROS receipt ns와 monotonic receipt s로 기록 |
| `loaded_parameters.yaml` | 시작 시 실제 ROS 파라미터 snapshot |
| `replay_tuning.yaml` | 허용 튜닝 항목만 추출한 재생용 snapshot. 운영 설정에는 쓰지 않음 |
| `manifest.json` | 모델/설정 경로·해시, 초기 속도, 토픽, 시간 기준과 기록 결과 |

`control.csv.frame_stamp_ns`가 원본 Image/LaserScan.header.stamp와 정확히 연결된다.
`received_ros_ns`는 rosbag receipt clock과 같은 ROS 시간 기준이다. 처리 시간은
monotonic duration이다. 센서 header는 ROS 발행 시각이며 하드웨어 노출/동기화 시각을 보장하지 않는다. 영상 재생에서는 `/video_replay_node/frame_source`에 원본 영상
프레임 index/영상 시간과 해당 Image stamp를 연결하여 bag에 저장한다.

CSV 조향/PWM은 **controller 요청값**이다. 게이트/6초 정지 이후 sender가 실제로 쓴
소프트웨어 명령은 `events.jsonl`의 `serial_command`와 bag의 command_status로 따로 확인한다.
물리 모터 PWM이나 이동 속도를 계측했다는 뜻은 아니다. 주차는 같은 Scan을 여러 20Hz tick에서
사용할 수 있다. 주차/수동 보정의 차선·CTE·방향 오차는 해당 없음이라 빈 값이며, 보정의
프레임 시각도 입력 센서가 없으므로 빈 값이다. 보정 PWM 설정과 실제 발행 PWM 0을 구분한다.

Ctrl+C 종료 뒤 bag metadata와 manifest를 확인한다. 기록기 오류는 controller 정지 조건을
추가하지 않는다. rosbag 비정상 종료/강제 종료 여부는 manifest에 남는다. 초기 discovery나
디스크/DDS 포화로 입력/로그 누락이 생길 수 있으므로 frame stamp join 수를 확인해야 한다.
기록을 켠 오프라인 영상/bag 재생은 controller와 rosbag2_recorder의 입력 구독이 모두
준비된 뒤 시작한다. 재생 시작 직후 계산 CSV만 있고 원본 bag 입력이 없는 경우를 방지한다.
bag 시간 재생의 입력과 격리된 상태 토픽에는 Reliable QoS를 사용해 재기록 시 전송 누락을
줄인다. 실센서 QoS와 controller의 최신 프레임 처리 정책은 변경하지 않았다.
실제 센서는 이 재생 시작 대기를 적용하지 않으므로 시작 시점의 discovery 누락 가능성은 별도다.

## 기록 부하와 재생 검증

같은 예제 영상 앞 100프레임을 20fps로, 실제 CUDA/ROS pipeline에서 기록 OFF/ON으로
실행했다. 첫 성공 결과 10개는 워밍업으로 제외했다. 단위는 ms이다.

| 모드 | 기록 OFF 제어 완료 median / p95 | 기록 ON median / p95 | 추가 evidence 작성 median / p95 |
|---|---|---|---|
| 트랙 | 23.231 / 32.614 | 21.958 / 33.528 | 0.113 / 0.165 |
| 미션 | 22.217 / 33.920 | 22.528 / 39.840 | 0.143 / 0.180 |

제어 완료 시간은 YOLO→경로→명령 발행까지이고, 추가 evidence 작성은 그 뒤의 작업이다.
파일 쓰기는 별도 프로세스이나 CPU/디스크/DDS 경쟁은 가능하다. 단일 비교에서 트랙 중앙값이
작아진 것을 기록이 성능을 개선한 것으로 해석하지 않는다. CUDA/스케줄링 변동이 포함되며
실차 센서/GUI/저장장치의 최악 부하는 미검증이다.

트랙 ON: bag 100프레임 / CSV 98개, 미션 ON: bag 100 / CSV 99개.
CSV의 모든 제어결과 stamp가 원본 bag 입력에 연결됐다. 최신 입력 하나만 처리하는 기존
정책 때문에 입력과 처리 수가 다를 수 있다.

일반 시간 재생의 동일 stamp 비교는 트랙 96개 모두 동일, 미션 96개 중 3개에 차이가 있었다.
미션의 한 프레임은 Red 확인 횟수 3/2 차이로 RED_STOP/정상 주행이 달랐다.
프레임 누락과 EMA/확인 카운터 이력이 달라질 수 있어 시간 재생이 항상 동일 결과라는
보장은 하지 않는다. 두 방식 모두 serial/gate는 없고 명령은 `/dry_run`으로 격리된다.

`processed_csv` 비교용 재생은 원래 CSV에 기록된 입력 stamp만 순서대로 보내고 해당
controller의 완료 응답 뒤 다음 프레임을 보낸다. 재생 기간은 기록 FPS/계산 지연에 따라
달라질 수 있고, 미션의 wall-time 유지 조건까지 모든 환경에서 보장하지는 않는다.
이번 입력에서 트랙 98/98, 미션 99/99의 경로/오차/조향/PWM/상태/목표 차선이 모두 동일했다.
구독 준비 대기와 재생 QoS 보완 후 반복 시험에서도 두 모드 각각 99/99가 동일했고,
일반 재생과 처리 프레임별 재생의 모든 CSV 행이 각각의 재기록 bag 입력에 연결됐다
(`record_ready_replay_validation.json`).
원래 파라미터를 쓰려면 사용자가 `tuning_config:=.../replay_tuning.yaml`을 명시한다.
실행 중 파라미터 변경은 bag에 기록하지만 재생 중 자동 적용하지 않는다.

수직/평행/수동 보정도 실제 ROS에서 합성 Scan/격리된 ARM 상태로 기록을 검증했다.
주차의 geometry_confirmed는 0을 유지했고 보정의 disarm 발행 PWM 0을 확인했다.
모터·시리얼·실제 센서는 사용하지 않았다. 전체 회귀 검사 및 GUI/정지 검증 결과는
`reports/tuning_analysis_2026-10-05/verification/`에 보존한다.
회귀 테스트 142개와 7개 패키지 빌드가 통과했고, 5개 모드의 GUI·파라미터 적용·임시 P 저장을
확인했다. 기존 20Hz 송신/10Hz 생존 확인, 짧은 지연 중 명령 유지, 6초 인지 중단 정지,
worker/카메라 중단, 제어 프로세스 종료, S/Ctrl+C 및 새 W 필요 조건도 모의 검증했다.
차선 전환 분석 도구의 직전·현재·직후 기록은 `mission_scenario_analysis.json`에 별도로 보존한다.

## 주차 화면과 측정 양식

주 화면은 **현재 뒤차축 중앙 원점, +X 전방, +Y 왼쪽**이다. 차량 외곽은
x=-rear_overhang부터 wheelbase+front_overhang, y=±vehicle_width/2로 표시한다.
치수가 없으면 임의 차량 외곽을 그리지 않는다. 설정값이 있으나 미확인이면 주황 외곽과
UNCONFIRMED를 표시한다. geometry_confirmed=0에서는 주차 제어의 기존 잠금을 유지한다.

LiDAR 위치/처리된 scan의 0도 방향 화살표, FOV/최소 거리 사각 영역, 측면 검사 sector와
거리, 수직주차 start_detect 거리 arc, 현재 단계와 추정 자세를 표시한다.
추정 자세는 **출발 시 뒤차축 기준의 별도 inset**이며 PWM 속도 기반 bicycle 추정이고
odometry가 아니다. 속도/바퀴각 실측이 없으면 pose_estimate_valid=false와 UNCONFIRMED pose를 표시한다. 주 화면의 센서 점은 현재 차량 좌표이므로 추정 world pose를 다시 더하지 않는다.
`lidar_rotation`은 드라이버에서 한 번 적용한다. 주차는 scan_angle_sign/lidar_yaw_deg의
측정 보정만 적용하며 driver_rotation_offset_deg는 화면 표시 전용으로 좌표에 재적용하지 않는다.
합성 Scan에서 driver 180/yaw 30 표시와 30도 몸체 변환을 확인했다. 실차 설치 방향 검증은 별도다.

측정 양식은 `tools/forms/`에 있다. 모든 실측값은 빈 값/NULL이며 자동 반영하지 않는다.

| 실측 항목 / 반영 파라미터 | 단위·기준점 | 작성 양식 |
|---|---|---|
| wheelbase_m | m, 뒤차축→앞차축 중심선 | parking_geometry.csv |
| front_overhang_m / rear_overhang_m | m, 각각 해당 차축→최전방/최후방 외곽 | 동일 |
| vehicle_width_m | m, 타이어 포함 최대 외곽 폭 | 동일 |
| lidar_x_m / lidar_y_m | m, 뒤차축 중앙→그릴 LiDAR 스캔 중심, +전방/+왼쪽 | 동일 |
| lidar_yaw_deg / scan_angle_sign | deg / ±1, **드라이버 회전 적용 후** 알려진 전방/좌측 표적으로 확인 | 동일 |
| visible_min_deg / visible_max_deg | deg, 차량 +X=0, 실제 차체 가림을 제외한 유효 범위 | 동일 |
| forward_pwm / reverse_pwm, forward_mps / reverse_mps | signed PWM 시험, 거리(m)/실제 이동시간(s), 속도는 양의 크기(m/s) | parking_speed_samples.csv |
| max_wheel_angle_deg | deg, 차량 직진 기준 ±7에서 **좌우 앞바퀴 각각** 각도 기록 | parking_steering_samples.csv |
| slot_depth_m / entry_lateral_m / entry_advance_m | m, 공간 유효 폭/뒤차축 목표 횡변위/공간 끝 검출 후 뒤차축 진입 위치 | parking_geometry.csv |

속도는 같은 기준점(예: 뒤차축 중앙의 지면 표식)을 사용해 PWM별 전진/후진을 각각
여러 번 측정한다. 실제 이동 시작~끝 시간과 거리, 배터리/노면/하중을 함께 기록한다.
GUI ARM 경과시간에는 지연/가속/감속/정지 시간이 포함될 수 있어 그대로 등속 m/s로 보지 않는다.
앞바퀴 각도는 0 정렬을 기준으로 각 바퀴 평면/직진선 사이 각도를 각도계나 위쪽 사진으로
측정하고 좌회전을 양수로 기록한다. 두 바퀴의 Ackermann 차이와 ±7 비대칭도 보존한다.
기존 주차 모델에는 대칭 단일 max_wheel_angle_deg만 있어 비대칭을 모두 표현할 수 없다.
측정/검토 후 유효 bicycle 각도를 수동으로 반영하며 값을 추측하지 않는다.

반영 파일: `~/.config/autodrive/perpendicular_parking.yaml`의
`perpendicular_parking_controller_node.ros__parameters`,
`~/.config/autodrive/parallel_parking.yaml`의 `parallel_parking_controller_node.ros__parameters`.
주차 PWM에 해당하는 실측 속도를 넣고 PWM이 바뀌면 재측정한다.
그릴 위치를 wheelbase+front_overhang으로 자동 추정하지 않는다.
`parking_measurements.template.yaml`은 기록 양식이며 ROS tuning_config 파일이 아니다.
실측 확인·검토 전에는 geometry_confirmed를 켜지 않는다.

## 사용할 명령

```bash
cd ~/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup_pkg
source install/setup.bash
```

각 기록 폴더는 새 이름을 쓴다. 기존 세션 덮어쓰기는 거부한다.
첫 센서 확인/기록은 다음처럼 sender와 gate를 제외한다.

```bash
ros2 launch vehicle_bringup_pkg track_drive_tuning.launch.py sensors_only:=true camera_device:=/dev/video0 record_dir:=/home/autolab/autodrive_logs/track_001
ros2 launch vehicle_bringup_pkg mission_drive_tuning.launch.py sensors_only:=true camera_device:=/dev/video0 record_dir:=/home/autolab/autodrive_logs/mission_001
ros2 launch vehicle_bringup_pkg perpendicular_parking.launch.py sensors_only:=true record_dir:=/home/autolab/autodrive_logs/perpendicular_001
ros2 launch vehicle_bringup_pkg parallel_parking.launch.py sensors_only:=true record_dir:=/home/autolab/autodrive_logs/parallel_001
ros2 launch vehicle_bringup_pkg parking_calibration.launch.py dry_run:=true record_dir:=/home/autolab/autodrive_logs/calibration_preview_001
```

/dev/video0는 확인한 실제 카메라 장치로 바꾼다. 내장 카메라는 별도 /dev/video2이며
그 결과를 실차 주행 성능으로 해석하지 않는다. 네 모드와 보정 도구는 한 번에 하나씩
실행한다. 실차 주행을 운영자가 시작할 때는 기존 실차 launch에 `record_dir`만 추가한다.
수동 실측도 기존 parking_calibration launch/GUI의 PWM·조향 설정과 W/S를 사용한다.
기록 옵션 자체가 W를 누르거나 모터를 켜지는 않는다.

저장된 입력을 GUI/계산에 재생한다. 아래 launch는 dry_run을 강제하며 sender/gate를 실행하지 않는다.

```bash
ros2 launch vehicle_bringup_pkg tuning_replay.launch.py mode:=track input_kind:=bag input_path:=/home/autolab/autodrive_logs/track_001/input_bag tuning_config:=/home/autolab/autodrive_logs/track_001/replay_tuning.yaml
ros2 launch vehicle_bringup_pkg tuning_replay.launch.py mode:=mission input_kind:=bag input_path:=/home/autolab/autodrive_logs/mission_001/input_bag tuning_config:=/home/autolab/autodrive_logs/mission_001/replay_tuning.yaml processed_csv:=/home/autolab/autodrive_logs/mission_001/control.csv
ros2 launch vehicle_bringup_pkg tuning_replay.launch.py mode:=perpendicular input_kind:=bag input_path:=/home/autolab/autodrive_logs/perpendicular_001/input_bag tuning_config:=/home/autolab/autodrive_logs/perpendicular_001/replay_tuning.yaml
ros2 launch vehicle_bringup_pkg tuning_replay.launch.py mode:=parallel input_kind:=bag input_path:=/home/autolab/autodrive_logs/parallel_001/input_bag tuning_config:=/home/autolab/autodrive_logs/parallel_001/replay_tuning.yaml
ros2 launch vehicle_bringup_pkg tuning_replay.launch.py mode:=calibration input_kind:=bag input_path:=/home/autolab/autodrive_logs/calibration_preview_001/input_bag tuning_config:=/home/autolab/autodrive_logs/calibration_preview_001/replay_tuning.yaml
```

주차/보정 bag의 ARM 상태는 `/dry_run/vehicle/drive_state`로만 재생한다. 모터 명령,
W/arm 요청, key, controller health는 재발행하지 않는다. 원본 bag에 ARM 상태가 없으면
자동으로 만들어 출발시키지 않는다. 분석/재생은 별도 ROS_DOMAIN_ID로 실제 주행과 분리할 수 있다.
`processed_csv`는 트랙/미션 단일 재생 전용이다. 일반 재생은 `rate:=0.5`, 영상은
`playback_fps:=10.0`처럼 속도를 정할 수 있고 GUI만 길게 보고 싶으면 `loop:=true`를 쓴다.
processed_csv와 loop는 함께 사용하지 않는다.

```bash
ros2 launch vehicle_bringup_pkg tuning_replay.launch.py mode:=mission input_kind:=video input_path:=/home/autolab/autodrive_ws/h-mobility-class/src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/driving_simulation.mp4 playback_fps:=10.0
python3 tools/analyze_tuning_log.py /home/autolab/autodrive_logs/mission_001/control.csv --output /home/autolab/autodrive_logs/mission_001/analysis.json
ROS_DOMAIN_ID=147 python3 tools/compare_drive_models.py --video /home/autolab/autodrive_ws/h-mobility-class/src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/driving_simulation.mp4 --device cuda:0 --output /home/autolab/autodrive_logs/model_compare_001
```

기록 OFF/ON·일반 bag 재생·처리 프레임별 일치 재생의 소프트웨어 시험을 다시 실행하려면
`ROS_DOMAIN_ID=149 python3 tools/validate_tuning_session.py --output /tmp/tuning_session_check_001`을
사용한다. 새 출력 경로를 지정해야 하며 이 시험은 dry_run, GUI OFF와 기존 예제 영상만 사용한다.

재생 결과도 저장하려면 재생 launch에 별도의 `record_dir:=...`를 추가한다.
`analysis.json`에는 상태별 개수/처리시간 분포와 차선 전환 직전·현재·직후의 경로,
CTE/방향 오차/조향 변화량을 넣는다. 재생 GUI의 P 저장을 원본 운영 튜닝과 분리하려면
명시적 tuning_config와 함께 `saved_tuning_path:=/home/autolab/autodrive_logs/mission_001/replay_gui_tuning.yaml`을
추가한다. 이 인자를 생략하면 기존 모드의 사용자 P 저장 경로를 사용한다.
