# 로컬 일반주행 분석과 구현 근거

분석 기준: `/home/autolab/autodrive_ws/Autonomous_Capstone-integrated`, 브랜치 `1030`, commit `56395e0c9930bf88bf616c061b86faf0ebff1d65`. 시작 시 작업 트리는 깨끗했다. 사용자가 원격 비교 대신 로컬 기준 진행을 승인했다. GitHub main은 기준으로 대체하지 않았다.

## 실제 호출 경로

`src/vehicle_bringup_pkg/launch/track_drive_tuning.launch.py:generate_launch_description → OpaqueFunction(_launch_nodes)`를 따라 확인했다. 기본 `bbox_obstacle_avoidance=false`, `use_lidar=false`; 이때 미션/주차 노드는 호출되지 않는다. `bbox_obstacle_avoidance=true`일 때만 `mission_bbox_only.launch.py`로 분기한다.

| 단계 / 실제 함수 | 입력 → 출력 | 단위 / 시각 / 주기 |
|---|---|---|
| `mode_launch.camera_node → CameraPublisherNode.on_timer` | V4L2 `/dev/video2`, MJPG 요청, 640×480 → `/track/image_raw` BGR8 | 요청 30 Hz, buffer 1. `cap.read()` 후 필요시 resize와 CvBridge 변환을 마친 뒤 호스트 ROS stamp 부여. 센서 노출 시각 아님. 캡처 오류 2회 후 재연결 |
| `TrackControllerNode.receive_image → process_latest_image → on_image` | ROS Image → 최신 프레임 한 개 | DDS depth 1 + 내부 latest 슬롯. 수신은 health callback group, 추론은 별도 callback. 바쁜 동안 중간 프레임을 덮어쓰므로 카메라 FPS와 명령 생성률은 다름 |
| `_image_to_bgr → YoloDetector.detect → _predict` | 원본 BGR → `best_v3.pt` Ultralytics YOLO segmentation | `retina_masks=True`; 원본 좌표 polygon/bitmap 복구, conf는 로컬 YAML 0.6, device는 명시값. OpenVINO 분기는 이 launch에서 사용하지 않음 |
| `LaneInfoExtractor.process → camera_perception_core.draw_edges/clean_lane_mask/...` | `lane2` 마스크 → perspective warp, ROI, 중심선 target points | BEV 픽셀 좌표. 전체 마스크를 homography로 변환한 뒤 roi_cut=300. 실거리 아님. EMA·virtual lane·관측 경계·일시 누락 유지 로직 포함 |
| `PathPlanner.plan` | 유효 차선점 → `scipy.interpolate.CubicSpline` 자연 경계 100점 경로 | BEV px. 차선 중심선, 차량 위치를 강제로 spline에 추가하지 않음 |
| `MotionPlanner.plan → compute_stanley_steering` | 경로 + 설정 → 내부 MotionCommand | heading + Stanley CTE, 내부 `v=1`은 실측 속도 아님. max_steering_angle=50은 명령 매핑용 내부 각도 상한, 실측 바퀴각 아님. round/clamp로 정수 -7…7 |
| `TrackControllerNode.on_image` | 내부 명령 → `interfaces_pkg/MotionCommand` | `steering:int32`, `left_speed/right_speed:int32`, 헤더 없음. enabled=false면 양쪽 PWM=0. 경로 일시 누락 시 기존 알고리즘은 마지막 조향과 설정 PWM 유지; path_valid만으로 성공 판정 불가 |
| `ControllerLiveness` / `PerceptionProgress` | 컨트롤러 health/progress → heartbeat | 독립 실행기·lease. command age와 health/progress silence를 구분 |
| `TrackTunerNode`, `DriveArmNode` | W/S 및 두 UI heartbeat, challenge → arm request/armed | fresh W + READY 필요. S, UI/health timeout, serial fault 후 재승인 필요. 웹 학습 GUI는 이 승인 UI를 대체하지 않음 |
| `SerialSenderNode.on_cmd → enforce_safe_state → write_gated_command → write_serial` | 요청 명령 + gate → ASCII `s{step}l{pwm}r{pwm}\n` / `X\n` | 주기 50 ms = 20 Hz, zero는 즉시 반영 가능. 115200 baud. PWM -255…255, m/s 아님. write 성공은 OS/드라이버가 요청 바이트를 받았다는 뜻; MCU 수신·액추에이터 응답 확인 아님 |

`MotionPlanner`의 오래된 주석에는 부호가 반대로 적힌 부분이 있지만 실제 node 설명과 현재 firmware의 pot map은 음수=왼쪽, 양수=오른쪽이다. 설치된 보드의 firmware/차륜 방향은 실차 확인 대상이다. `src/control/driving_user_pins` 계열은 프로그램 소스이며 실제 업로드된 firmware를 읽어서 확인한 것은 아니다.

## 설정과 보조 기능

- 모델 기본 설치 위치: `share/skku_track_drive_pkg/models/best_v3.pt`. 원본 `setup.py` glob와 실제 파일을 확인했다.
- `tuning_configuration.resolve_tuning`: package `track_tuning.yaml` 기본값 위에 `~/.config/autodrive/track_tuning.yaml`을 병합하고 launch 명시 인자를 최우선 적용한다.
- 현재 유효값: speed **255 PWM**, stanley_gain .04, heading_gain .5, lookahead_index 30, confidence .6, bev_top_shift -10, roi_cut 300, look_shift 0, ema_alpha .6, virtual_lane_width 240, car_center_x **325**, car_center_y 179, reference_roi_cut 300. x=325는 저장 YAML에 없어서 package 기본값에서 상속된다.
- 기존 카메라 `header.frame_id='camera_frame'`은 좌표 프레임 이름이다. 순번 용도로 바꾸지 않았다.
- 기존 `tuning_recorder_node`: 독립 ROS 프로세스, ros2 bag subprocess, timing String → CSV/events, controller parameter dump/model hash. 기존 영상 별도 캡처는 중복 저장 가능성이 있어 작업본에서 제외했다.
- 원본 `tuning_bag_replay_node`와 `video_replay_node`는 프레임 재생·출처 정보를 제공하지만, 기존 MotionCommand 자체에는 source identity가 없다. 새 기록은 SourceFrame wrapper 자체를 bag에서 재생하면 원본 identity/stamp를 그대로 보존한다.
- 기존 `tools/tests`, `skku_track_drive_pkg/test`에서 liveness, stop history, perception delay, BEV ROI, capture, tuning 테스트를 확인했다. 미션/주차 테스트는 전용 작업본의 빌드 대상이 아니다. 새 검증은 `normal_drive/tests`에 별도로 두었다.

## 전용 작업본과 변경 구분

원본 `src/`, datasets, reports는 수정·삭제하지 않았다. `normal_drive/COLCON_IGNORE`로 원본 루트 colcon이 중복 패키지를 재귀 발견하지 않게 했다. 작업본 안에서는 `colcon build --base-paths src`로 명시한다.

`docs/source_inventory.json`에 복사한 원본 파일 SHA256과 근거가 있다. 유지한 5개 ROS 패키지에 `e2e_learning_pkg`를 추가했다. 카메라/주행제어/게이트/메시지/launch만 남기고 manual_drive, lidar, mission, parking, legacy 패키지·노드 entry point·모델은 포함하지 않았다. MotionPlanner와 tuner의 공통 구현에 있는 비활성 확장 지점은 공용 코드로 유지했다. 미션 제어기/상태기계 import는 없다. 튜너의 신호등 패널 의존성만 제거했다.

변경이 없는 알고리즘: YOLO wrapper, lane_processing, camera_perception_core, BEV geometry, path_planner, motion_planner, messages. 원본과 byte 비교 및 동일 합성 차선 입력의 경로·명령 비교 결과는 `teacher_equivalence.json`에 있다. 실제 주행 bag 동일 입력 비교는 자료가 없어 미수행이다.

의도적으로 달라지는 실행 구성:

1. SourceFrame이 카메라 UUID/sequence를 Image와 같은 메시지에 담고, TeacherCommand가 이를 직접 전달한다. sender는 legacy 토픽 대신 이 추적 명령을 받는다. 튜너용 legacy 명령도 동일 값으로 계속 발행한다.
2. 원본 알고리즘·조향 튜닝은 유지한다. 전용 설정 기본값은 원본 package YAML + 복사한 사용자 snapshot 병합이다. 전용 P 저장 경로는 `~/.config/autodrive/normal_track_tuning.yaml`로 원본과 분리한다.
3. 기본 `sensors_only=true`, device=cpu. 직렬 노드를 실행하려면 `sensors_only=false speed:=값`이 필요하다. **PWM 80은 예시이지 실제 저속 보장값이 아니다.** 고정 PWM이며 실측 일정 m/s 제어가 아니다. 런타임 speed 튜닝은 잠근다.
4. perception silence 한계는 원본 6초, UI .75초, controller health lease 방식, STOP·challenge·fresh W 로직은 유지한다. 새 gate/serial 알고리즘을 도입하지 않았다. serial telemetry 비용/SourceFrame DDS 부하는 실차 계측 대상이다.
5. SourceFrame 영상 경로는 best-effort depth 1이다. 느린 기록기가 카메라·제어에 reliable backpressure를 주지 않도록 recorder 구독도 best-effort로 설정한다. 원본 reliable 영상 수신과 달리 부하 시 프레임 손실 특성이 바뀔 수 있다. ID gap과 매칭 실패로 드러내고 학습에 임의 보간하지 않는다. 주행 타이밍 완전 동등성은 실차 미검증이다.

## 데이터와 시간 계약

- `SourceFrame`: `camera_instance`는 카메라 프로세스 UUID, `sequence:uint64`는 그 인스턴스의 발행 순번. Image header 좌표 frame_id는 그대로다. stamp는 카메라 read/resize 이후 host ROS system clock이다. 센서 노출 지연은 null/unmeasured.
- `TeacherCommand`: command_id는 컨트롤러 UUID+증가 순번. 실제 사용한 SourceFrame의 UUID, sequence, 원본 stamp, clock_domain을 복사한다. 재처리한 동일 프레임에 서로 다른 명령이 생기면 ambiguous로 기본 제외한다. STOP에는 가짜 프레임을 붙이지 않는다.
- transmission_id는 sender UUID+쓰기 시도 순번. command_id는 반복 전송 동안 유지한다. receipt ROS/monotonic ns, serial write 시작/완료 monotonic ns, payload, 성공, armed, calibration_ready, stop_reason 기록. `X`는 게이트 정지 요청이며 교사 주행 샘플로 사용하지 않는다.
- ns는 Python/ROS/JSON 파일에서 정수다. 브라우저 API는 `_ns`와 sequence를 **십진 문자열**로 보낸다. JS는 BigInt로 차이를 계산해 정밀도를 유지한다.
- 경과시간은 같은 host monotonic domain의 각 처리·write 내부에서만 계산한다. ROS system stamp와 monotonic 값을 직접 빼지 않는다. replay 때 원본 stamp와 새 수신/예측 시각을 구분한다. sim-time 카메라 운용은 이 launch의 지원 범위가 아니다.
- 직접 매칭 조건은 UUID+sequence+원본 stamp+clock domain 전부 일치. 가까운 시각 자동 연결 없음. 이전 bag은 유일한 **원본 header exact stamp** 정보가 있을 때만 legacy 수준으로 표시한다. 영상만 있으면 unlabeled.
- 원본 시각 정렬 메모/legacy 명시 command_id 재연결과 조향 라벨 수정은 별도 이력이다. 직접 매칭은 offset으로 수정 불가. 수동 연결 샘플은 별도 허용과 운영자 검토가 필요하다.
- source ID 누락/중복/충돌/재처리/없는 명령/시각 불일치/clock 불일치/디코딩 실패를 통계로 공개한다. DDS 내부의 정확한 queue overflow 횟수는 계측할 수 없어 **불명**으로 표시한다. sequence gap, camera 발행 상태, rosbag 로그로 손실을 감사한다. 세션 시작 전/끝 이후 손실량은 완전 확정할 수 없다.

## 학습·평가와 통합

PilotNet 원논문 구조를 따른 5개 conv(24/36/48/64/64)와 100/50/10 회귀 head, ELU/Tanh 변형이다. 논문 그대로의 물리 조향각 예측이 아니라 현재 차량의 정규화 command step 회귀다. 입력 기본 200×66 YUV, crop `[.35,1,0,1]` 영상 비율, `pixel/127.5-1`. 같은 preprocess 함수가 학습·미리보기·평가·shadow에서 사용된다. crop/색상/크기/정규화는 checkpoint로 고정된다.

회차 train/validation/test 분리, origin_run_id/같은 source identity 중복 누출 방지, train만 선택적 bin cap, 기본 운영자 검토·전진·경로 유효·관측·교사와 동일한 허용 전송 요청 필터. path_valid와 운전자 검토를 주행 성공률로 표시하지 않는다. 데이터셋은 review snapshot, 설정, 원본 identity, 이미지 SHA256, manifest 무결성, 버전을 함께 보존한다. manifest/이미지가 바뀌면 학습·평가가 거부된다.

신규/가중치 파인튜닝/optimizer·scheduler·RNG 포함 재개를 구분한다. 재개 설정과 데이터셋 불일치를 거부한다. train/validation loss와 gap을 표시하고 best/last checkpoint를 저장한다. 중단은 epoch 경계에서 저장한다. 파일 전용 magic/output contract를 `torch.load(weights_only=True)`로 확인하므로 YOLO .pt 선택은 실패한다.

ROS shadow는 `/e2e/prediction` String만 발행한다. MotionCommand publisher, serial, arm service는 없다. 향후 실구동 통합은 source identity와 freshness를 확인하는 명시적 teacher/E2E selector를 **기존 승인/통신감시 게이트 앞**에 놓고 step=`round(clamp(pred,-1,1)*7)`, PWM은 별도 검증한 고정값으로 구성한다. shadow의 소수 예측과 실제 정수 step의 차이를 따로 평가해야 한다. 자동 구동 selector는 이번 구현에 활성화하지 않았다.

웹 서버는 127.0.0.1만 수신, 동일 출처 토큰으로 변경 요청을 제한한다. 브라우저/HTTP backend/변환·학습 subprocess를 분리했다. 로컬 경로 가져오기가 기본이며 HTTP 대용량 업로드는 제공하지 않는다. 원격은 rsync/외장 디스크로 세션을 옮기고 해당 PC에서 import한다. GUI는 bag 옆 절대경로 대신 registry-relative 경로를 저장한다. work+세션의 상대 위치를 유지해 복사하거나 새 PC에서 재import할 수 있다. 배포용 인증·멀티유저·원격 업로드는 범위 밖이다.

## 근거 자료

- NVIDIA 연구진, [End to End Learning for Self-Driving Cars (2016)](https://arxiv.org/abs/1604.07316): 영상에서 조향을 회귀하는 PilotNet 기반.
- [ROS 2 rosbag2 공식 소스](https://github.com/ros2/rosbag2/tree/humble): 설치된 Humble의 SequentialReader/Writer, ros2 bag record 및 QoS override 기능을 사용. docs.ros.org 페이지는 접근 제한으로 본문 확인이 안 되어 실제 설치 모듈·실행 결과를 근거로 삼았다.
- [PyTorch serialization 공식 문서](https://docs.pytorch.org/docs/stable/notes/serialization.html): state_dict 기반 저장. 실제 로컬 PyTorch 2.5.1에서 weights_only, optimizer/scheduler/RNG 저장·복원을 실행 검증했다.
