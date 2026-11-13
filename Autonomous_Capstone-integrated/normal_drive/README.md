# 일반주행 E2E 작업본

원본 저장소와 별도로 실행하는 ROS 2 일반주행 + 기록·PilotNet 학습 작업본이다. 원본 코드/데이터는 삭제하지 않았다. 웹 GUI는 차량과 연결되지 않는다. 첫 버전의 속도는 **고정 PWM**이며 실제 일정한 m/s 저속은 보장하지 않는다.

- [실제 호출 경로·단위·변경 근거](docs/architecture_ko.md)
- [검증 결과·미검증 범위](docs/validation_ko.md)
- [원본 복사 파일 목록과 SHA256](docs/source_inventory.json)
- GUI 스크린샷: [영상 검토](docs/screenshots/review.png), [데이터셋](docs/screenshots/dataset.png), [학습](docs/screenshots/training.png), [평가](docs/screenshots/evaluation.png)

## 지금 이 PC에서 웹 GUI 실행

ROS source나 차량 연결이 필요 없다. rosbags 등 GUI/변환 의존성은 작업본 `.deps`에 설치했다. 실제 호스트에는 RTX 4050 GPU가 있으며 CPU 또는 cuda:0를 선택할 수 있다. 샌드박스에서는 GPU가 가려질 수 있다.

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated/normal_drive
./e2e serve --work work/final_gui --port 8765
```

브라우저: **http://127.0.0.1:8765**. `work/final_gui`에는 실제로 검증한 **합성 DEMO** 3회차, 데이터셋, 중단·재개 checkpoint, A/B 평가가 있다. 실제 세션은 별도 `--work work/real`을 권장한다. GUI의 파일 경로는 **서버 PC의 폴더**다. 로컬 폴더를 선택해 가져오기 → 검토 승인/제외/라벨 수정 → 회차별 train/validation/test → 버전 생성 → 학습 → 시험 평가 순서다. 운영자 검토 승인과 실제 주행 성공은 다르다.

## 일반주행 전용 ROS 빌드

원본 루트에서 빌드하지 말고 이 작업본으로 이동한다. ROS 노드 패키지 이름은 원본과 같으므로 **새 터미널에서 이 overlay를 마지막에 source**한다.

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated/normal_drive
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
ros2 pkg prefix skku_track_drive_pkg
ros2 launch vehicle_bringup_pkg normal_teacher.launch.py --show-args
```

`ros2 pkg prefix`가 `normal_drive/install/...`인지 확인한다. 로그 경로 제한이 있는 환경에서는 `export ROS_LOG_DIR=/tmp/normal_drive_ros_logs`를 사용한다.

## 센서만 실행하고 기록

다음 명령은 **시리얼·arm 노드를 생성하지 않는다.** 카메라/교사/기존 튜너와 별도 recorder만 실행한다. 센서만 기록한 데이터는 실제 전송·운전 허용 증거가 없어 기본 학습 필터에서 제외된다.

```bash
ros2 launch vehicle_bringup_pkg normal_teacher.launch.py \
  camera_device:=/dev/video2 device:=cpu sensors_only:=true \
  record_dir:="$PWD/work/records/bench_$(date +%Y%m%d_%H%M%S)"
```

CUDA 드라이버가 정상인 PC에서만 `device:=cuda:0`으로 바꾼다. record_dir은 **아직 존재하지 않는 디렉터리**여야 한다. 기존 폴더를 덮어쓰지 않는다. Ctrl+C로 정상 종료한다.

실차 운영 시 사용자가 직접 실행할 형식은 아래와 같다. 이번 작업에서는 실행하지 않았다. `검증한_PWM`은 바퀴를 띄운 시험과 저속 확인으로 정한 값으로 교체한다. 기존 저장값 255를 저속이라고 간주하지 않는다.

```bash
ros2 launch vehicle_bringup_pkg normal_teacher.launch.py \
  camera_device:=/dev/video2 device:=cpu sensors_only:=false \
  speed:=검증한_PWM \
  record_dir:="$PWD/work/records/drive_$(date +%Y%m%d_%H%M%S)"
```

기존 튜너·DriveArm UI가 살아 있어야 하며 READY 이후 사용자의 fresh W가 있어야 승인된다. S, UI·controller 감시, 시리얼 fault 정지 경로는 유지한다. 고정 PWM 외 조향 튜닝은 원본 package 값 + 복사한 사용자 snapshot을 사용한다. 전용 튜너 저장 파일은 `~/.config/autodrive/normal_track_tuning.yaml`이다.

## 기록 파일과 CLI 변환

```text
records/<session>/
  manifest.json                   # run_id, clock/단위, 코드 hash, 정상 종료 여부
  input_bag/*.db3 + metadata.yaml # SourceFrame 원본 영상 + TeacherCommand + 상태/전송
  control.csv / events.jsonl      # 작은 기존 교사 분석·전송 증거
  loaded_parameters.yaml         # controller 설정 (가용 시)
  camera_publisher_node_parameters.yaml / serial_sender_node_v2_parameters.yaml
  replay_tuning.yaml / bag_qos.yaml / rosbag.log
work/sessions/<id>/
  session.json                   # bag-relative index, identity 감사, 매칭 통계
  review.jsonl                   # append-only 수정·되돌리기 이력
work/datasets/<version>/
  dataset.json / images/*.png     # 선택한 학습 이미지, 고정 라벨·분할·SHA256
work/training/<id>/
  config.json / metrics.json / best.pt / last.pt
```

영상은 기록 시 SourceFrame bag에 한 번 저장한다. import는 이미지를 복제하지 않는다. GUI preview는 요청할 때 디코딩한다. dataset을 고정할 때 선택한 프레임만 lossless PNG로 추출한다. SQLite3 `.db3` bag을 지원한다. MCAP은 먼저 별도로 sqlite3로 변환해야 한다. 손상된 metadata.yaml이나 비정상 종료 세션도 db3의 읽을 수 있는 메시지를 import하고 partial로 표시한다. 원본 bag을 수정하거나 자동 reindex하지 않는다.

```bash
./e2e import /data/records/run_001 work/real/sessions/run_001
./e2e dataset --work work/real --config examples/dataset.json
./e2e train --dataset work/real/datasets/DATASET_ID \
  --output work/real/training/model_001 --config examples/train.json
./e2e evaluate --dataset work/real/datasets/DATASET_ID \
  --checkpoints work/real/training/model_001/best.pt \
  --output work/real/evaluation.json --device cpu
```

`examples/dataset.json`의 회차 ID를 실제 import ID로 바꾼다. 세 회차를 따로 가져와야 기본 train/validation/test 생성이 가능하다. 조향 라벨은 teacher step/7, 음수=왼쪽이다. `require_review`와 `require_transmission`은 기본 true다. legacy/partial/DEMO/수동 정렬은 각각 명시적으로 허용해야 한다. GUI와 CLI는 같은 `create_dataset`, `train`, `evaluate` 함수를 호출한다. GUI 작업의 정확한 설정은 `work/.../jobs/<job_id>/request.json`에서 재사용할 수 있다.

외부 PilotNet checkpoint는 학습 화면의 로컬 경로 가져오기로 형식 검사 후 등록할 수 있다. CLI는 `./e2e model-import /path/model.pt work/real/models`다. 실제 repository YOLO .pt 거부도 브라우저에서 시험했다.

신규 학습은 `mode=new`, checkpoint 공란이다. 파인튜닝은 `mode=finetune` + E2E checkpoint, 재개는 `mode=resume` + last.pt이며 epochs/batch/lr/optimizer/scheduler/augmentation/seed/데이터셋/전처리를 그대로 유지한다. 재개 checkpoint에서 이미 끝난 epoch까지 다시 학습하지 않는다. 총 epoch를 바꾸려면 파인튜닝을 사용한다. SIGINT/SIGTERM 중단은 epoch 경계에서 last.pt를 저장한다. OOM/SIGKILL은 마지막으로 완료된 checkpoint까지만 복구한다.

## ROS shadow 추론

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run e2e_learning_pkg e2e_shadow --ros-args \
  -p checkpoint:="$PWD/work/real/training/model_001/best.pt" -p device:=cpu
```

입력 `/e2e/source_frame`, 출력 `/e2e/prediction`만 사용한다. YOLO .pt는 검증 단계에서 거부한다. shadow는 steering step 예측, source ID/stamp, 실행시간, 모델 dataset_version을 발행하고 차량 명령은 발행하지 않는다. 학습과 동일한 전처리 함수를 쓴다. 향후 제어 통합은 [분석 문서의 통합 계약](docs/architecture_ko.md)을 따른다. 자동 실차 전환은 구현·활성화하지 않았다.

저장 bag을 재생할 때는 **원래의 차량 ROS domain과 분리**하고 SourceFrame만 명시적으로 선택한다. 아래는 사용자 실행용이며 이번 작업은 별도 합성 publisher로 검증했다.

```bash
ROS_DOMAIN_ID=203 ROS_LOCALHOST_ONLY=1 ros2 bag play /data/records/run_001/input_bag \
  --topics /e2e/source_frame
# shadow 터미널에도 같은 domain/localhost 환경변수를 적용
```

## 차량 없는 다른 PC

Python 3.10+, torch, NumPy, OpenCV, rosbags만으로 웹/변환/학습/평가를 실행할 수 있다. ROS는 shadow/차량 기록에만 필요하다. 이 폴더의 `src/e2e_learning_pkg`, `e2e`, 요구사항, 세션을 복사한다. 환경이 다르면 가상환경에 설치한다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-offline.txt
python -m pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cpu
./e2e serve --work work --port 8765
```

CPU wheel 명령은 [PyTorch 이전 버전 공식 안내](https://pytorch.org/get-started/previous-versions/)에 근거한다. GPU wheel은 해당 PC 드라이버에 맞게 선택한다. 이 PC는 `python3-venv/ensurepip`가 없어 `.deps --target` 방식으로 rosbags/Playwright를 설치했다. 글로벌 Python·CUDA·드라이버는 변경하지 않았다. `.deps`는 다른 OS/아키텍처로 복사하지 말고 재설치한다.

세션을 다른 위치로 옮겼다면 GUI에서 새 위치를 재import하면 된다. 수정 이력까지 옮기려면 work와 원본 세션의 상대 위치를 유지해 복사한다. 데이터셋은 자체 images를 포함하므로 단독 이동 가능하다. 모델은 전처리·출력 계약·데이터셋 버전·학습 상태를 포함한다. 원격 업로드/멀티유저 서비스는 제공하지 않는다.

## 재검증

```bash
PYTHONPATH="$PWD/.deps:$PWD/src/e2e_learning_pkg" python3 -m pytest \
  tests/test_backend.py tests/test_learning.py -q
PYTHONPATH="$PWD/.deps:$PWD/src/e2e_learning_pkg" python3 tests/compare_teacher.py
```

실제 GUI 시험은 `tests/browser_flow.py`, ROS 기록/복구/교사/shadow 시험은 `tests/ros_*_smoke.py`다. 각각 합성 데이터만 사용하며 별도 ROS domain을 지정해야 한다. 브라우저 시험은 설치된 Chrome과 Playwright가 필요하다. 실행 증거는 `docs/*validation.json`, 안전성 시험 결과는 `docs/safety_regression.txt`와 `docs/trace_serial_test.txt`에 있다.

## 사용자가 실차에서 확인할 짧은 순서

1. 작업본 overlay 경로·카메라 원본 영상·원본 튜닝 snapshot을 확인한다. 첫 시험은 `sensors_only=true`로 한다.
2. 바퀴를 띄운 상태에서 보드의 좌우 step 부호, 중앙/한계 ADC, STOP, fresh W 재승인, UI/프로세스/통신 단절 정지를 확인한다. 첫 저속 PWM을 실제 측정해 정한다.
3. 운영자가 기존 교사로 짧게 운전하고 Ctrl+C로 기록을 닫는다. GUI에서 원본 영상·교사·전송의 ID와 stop_reason, 누락/partial을 확인한다.
4. 주행 영상을 검토해 이탈/정지/수동 개입/실패 구간을 제외하고 회차별 train/validation/test로 고정한다. 서로 다른 독립 주행 회차가 필요하다.
5. 학습 모델을 shadow로만 실행해 고정 시험 세션 오차·지연·step 양자화를 확인한다. 실차 폐루프 E2E 전환은 별도 검증 후 명시적으로 진행한다.
