# 검증 결과와 한계

원본 `src/`, 사용자 데이터, Git 이력은 변경·삭제하지 않았다. 차량·USB 카메라·시리얼 포트를 열지 않았다. 실제 ROS 검증은 ROS_LOCALHOST_ONLY=1, 격리된 ROS_DOMAIN_ID 198~202에서 합성 publisher와 대상 노드만 사용했다. 일반주행 실차 launch의 구동 모드는 실행하지 않았다.

## 실제 확인한 환경

`environment.json` 기준 Python 3.10.12, ROS 2 Humble, PyTorch 2.5.1+cu124, torchvision .20.1+cu124, Ultralytics 8.4.6, OpenCV 4.12.0, NumPy 1.26.4, SciPy 1.8.0, PyYAML 5.4.1이다.

**최종 실제 호스트 확인에서 NVIDIA GeForce RTX 4050 Laptop GPU와 드라이버 595.99.02가 확인됐고 torch.cuda.is_available()=true였다.** 드라이버가 보고한 CUDA 지원 버전은 13.2, PyTorch build runtime은 12.4다. 처음 샌드박스에서 실행한 확인에서는 `/dev/nvidia0`가 가려져 false/드라이버 통신 실패가 나왔다. 그 결과를 실제 GPU 부재로 판단한 부분을 정정하고 호스트에서 GPU 검증을 추가했다. `environment_sandbox.json`은 최초 제한 환경의 증거, `host_devices.json`은 실제 호스트의 결과다. 카메라 `/dev/video2`는 실제 호스트에도 없으므로 카메라 실입력은 시험하지 않았다.

SciPy 1.8.0은 현재 NumPy 1.26.4에 지원 범위 경고를 출력한다. 동일 환경에서 아래 합성 경로·제어 시험은 통과했지만 장시간 실차 안정성을 입증하지는 않는다. 글로벌 패키지는 수정하지 않았다. rosbags .10.9와 Playwright 1.51.0은 작업본 `.deps`에 설치했다. 다른 PC의 요구사항 파일은 별도 가상환경 설치용이며 해당 새 환경 전체를 이 장비에서 재구축한 것은 아니다.

## 수행한 검증

| 항목 | 실행 결과 | 증거 |
|---|---|---|
| 전용 colcon 빌드 | 6개 패키지 성공 | `final_build.log` |
| launch 인자·튜닝 로딩 | 하드웨어 생성 없이 확인. 원본 기본+snapshot 병합, x=325 / gain=.04 / PWM255 유지 | `launch_arguments.txt`, `loaded_tuning.json` |
| 알고리즘 보존 | 원본과 7개 알고리즘 파일 byte 동일, 합성 차선 입력 7개에서 경로/조향/PWM 완전 동일 | `teacher_equivalence.json` |
| 신규 백엔드·학습 회귀 | 13 passed | `pytest_result.txt` |
| 기존 안전성 회귀 | 실제 분리 sender/arm/tuner callback + FakeSerial, 35 passed | `safety_regression.txt` |
| 추가 추적 시리얼 | 같은 command_id로 서로 다른 transmission_id 반복, payload·write 실패·disarm 확인 | `trace_serial_test.txt` (기존 35개를 포함해 36 passed) |
| 실제 ROS 기록기 | 합성 SourceFrame 18 / TeacherCommand 18 / 전송 36; 정상 종료 및 재import | `ros_record_validation.json` |
| 실제 강제 종료 복구 | 새 합성 기록 프로세스 그룹 SIGKILL 후 18프레임/36전송을 db3에서 읽음. closed_cleanly=false 유지 | `ros_recovery_validation.json` |
| 실제 CPU 교사 노드 | best_v3.pt YOLO 실행, 합성 4프레임의 source UUID/sequence/stamp/domain이 TeacherCommand까지 보존 | `ros_teacher_validation.json`, `ros_teacher_process.log` |
| 실제 ROS shadow | 학습 checkpoint 로딩, 예측 4건. MotionCommand·TeacherCommand publisher 0 | `shadow_validation.json` |
| 실제 GPU 학습·추론 | RTX 4050에서 합성 데이터 학습·중단·재개·고정 시험 평가 | `gpu_validation.json` |
| 실제 Chrome 전체 흐름 | import → 승인/라벨 수정/undo/제외·복원 → split → dataset → train 중단/재개 → A/B 평가 → worst frame 이동; JS 오류 0 | `browser_validation.json`, screenshots |
| GUI/CLI 공통 backend | GUI 요청 설정을 create_dataset에 다시 넣어 동일 dataset_id 확인 | `tests/browser_flow.py`, 고정 dataset manifest |
| GUI 모델 가져오기·구간 재생 | E2E 가져오기 성공, 실제 YOLO .pt 거부, 선택 구간 재생, JS 오류0 | `model_import_validation.json` |
| 원본 ns 표시 | 19자리 원본 ns를 API 문자열/JS BigInt로 정확히 표시; 실측 속도/바퀴각은 null | `browser_final_check.json` |

고유 자동시험은 백엔드·학습 13 + 기존 안전성 35 + 추가 추적 1 = **49개**다. 테스트 runner에서 import한 기존 unittest 클래스가 추가 추적 파일에서도 수집되므로 로그 13+35+36을 합산하면 중복 집계가 된다.

백엔드 시험은 지연·순서 변경·stamp 불일치·다른 clock·누락·중복·충돌 command ID·동일 프레임 재처리·반복 전송·teacher/실행 요청 불일치·부분 JSONL·metadata 없는 bag·영상 전용 bag·legacy exact stamp·legacy 명시 연결 보정·직접 매칭 수정 거부·수정 되돌리기·버전 불변·폴더 이동·origin/source 분할 누출 방지를 포함한다. 임의의 가까운 시각 매칭은 없다.

학습 시험은 신규 학습, checkpoint 저장/읽기, 파인튜닝, A/B 평가, 잘못된 YOLO 형식, 변경한 epochs로 resume 시도 거부를 포함한다. 실제 중단/재개는 브라우저 CPU 시험과 실제 RTX 4050 GPU 시험에서 모두 검증했다. GPU 재개 중 RNG state tensor가 CUDA로 이동하는 오류를 발견해 CPU ByteTensor로 복원하도록 고쳤다. CUDA 결정성 설정을 위해 CUBLAS_WORKSPACE_CONFIG도 저장 프로세스에서 설정한다. 시험으로 만들어진 모델은 모두 DEMO이며 실차 사용용 모델이 아니다.

## 최종 GUI 합성 시험의 측정값

`browser_validation.json`의 고정 데이터셋 `690f71f68ae76cce`: 독립된 **합성** 회차 3개, train/validation/test 각 18장. 3 epoch에서 중단해 last checkpoint를 저장한 뒤 optimizer 상태를 복원해 20 epoch까지 진행했다. augmentation brightness .1, batch6, seed42, CPU 2 threads, scheduler none, early stopping off로 기능 검증했다. 공식 PilotNet 신규 기본값을 그대로 학습한 실주행 결과는 아니다.

| 모델 | 합성 시험 MAE (command step) | 합성 RMSE | p50 / p95 (ms) |
|---|---:|---:|---:|
| 재개 후 20 epoch last | 0.2718 | 0.3126 | 0.806 / 1.076 |
| 중단 시점 3 epoch last | 0.8517 | 1.0657 | 0.786 / 0.980 |

조건: PyTorch 2.5.1+cu124의 CPU 실행, batch1, thread2, warmup5. 시간은 전처리+device copy+forward이며 디스크 image decode 제외다. 매 프레임 측정값도 원본 evaluation JSON에 있다. 작은 합성 영상의 smoke 결과를 실제 도로 추론지연·차선 유지 성능·주행 성공률로 해석하면 안 된다. 합성 회차 사이 패턴이 비슷하므로 일반화 근거도 아니다.

실제 CPU 교사 시험에서 합성 그림은 차선으로 인식되지 않았다(path_valid=false). 이때 원본 알고리즘대로 steering0/PWM255 요청을 생성했으나 격리된 검증 토픽에만 발행했고 sender는 없었다. 이는 source identity 전달 시험이다. 해당 프레임은 기본 dataset 필터에서 제외된다.

## 미검증·제약

- 실제 일반주행 bag이 제공되지 않아 원본·분리 코드의 **실주행 영상 동일 입력 비교**, 실제 라벨 품질·손실률·명령 지연은 미검증이다. 원본 reports의 과거 측정치는 현재 실측으로 재표시하지 않았다.
- USB 카메라/V4L2 실제 협상, 센서 노출 지연, 차량 저속 PWM, 조향 step→바퀴각, 시리얼 write→MCU 수신→차륜 응답, 실제 stop 거리·watchdog 종단 동작은 미검증이다. 보드 firmware 업로드 상태를 추측하지 않았다.
- 고정 PWM은 속도 폐루프 제어가 아니다. slip/배터리/노면 변화에 따른 m/s 일정성은 보장하지 않는다.
- 원본 reliable 영상 경로 대신 best-effort SourceFrame 경로를 사용하므로 실제 부하에서의 drop·성능 특성은 재계측해야 한다. 정확한 DDS queue overflow 횟수와 센서 내부 누락은 미계측이다. ID gap은 관측 가능한 증거다.
- recorder 정상 종료와 읽을 수 있는 sqlite 메시지 복구를 확인했다. 모든 종류의 디스크 손상·파일시스템 전원 장애 복구는 보장하지 않는다. partial은 운영자가 명시적으로 허용하기 전 dataset에서 제외된다.
- 장시간·대규모 실주행 데이터, MCAP 직접 import, 원격 업로드, 멀티유저 운영은 미검증/미제공이다. 현재 registry/session JSON은 세션별 index를 메모리에 읽으므로 매우 긴 세션의 페이지 단위 조회는 후속 확장 대상이다.
- 웹 시간 정렬은 source 직접 연결을 기본으로 한다. legacy 명시 재연결은 원본 ID 증거가 있는 직접 매칭과 같은 품질로 취급하지 않는다. 정상 주행 성공 여부는 path_valid만으로 결정하지 않는다.
- E2E 실차 폐루프, 자동 teacher/E2E 전환, 미션·장애물·신호등·주차·수동 운전 수집은 제공하지 않는다. 승인·STOP·통신 감시 앞에 향후 selector를 넣는 통합 계약만 문서화했다.

## 실차에서 필요한 자료

원본 교사로 주행한 새 schema2 세션 3회차 이상(가능하면 다른 시간/조건), 사용한 firmware와 실제 조향 캘리브레이션, 확인한 저속 PWM, 정상 종료 manifest 및 parameter snapshot, 운영자의 이탈/실패/정상 구간 검토가 필요하다. 확인 순서는 README 마지막 절에 있다. 실제 데이터가 생기면 같은 CLI와 GUI로 재검증할 수 있다.

## GPU 추가 검증

실제 호스트 RTX 4050 Laptop GPU, driver 595.99.02, torch 2.5.1+cu124, cuda:0에서 같은 고정 합성 데이터셋으로 학습했다. 2 epoch 완료 후 SIGTERM 중단, optimizer/scheduler 설정·CPU/CUDA RNG를 복원해 8 epoch까지 재개했다. best.pt와 last.pt를 저장하고 모델 파라미터가 실제 CUDA tensor인지 확인했다.

재개 모델의 합성 시험 MAE 0.4671 command step / RMSE 0.5668, GPU 추론 pipeline p50 0.612 ms / p95 0.877 ms였다. batch1, thread2, warmup5, 전처리+복사+forward 포함, 디스크 decode 제외, GPU synchronize로 완료를 기다려 측정했다. 이 수치는 작은 합성 영상의 기능 시험이다. 실제 주행 성능이나 장시간 GPU 처리량을 뜻하지 않는다. `gpu_validation.json`과 해당 output 폴더의 로그·checkpoint·evaluation.json을 보존했다. peak_allocated_bytes는 평가를 실행한 부모 프로세스의 peak이며 별도 학습 프로세스 전체 peak가 아니다.
