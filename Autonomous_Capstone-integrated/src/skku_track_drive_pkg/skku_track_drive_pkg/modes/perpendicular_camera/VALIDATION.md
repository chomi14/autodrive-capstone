# 구현 및 검증 기록 — 2026-11-13

## 차량 노트북 적용 검증 (`1113`)

`camera-perpendicular-parking-1030.zip`의 패치를 기준 커밋에 적용했다.
아래 원본 기록은 압축파일 작성 환경의 결과이며, 노트북에서 추가 확인한 결과는 다음과 같다.

- ROS 2 Humble/Python 3.10에서 `colcon build --symlink-install --packages-up-to vehicle_bringup_pkg`: 7개 패키지 성공.
- README의 회귀검증을 ROS 환경을 불러온 뒤 실행: **162 passed**, skip 없음.
  여기에는 실제 ROS launch 객체 조립 테스트 8개가 포함된다. 센서·제어 프로세스를 실행하는 검증은 아니다.
- 설치된 preview/live launch의 `--show-args` 및 신규 console executable 3개 등록 확인.
- `git diff --check` 통과. 기존 주행·장애물회피·평행주차 제어 소스 변경 없음.
- 시스템 SciPy가 NumPy 1.26.4에 대해 버전 경고 1건을 출력했으며 테스트 실패는 없었다.
- 주차 전용 모델 추론·카메라 연결·BEV 보정·실차 주차는 미수행.

## 압축파일 원본 검증 기록

- 대상 기준: `56395e0c9930bf88bf616c061b86faf0ebff1d65` (`1030`).
- 참고 기준: `9c657d6007354e9bad319eb2f55f44d75a1dc43c` (`4th_skku_contest`).
- 작업 브랜치: `camera-perpendicular-parking`.
- 환경: Linux, Python 3.12, NumPy 2.3.5, OpenCV 5.0.0, pytest 9.2.0.
  이는 검증 컨테이너 환경이며 차량 노트북의 ROS Humble/Python 3.10 환경이 아니다.

README의 오프라인 검증 명령 결과: **154 passed, 1 skipped**.
외부 pytest plugin의 폐기 예정 API warning 한 건이 있으며 테스트 실패는 없다.
skip은 ROS가 없는 환경에서의 `test_ros_launch.py` 모듈이다.

검증한 범위:

- 정상 주차, 선택적 출차, 종료선 감지 주기의 즉시 정지.
- 초기 미검출, 검출 순서·노이즈, 같은 프레임 반복, state/W 이후 capture barrier.
- disarm 중 타이머 정지, 빠른 S/W, stale/invalid 관측과 시계 역행.
- 기동 timeout, 관측 상실 watchdog, fault/reset.
- 후진 중심 오차의 조향 부호, pi 주기 각도, deadband·반올림·포화.
- 여러 영상 해상도의 정규화 좌표, 양측 영상선 중심, 한쪽 선 보정, 빈 ROI.
- 클래스 이름과 실제 ID의 동적 대응, 박스 전용 출력 거부, 누락 모델 오류.
- 카메라 YAML 검증·전체 설정 저장/로드.
- 실제 adapter 메서드의 in-memory 입력·출력, 실제 launch 조립 함수의
  inert action double 기반 모드 선택·preview 토픽/서비스 격리.
- 기존 순수 Python 미션, 시간 기반/평행주차, 차선 처리, bbox 회피,
  BEV/ROI, LiDAR-camera geometry 회귀 테스트.
- 신규 Python 소스 컴파일, 패키지 검색 경로, XML 파싱 및 `git diff --check`.

실행하지 못한 범위:

- colcon build 및 실제 launch_ros/rclpy/DDS 통신.
- 실제 주차 모델의 load/export/inference, 학습 성능·현장 검출 정확도.
- 전·후방 USB 카메라 동시 연결, GPU 메모리, 프레임률과 지연 예산.
- 실제 카메라 BEV·바퀴 조향 부호·PWM/속도·범퍼 정지 위치.
- 실제 주차·출차 및 후방 충돌 감지.

기존 가중치는 pickle을 실행하지 않고 checkpoint 메타데이터 문자열을 확인했다.
best_v2/best_v3는 lane2/traffic_light, best_m은 lane1/lane2/obstacle/traffic_light가
확인되며 이들에 주차 전용 클래스가 있다고 가정하지 않는다.
외부 주차 가중치나 현장 영상을 만들어 사용한 것으로 표시하지 않았다.

기존 track/mission/parallel/legacy perpendicular 제어 소스와 sensor/serial/firmware는
수정하지 않았다. 기존 파일 수정은 루트 실행 안내, 새 console entry point 등록,
명시적 NumPy/OpenCV 의존성 등록이다. 나머지는 전용 폴더·설정·launch·테스트 추가다.
원격 push/merge 및 실제 모터 구동은 수행하지 않았다.
