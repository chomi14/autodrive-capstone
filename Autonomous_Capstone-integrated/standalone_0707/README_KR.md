# 0707 대회 주행 — Ubuntu 독립 실행

기준: LaneProject/0707/0707float.py. original/0707float.py에 사본 보존.
외부 LaneProject 원본과 기존 ROS 코드는 수정하지 않았습니다.
drive_0707_ubuntu.py만 실행합니다. ROS/colcon 빌드가 필요하지 않습니다.
기존 ROS 주행·serial_sender와 동시에 실행하지 마세요. 한 프로그램만 포트를 제어해야 합니다.

## 변경 범위

- edge 추출, 차선 중심점, CubicSpline, 기울기 조향, float 제한, 속도·차선 소실 처리는 원본 유지.
- gain=0.08, 차선 폭=280, 차량 중심=(315,170), 기본 PWM=250도 원본 값. Stanley로 바꾸지 않았습니다.
- 옮긴 ROI: TL/TR/BR/BL=(238,316)/(402,313)/(501,476)/(155,476). 상단 두 점에 y=-8 적용, CUTTING_IDX=300. 추가 src-y-offset은 기본 0.
- 모델은 원본에서 지정한 LaneProject/sangmin.pt를 동봉했습니다. SHA256: 69ccb253da4e65c136293b48d68dbd774caeb468062d934f832caad211d18398.
- Ubuntu 경로·카메라 장치·serial 호환과 SIGTERM 종료 정리를 추가했습니다.
- 경로 확인 전 직진하던 원본 launch burst(0.8초)는 기본 0으로 바꿨습니다. Enter 출발 대기 유지.
- BBOX 회피·LiDAR 연결 없음. 원본의 --enable-lane-change는 신호등 기반 전환이므로 주행만 할 때 넣지 마세요.

## 설치 / 화면 확인

    git fetch origin
    git switch sangmin_2
    git pull --ff-only origin sangmin_2
    cd Autonomous_Capstone-integrated/standalone_0707
    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install -r requirements.txt

모터 연결 없이 영상·ROI 확인 (준비 후 터미널 Enter):

    python drive_0707_ubuntu.py --camera /dev/video2 --device cpu --preview --no-serial

Ubuntu 데스크톱 GUI가 필요합니다. CUDA 지원 PyTorch 환경이면 --device 0 사용 가능.
카메라 장치는 실제 차량에 맞추세요. 숫자 --camera 2도 가능합니다.

## 현재 Arduino 펌웨어로 주행

현재 저장소 펌웨어는 정수 조향 / 115200입니다. 기본 integer 모드는 내부 float 계산을
유지하고 전송 시에만 반올림합니다. 정지 시 X 명령을 전송합니다.
이 독립 프로그램은 ROS calibration/READY/heartbeat 게이트를 사용하지 않습니다.
Enter 출발 대기, q/ESC/Ctrl+C 종료를 사용합니다. Arduino 배선·보정값은 자동 변경하지 않습니다.

먼저 바퀴를 띄워 방향·정지를 확인하고 기본 250보다 낮은 PWM으로 시험하세요:

    python drive_0707_ubuntu.py --camera /dev/video2 --port /dev/arduino --device cpu --preview --normal-speed 80

실제 포트가 /dev/ttyACM0이면 --port 값을 바꾸세요. 포트 권한이 없으면 Ubuntu 계정을
dialout 그룹에 추가하고 다시 로그인하세요. 다른 serial 프로그램은 모두 종료해야 합니다.

## 예전 0707 float 펌웨어를 그대로 쓸 경우에만

    python drive_0707_ubuntu.py --serial-format float --camera /dev/video2 --port /dev/ttyACM0 --preview --normal-speed 80

원본의 소수 조향 메시지 / 기본 9600 사용. 현재 정수 펌웨어에는 이 모드를 쓰지 마세요.
펌웨어 자동 업로드나 배선·조향 보정값 변경은 하지 않습니다.

## 파라미터 / 검사

ROI·주행 기본값은 실행 파일 상단, 나머지는 --help에 있습니다.
알고리즘 함수 AST 동일성, 같은 ROI의 합성 차선 경로·조향, ROI 값, serial 형식·가짜 포트 검사는:

    python test_0707_port.py

실제 카메라·모터 주행과 Ubuntu CUDA 추론 성능은 별도 검증이 필요합니다.
