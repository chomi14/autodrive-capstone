# 0708 방식: obstacle bbox 위치로 반대 차선 전환

0708_250.py의 신호등 bbox 전환 조건을 장애물에 적용한 별도 독립 실행 파일입니다.
ROS launch가 아닙니다. 기존 원본은 original/0708_250.py로 보존했고 외부 LaneProject 원본은 수정하지 않았습니다.

## 정확한 전환 원리

- obstacle / obstacles 클래스 모두 지원. 원래 신호등 검출 함수처럼 첫 장애물 bbox를 사용합니다.
- lane2 주행 중 bbox 윗변이 10 < y < 100이면 lane1로 변경.
- lane1 주행 중 10 < y < 150이고 bbox 중심 x > 185이면 lane2로 변경.
- 좌표 기준 640x480. 다른 영상 크기면 해당 기준으로 환산합니다.
- 마지막 변경 후 10초를 초과해야 다시 변경. 변경 후 기존 20초 감속 흐름 유지.
- 이 버전의 회피는 기본 활성. --disable-obstacle-avoidance로 끌 수 있습니다.
- 기존 신호등 기반 차선 변경은 이 새 파일에서는 사용하지 않습니다.

이는 bbox 면적/하단 기준이 아닙니다. 특정 위치에 장애물이 보이면 target_lane만 반대로 바꿉니다.
현재 경로와의 겹침 또는 반대 차선의 빈 공간을 확인하는 sangminv2의 면적 모드와 다릅니다.
원본과 동일하게 장애물이 계속 보이면 쿨다운 후 다시 반대 차선으로 바뀔 수 있습니다.
따라서 주행 가능한 공간을 보증하는 안전한 회피 계획기로 간주하면 안 됩니다.

## 원본 대비 다른 부분

- 좌표 조건·쿨다운·CubicSpline·조향 계산·기본 주행/회피 속도는 유지했습니다.
- 앞서 확인한 버그만 새 복사본에서 수정: 빈 ROI의 가짜 목표점 생성 방지,
  가상 중심의 좌표 제한을 차선 폭 대신 영상 폭으로 적용.
  유효 목표점이 부족하면 원본의 기존 차선 소실 처리를 사용합니다. 새로운 정지 상태를 추가하지 않았습니다.
- 시작 전 무조건 직진하던 launch burst는 기본 0. Enter 출발 대기는 유지.
- 기본 포트는 Windows COM4 / Ubuntu /dev/arduino, baud=115200.
- 기본은 float 전송입니다. 원래 9600 펌웨어는 --baudrate 9600으로 별도 지정해야 합니다.
- 모델은 저장소 models/mission/best.pt를 기본 사용. --model로 기존 가중치 지정 가능.
- 설정은 실행 파일 상단. 기존 0713에서 변경한 ROI·조향 부호를 이 파일이 자동으로 읽지는 않습니다.

## 설치 / 실행

저장소 루트에서:

    git switch sangmin
    git pull --ff-only origin sangmin
    cd Autonomous_Capstone-integrated/standalone_0708
    python -m pip install -r requirements.txt

Windows, 모터 없이 카메라/회피 판단 먼저 확인:

    py -3.9 .\0708_bbox_obstacle.py --preview --no-serial

Windows 실제 float 펌웨어 (보드 업로드값/방향 확인 후):

    py -3.9 .\0708_bbox_obstacle.py --preview --port COM4 --normal-speed 80 --low-speed 80 --lane-miss-speed 80

Ubuntu의 ROS 정수 펌웨어:

    python3 0708_bbox_obstacle.py --preview --camera 0 --port /dev/arduino --serial-format integer --normal-speed 80 --low-speed 80 --lane-miss-speed 80

현재 펌웨어가 float이면 integer 옵션을 빼세요. ROS 주행 프로그램/시리얼 모니터와 동시에 실행하지 마세요.
카메라 번호와 포트는 연결된 장치에 맞춰야 합니다. Enter 후 주행 시작, q/ESC/Ctrl+C 종료.
현재 차선에서 출발하도록 --target-lane lane1 또는 lane2를 지정하세요.

기본 속도는 원본처럼 250이고 회피/차선 소실 속도는 150입니다. 초기 시험에는 위처럼 낮은 속도를 명시하세요.
앞바퀴/뒷바퀴를 띄워 방향·정지·프로토콜부터 확인해야 합니다. 펌웨어의 통신 끊김 정지가 꺼져 있으면
프로그램 종료/연결 끊김만으로 차량이 멈춘다고 가정하지 마세요.

## 검사

    python test_0708_bbox_obstacle.py

원본 신호등 조건과 새 장애물 조건의 동등성, 양방향 전환/쿨다운/해상도,
빈 ROI 처리, 경로·조향 비교, 두 serial 형식을 장치 없이 검사합니다.
실제 카메라·차량 주행 검증은 별도로 필요합니다.
