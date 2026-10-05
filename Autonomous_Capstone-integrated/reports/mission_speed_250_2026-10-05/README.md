# 미션 정상 속도 250 검증

패키지 기본값과 사용자 저장값의 정상 speed만 80에서 250으로 변경했다.
실제 mission_drive_tuning.launch.py를 speed 인자 없이 dry_run:=true, gui:=false, device:=cpu로 실행했다.
GUI는 실제 파라미터 서비스와 그리기 코드를 사용하고 창 관련 HighGUI 호출만 headless로 대체했다.
정상/회피/노랑/정지 명령 검증은 실제 launch 로딩 설정을 재사용하고 YOLO 검출만 합성 lane/box로 대체했다.
기존 BEV·경로·Stanley·미션 계산과 ROS 명령 발행을 사용했다.

- ROS speed, GUI 초기값/슬라이더: 250
- 정상 유효 차선: 좌우 PWM 250
- 회피: 80, 노랑: 60, 경로 장애물/빨강/빨강 이후 Unknown: 0
- 사용자 저장 파일이 없는 경우 패키지 기본값: 250
- 두 설정의 speed 외 값과 모델, 기존 기록 YAML 5개, 정지 정책 소스의 변경 없음 확인
- 모터 구동, 차량 시리얼 연결, 펌웨어 업로드 없음

loaded_parameters.yaml은 이번 검증의 새 snapshot이다. 기존 tuning_analysis 기록의 speed 80 snapshot은 수정하지 않았다.
실차 주행 성능을 검증한 결과가 아니다.
