"""Parking controller and drive-gate diagnostics for the viewer."""

REASONS = {
    'ACTIVE': '주행 중',
    'AWAITING_START': 'W 입력 대기',
    'DISARMED': '구동 허가 해제',
    'OPERATOR_STOP': 'S 입력으로 정지',
    'RESET': 'R 입력으로 주차 상태 초기화',
    'TUNING_APPLIED': '설정 적용으로 주차 상태 초기화',
    'WAIT_BOTH_PERCEPTION_PIPELINES': '전·후방 인식 초기 관측 대기',
    'SAVE_AND_RESTART_FOR_CAMERA_CAPTURE_SETTINGS': '카메라 설정 저장 및 재시작 대기',
    'CALIBRATION_NOT_CONFIRMED': '카메라·조향 보정 확인 대기',
    'FRONT_BOUNDARY_MISSING': '탐색용 전방 경계선 미검출',
    'FRONT_LIMIT_REACHED': '전방 평행주차선 접근 — 후진 전환을 위해 정지',
    'DIRECTION_PAUSE': '전진→후진 전환 대기',
    'DIRECTION_PAUSE_COMPLETE': '전진→후진 전환 대기 완료',
    'REAR_PARKING_GEOMETRY_MISSING': '후방 칸 중심선 또는 정렬 구간의 종료선 미검출',
    'REAR_PARKING_COMMAND_HELD': '후방 추종선 미검출 — 이전 조향·PWM 유지',
    'REAR_END_REACHED': '후방 종료선 정지 기준 도달',
    'PARKED': '주차 완료 — 정지 유지',
    'PARKED_HOLD_COMPLETE': '주차 정지 유지 시간 완료',
    'ESCAPE_COMPLETE': '출차 직진 시간 완료',
    'EXIT_LINE_MISSING': '전방 출차선 미검출',
    'EXIT_LINE_REACHED': '전방 출차선 기준 도달',
    'FINAL_STRAIGHT_COMPLETE': '마지막 직진 시간 완료 — 출차 종료',
    'FINISHED': '출차 완료',
    'ROS_CLOCK_MOVED_BACKWARD_RESET_REQUIRED': 'ROS 시계 역행 — 초기화 필요',
    'UNKNOWN_STATE': '알 수 없는 주차 상태',
}

GATE_REASONS = {
    'startup': 'W 입력 및 구동 준비 대기',
    'operator/UI disarm': 'S 입력으로 정지',
    'serial fault': '시리얼 통신 오류',
    'calibration failed': '구동 장치 보정 실패',
    'controller shutdown': '제어기 종료',
    'controller process replaced': '제어기 프로세스 교체',
    'controller health timeout': '제어기 heartbeat 시간 초과',
    'control command timeout': '제어 명령 수신 시간 초과',
    'UI heartbeat timeout': 'GUI heartbeat 시간 초과',
    'UI heartbeat timeout during calibration': '보정 중 GUI heartbeat 시간 초과',
    'serial transmission gap exceeded firmware watchdog; fresh W required': '시리얼 명령 전송 지연 — W 입력 필요',
}

CAMERA_PROBLEMS = {
    'INPUT_MISSING': '영상 입력 없음',
    'INPUT_STALE': '영상 지연',
    'OBSERVATION_INVALID': '영상 인식 실패',
    'WAIT_FRESH': '새 영상 대기',
}


def reason_text(reason):
    code, _, detail = reason.partition(': ')
    if code == 'SPACE_MISSING':
        return f'전방 주차칸 연속 {detail}회 미검출'
    if code == 'FRONT_LIMIT_FALLBACK':
        duration = f'{detail}초' if detail else '설정 시간'
        return f'전방 평행주차선 미검출 — 좌회전 전진 {duration} 경과로 정지'
    if code == 'RIGHT_REVERSE_COMPLETE':
        return f'고정 우조향 후진 {detail}초 완료'
    if code == 'STATE_TIMEOUT':
        return f'상태 제한 시간 초과 ({detail})'
    if code == 'OBSERVATION_LOSS_TIMEOUT':
        return f'관측 상실 제한 시간 초과 — {reason_text(detail)}'
    camera, _, problem = code.partition('_')
    if camera in ('front', 'rear') and problem in CAMERA_PROBLEMS:
        label = '전방' if camera == 'front' else '후방'
        return f'{label} {CAMERA_PROBLEMS[problem]}'
    return REASONS.get(reason, reason)


def gate_reason_text(reason):
    if reason.startswith('perception stale: '):
        return '구동 게이트의 인식 지연 감지 — ' + reason.removeprefix('perception stale: ')
    return GATE_REASONS.get(reason, f'구동 게이트 정지 ({reason})')
