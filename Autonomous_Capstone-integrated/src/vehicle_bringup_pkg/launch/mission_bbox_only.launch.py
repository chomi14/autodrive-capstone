"""Camera-only bbox mission: edit the parameter blocks at the top."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vehicle_bringup_pkg.configuration import config_argument
from vehicle_bringup_pkg.mode_launch import common_arguments, command_topic, preview_remappings
from vehicle_bringup_pkg.recording import analysis_enabled
from vehicle_bringup_pkg.perception_policy import perception_arguments
from vehicle_bringup_pkg.tuned_modes import assemble
from vehicle_bringup_pkg.tuning_configuration import tuning_launch_arguments, resolve_tuning, _profile, _validate_parameter_file, ALLOWED_TUNING_PARAMETERS

# =========================
# EDIT HERE: 실행 / 장치 / 저장
# =========================
DEFAULT_MODEL = ''                  # 빈 값: models/mission/best.pt 자동 선택
DEFAULT_CAMERA = '/dev/video0'       # 실제 전방 카메라 장치
DEFAULT_PORT = '/dev/arduino'        # Arduino serial 장치
DEFAULT_BAUDRATE = 115200            # 기존 차량 펌웨어 통신 속도
DEFAULT_DEVICE = 'cuda:0'            # GPU 없으면 'cpu'
DEFAULT_STEERING_SIGN = 1.0          # 조향 반전 시 -1.0
DEFAULT_INITIAL_LANE = 2             # 시작 차선: 1 또는 2
DEFAULT_AUTO_CALIBRATE = False       # 출발 시 조향 endpoint 자동 보정
DEFAULT_CALIBRATION_TOLERANCE = 35    # 조향 보정 허용 ADC 오차
DEFAULT_GUI = True                  # Mission Tuner 표시
DEFAULT_SENSORS_ONLY = False         # True: 실제 카메라/GUI만, 모터 연결 제외
DEFAULT_DRY_RUN = False              # True: 실제 센서/모터 모두 제외
LOAD_SAVED_TUNING = False            # True: 이전 GUI P 저장값 자동 적용
BBOX_TUNING_PATH = str(Path.home() / '.config/autodrive/mission_bbox_only_tuning.yaml')
DEFAULT_TUNING_CONFIG = ''           # 명시 YAML 경로: 코드 상단값보다 우선
DEFAULT_RECORD_DIR = ''              # 빈 값: rosbag 기록 안 함
LOAD_SAVED_TRACK_TUNING = True       # 현재 트랙 GUI 저장값을 기본 주행에 사용
TRACK_TUNING_PATH = str(Path.home() / '.config/autodrive/track_tuning.yaml')
DEFAULT_ANALYSIS = False             # 주행 분석 토픽 발행

# =========================
# EDIT HERE: 카메라 / 차선 인식
# =========================
CAMERA_DEFAULTS = {
    'width': 640,                   # 영상 폭: BEV 좌표와 함께 보정
    'height': 480,                  # 영상 높이: 기본 기준 640x480
    'fps': 30.0,                    # 요청 fps
    'fourcc': 'MJPG',               # 카메라 입력 형식
    'buffer_size': 1,               # 오래된 프레임 최소화
    'reopen_after_failures': 2,      # 연속 실패 후 카메라 재연결
    'disable_dynamic_framerate': True,  # 자동 fps 변경 방지
    'reliability': 'reliable',      # 카메라/controller 영상 QoS
    'show': False,                  # 원본 카메라 창(Debug 창은 별도)
}
VISION_DEFAULTS = {
    'confidence': 0.5,              # YOLO 최소 검출 확률
    'bev_top_shift': -8,           # BEV 상단 두 점 y 이동(px)
    'roi_cut': 300,                 # BEV 상단 잘라낼 높이(px)
    'look_shift': 50,                # 중심점 샘플 높이 이동(px)
    'ema_alpha': 0.6,               # 기울기 현재 관측 가중치(0~1)
    'virtual_lane_width': 300,      # 가려진 차선 복원용 가상 폭(px)
    'bev_pad': 250,                 # BEV 시각화 좌우 padding(px)
    'center_ema_alpha': 0.35,       # 중심점 현재 관측 가중치(0~1)
    'max_center_jump_px': 80.0,     # 프레임 간 중심점 최대 이동(px)
    'max_missed_frames': 12,        # 차선 미검출 시 기존 경로 유지 프레임
    'min_component_area': 250,     # 차선 잡음 제거 최소 면적(px)
}
SRC_MAT_BASE = [[238, 316], [402, 313], [501, 476], [155, 476]]
# BEV 원본 사다리꼴: 좌상 / 우상 / 우하 / 좌하. 카메라 장착에 맞춰 보정.

# =========================
# EDIT HERE: 기본 주행 / Stanley 조향
# =========================
DRIVING_DEFAULTS = {
    'speed': 250,                    # 기본 좌우 모터 PWM(0~255)
    'stanley_gain': 0.020,          # 횡방향 오차 gain
    'heading_gain': 0.60,           # 방향 오차 gain
    'stanley_softening': 0.001,     # Stanley 분모 안정화 값
    'lookahead_index': 10,          # 경로 목표점 인덱스
    'heading_step': 3,             # 방향 계산 점 간격
    'car_center_x': 325.0,          # 차량 기준점 x(px)
    'car_center_y': 179.0,          # 차량 기준점 y(px)
    'max_steering_angle': 50.0,     # 조향 명령 변환 최대각(deg)
}
MAX_STEERING_COMMAND = 7.0          # 펌웨어 조향 명령 최대값
ALLOW_SPEED_TUNING = True           # GUI PWM 변경 허용
START_ENABLED = True               # 계산 활성화; 실제 출발은 기존 W gate

# =========================
# EDIT HERE: bbox 장애물 회피
# =========================
BBOX_DEFAULTS = {
    'obstacle_near_y': 300,         # bbox 하단 근접선(480px 기준)
    'path_margin_px': 35,          # 경로/bbox 좌우 여유(640px 기준)
    'obstacle_confirm_frames': 3,  # 회피 확정 연속 프레임(확인 중 정지)
    'obstacle_clear_frames': 5,    # 장애물 해제 연속 프레임
    'alternate_min_area_px': 250,  # 대체 차선 최소 mask 면적
    'avoid_hold_s': 2.0,           # 최소 회피 유지시간(s)
    'avoid_speed': 80,            # 회피 PWM 상한(기본 속도를 올리지 않음)
    'image_timeout_s': 0.75,       # 영상 지연 표시(s); 정지 기준은 아래 별도
}

# =========================
# EDIT HERE: 신호등 / HSV
# =========================
TRAFFIC_DEFAULTS = {
    'yellow_speed': 60,            # 노란불 PWM 상한
    'red_confirm_frames': 3,       # 빨간불 확정 연속 프레임
    'green_confirm_frames': 3,     # 초록불 확정 연속 프레임
    'wait_for_green': 0,           # 1: 최초 출발도 초록불까지 대기
    'traffic_min_width': 20,       # 신호등 최소 bbox 폭(640px 기준)
    'traffic_min_bottom_y': 80,    # 신호등 최소 bbox 하단 y(480px 기준)
    'traffic_min_color_ratio': 0.02, # bbox 내부 색상 증거 최소 비율
    'hsv_s_min': 100,              # HSV 최소 채도
    'hsv_v_min': 95,               # HSV 최소 밝기
    'red_h_low_max': 10,           # 빨강 낮은 hue 구간 상한
    'red_h_high_min': 160,         # 빨강 높은 hue 구간 하한
    'green_h_min': 40,             # 초록 hue 하한
    'green_h_max': 90,             # 초록 hue 상한
    'yellow_h_min': 20,            # 노랑 hue 하한
    'yellow_h_max': 30,            # 노랑 hue 상한
}

# =========================
# EDIT HERE: 상태 / 지연 / 디버그
# =========================
DEFAULT_COMMAND_TIMEOUT = 0.0      # controller 생존 lease(s)
DEFAULT_UI_TIMEOUT = 0.75           # GUI heartbeat 제한(s)
DEFAULT_PERCEPTION_STOP = 6.0       # 인지 결과 무응답 X/disarm 기준(s)
NODE_DEFAULTS = {
    'publish_debug': True,         # 경로/bbox/신호등 debug 영상
    'publish_bev_debug': True,     # BEV 영상
    'debug_log': False,            # 주행 console 로그
    'debug_log_interval': 10,      # 로그 프레임 간격
    'profile': False,              # 추론시간 측정
    'profile_warmup_frames': 10,   # 측정에서 제외할 초기 프레임
    'profile_report_interval': 50, # 측정 통계 출력 간격
    'profile_input_fps': 0.0,      # 측정용 예상 입력 fps(0: 미지정)
}

# 아래는 실행 연결 코드입니다.
BEV_DEFAULTS = {
    f'bev_src_{corner}_{axis}': point[index]
    for corner, point in zip(('tl', 'tr', 'br', 'bl'), SRC_MAT_BASE)
    for index, axis in enumerate(('x', 'y'))
}
TUNING_DEFAULTS = {**VISION_DEFAULTS, **DRIVING_DEFAULTS, **BBOX_DEFAULTS, **TRAFFIC_DEFAULTS, **BEV_DEFAULTS}


def resolve_bbox_settings(context):
    settings, _, _, source = resolve_tuning(context, 'mission')
    use_saved = LaunchConfiguration('load_saved_tuning').perform(context).lower() in ('true', '1')
    if source == 'package default' or (source == 'saved user' and not use_saved):
        settings.update(TUNING_DEFAULTS)
        use_track = LaunchConfiguration('load_saved_track_tuning').perform(context).lower() in ('true', '1')
        track_path = Path(LaunchConfiguration('track_tuning_path').perform(context)).expanduser()
        if use_track and track_path.is_file():
            # Read only: do not overwrite the current track calibration file.
            settings.update(_validate_parameter_file(track_path, 'track_controller_node', ALLOWED_TUNING_PARAMETERS))
    _, _, types, _ = _profile('mission')
    for name, default in TUNING_DEFAULTS.items():
        raw = LaunchConfiguration(name).perform(context).strip()
        if raw:
            settings[name] = types.get(name, type(default))(raw)
    return settings


def bbox_assemble(context):
    actions = assemble(context, 'mission')
    index = next(i for i, a in enumerate(actions)
                 if isinstance(a, Node) and a.node_executable == 'mission_controller_node')
    settings = resolve_bbox_settings(context)
    for name, default in NODE_DEFAULTS.items():
        raw = LaunchConfiguration(name).perform(context)
        if isinstance(default, bool):
            if raw.lower() not in ('true', 'false', '1', '0'):
                raise ValueError(f'{name}: expected true or false')
            settings[name] = raw.lower() in ('true', '1')
        else:
            settings[name] = type(default)(raw)
    settings.update({
        'model_path': LaunchConfiguration('model_path'), 'device': LaunchConfiguration('device'),
        'image_topic': '/mission/image_raw', 'debug_topic': '/mission/debug_image',
        'image_reliability': CAMERA_DEFAULTS['reliability'],
        'publish_analysis': analysis_enabled(context), 'start_enabled': START_ENABLED,
        'max_steering': MAX_STEERING_COMMAND, 'allow_speed_tuning': ALLOW_SPEED_TUNING,
        'cmd_topic': command_topic(context),
        'steering_sign': ParameterValue(LaunchConfiguration('steering_sign'), value_type=float),
        'initial_target_lane': DEFAULT_INITIAL_LANE,
    })
    actions[index] = Node(
        package='skku_track_drive_pkg', executable='bbox_mission_controller_node',
        name='mission_controller_node', output='screen',
        parameters=[settings], remappings=preview_remappings(context),
    )
    for i, a in enumerate(actions):
        if isinstance(a, Node) and a.node_executable == 'camera_publisher_node':
            actions[i] = Node(
                package='sensor_bringup_pkg', executable='camera_publisher_node',
                name='camera_publisher_node', output='screen',
                parameters=[{**CAMERA_DEFAULTS, 'device': LaunchConfiguration('camera_device'),
                             'topic': '/mission/image_raw'}],
            )
    return actions


def generate_launch_description():
    model = DEFAULT_MODEL or str(
        Path(get_package_share_directory('skku_track_drive_pkg')) / 'models/mission/best.pt')
    overrides = {
        'camera_device': DEFAULT_CAMERA, 'arduino_port': DEFAULT_PORT,
        'arduino_baud': DEFAULT_BAUDRATE, 'device': DEFAULT_DEVICE,
        'auto_calibrate': DEFAULT_AUTO_CALIBRATE, 'calibration_tolerance': DEFAULT_CALIBRATION_TOLERANCE,
        'gui': DEFAULT_GUI, 'sensors_only': DEFAULT_SENSORS_ONLY, 'dry_run': DEFAULT_DRY_RUN,
        'command_timeout': DEFAULT_COMMAND_TIMEOUT, 'ui_timeout': DEFAULT_UI_TIMEOUT,
        'perception_stop_s': DEFAULT_PERCEPTION_STOP,
        'record_dir': DEFAULT_RECORD_DIR, 'analysis': DEFAULT_ANALYSIS,
        'saved_tuning_path': BBOX_TUNING_PATH, 'tuning_config': DEFAULT_TUNING_CONFIG,
    }
    arguments, declared = [], set()
    for a in [*common_arguments(camera=True), *perception_arguments('mission'), *tuning_launch_arguments('mission')]:
        declared.add(a.name)
        if a.name in overrides:
            value = overrides[a.name]
            a = DeclareLaunchArgument(a.name, default_value=str(value).lower() if isinstance(value, bool) else str(value))
        arguments.append(a)
    for name in sorted(TUNING_DEFAULTS.keys() - declared):
        arguments.append(DeclareLaunchArgument(name, default_value=''))
    for name, value in NODE_DEFAULTS.items():
        arguments.append(DeclareLaunchArgument(name, default_value=str(value).lower() if isinstance(value, bool) else str(value)))
    return LaunchDescription([
        config_argument(), *arguments,
        DeclareLaunchArgument('load_saved_tuning', default_value=str(LOAD_SAVED_TUNING).lower()),
        DeclareLaunchArgument('load_saved_track_tuning', default_value=str(LOAD_SAVED_TRACK_TUNING).lower()),
        DeclareLaunchArgument('track_tuning_path', default_value=TRACK_TUNING_PATH),
        DeclareLaunchArgument('model_path', default_value=model),
        DeclareLaunchArgument('steering_sign', default_value=str(DEFAULT_STEERING_SIGN)),
        OpaqueFunction(function=bbox_assemble),
    ])
