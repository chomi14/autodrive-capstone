"""Separate camera-only bbox obstacle mission; reuses the common mission GUI."""
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
from vehicle_bringup_pkg.tuning_configuration import tuning_launch_arguments, resolve_tuning

# 사용자가 자주 바꿀 시작값. 실행 시 speed:=값으로 덮어쓸 수 있습니다.
TEST_SPEED = '30'  # 첫 실차 시험 PWM
BBOX_DEFAULTS = {
    'obstacle_near_y': 300,         # 480px 높이 기준 bbox 하단 근접선
    'path_margin_px': 35,          # 640px 폭 기준 경로/bbox 좌우 여유
    'obstacle_confirm_frames': 3,  # 회피 확정 연속 프레임(확인 중 정지)
    'obstacle_clear_frames': 5,    # 경로가 비었다고 판단할 연속 프레임
    'alternate_min_area_px': 250,  # 대체 차선 최소 mask 면적
    'avoid_hold_s': 2.0,           # 차선 변경 이후 최소 회피 유지 시간
    'avoid_speed': 80,            # 회피 중 PWM 상한
}
BBOX_TUNING_PATH = str(
    Path.home() / '.config/autodrive/mission_bbox_only_tuning.yaml'
)  # 이 모드 전용 GUI P 저장 파일


def bbox_assemble(context):
    actions = assemble(context, 'mission')
    index = next(
        i for i, action in enumerate(actions)
        if isinstance(action, Node) and action.node_executable == 'mission_controller_node'
    )
    settings, _, _, source = resolve_tuning(context, 'mission')
    # 저장/명시 YAML 및 명시 launch 인자가 코드 기본값보다 우선합니다.
    if source == 'package default':
        for name, value in BBOX_DEFAULTS.items():
            if not LaunchConfiguration(name).perform(context).strip():
                settings[name] = value
    settings.update({
        'model_path': LaunchConfiguration('model_path'), 'device': LaunchConfiguration('device'),
        'image_topic': '/mission/image_raw', 'debug_topic': '/mission/debug_image',
        'image_reliability': 'reliable', 'publish_debug': True, 'publish_bev_debug': True,
        'publish_analysis': analysis_enabled(context), 'start_enabled': True,
        'max_steering': 7.0, 'allow_speed_tuning': True, 'cmd_topic': command_topic(context),
        'steering_sign': ParameterValue(LaunchConfiguration('steering_sign'), value_type=float),
    })
    actions[index] = Node(
        package='skku_track_drive_pkg',
        executable='bbox_mission_controller_node',
        name='mission_controller_node',
        output='screen',
        parameters=[settings],
        remappings=preview_remappings(context),
    )
    return actions


def generate_launch_description():
    model = str(
        Path(get_package_share_directory('skku_track_drive_pkg'))
        / 'models/mission/best.pt'
    )
    tuning = [
        argument for argument in tuning_launch_arguments('mission')
        if argument.name not in ('speed', 'saved_tuning_path')
    ]
    return LaunchDescription([
        config_argument(),
        *common_arguments(camera=True),
        *perception_arguments('mission'),
        *tuning,
        DeclareLaunchArgument('speed', default_value=TEST_SPEED),
        DeclareLaunchArgument('saved_tuning_path', default_value=BBOX_TUNING_PATH),
        DeclareLaunchArgument('model_path', default_value=model),
        DeclareLaunchArgument('steering_sign', default_value='1.0'),
        OpaqueFunction(function=bbox_assemble),
    ])
