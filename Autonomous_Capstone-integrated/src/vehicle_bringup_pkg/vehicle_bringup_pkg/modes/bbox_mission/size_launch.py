"""Compatibility launch: immediate lane-mask/bbox-bottom switching; no LiDAR."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 과거 면적 회피 실행 이름/저장 경로 유지. 현재는 bbox_only와 동일한 Y 기준 전환.
# 실제 회피에 쓰는 값은 initial_target_lane, obstacle_near_y입니다.
# 면적/확인 프레임/회피 속도/쿨다운은 과거 설정 호환용이며 적용하지 않습니다.
# 현재 자동 전환 쿨다운은 mission_core.py의 5초 고정값입니다. L로 즉시 수동 반전.
SIZE_DEFAULTS = {
    'initial_target_lane': '2',         # 출발 차선: 1 또는 2. 현재 차선의 반대로 전환.
    'obstacle_trigger_mode': '1',       # 1: 면적 기준 / 0: 기존 하단 기준
    'obstacle_min_area_ratio': '0.02',  # 화면의 2% (640x480에서 6144px²)
    'obstacle_near_y': '300',           # 기존 하단 근접 조건 유지. 480px 기준.
    'obstacle_confirm_frames': '1',     # 면적 모드는 첫 프레임에 전환 요청
    'avoid_speed': '250',               # 회피 PWM 상한
    'lane_change_cooldown_s': '2.0',     # 책의 쿨다운 구조. 최초 전환은 대기 없음.
    'load_saved_tuning': 'false',       # 예전 bbox 저장값의 자동 개입 방지
    'saved_tuning_path': str(Path.home() / '.config/autodrive/mission_bbox_size_tuning.yaml'),
}


def generate_launch_description():
    base = Path(get_package_share_directory('vehicle_bringup_pkg')) / 'launch/mission_bbox_only.launch.py'
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in SIZE_DEFAULTS.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(base)),
            launch_arguments={name: LaunchConfiguration(name) for name in SIZE_DEFAULTS}.items(),
        ),
    ])
