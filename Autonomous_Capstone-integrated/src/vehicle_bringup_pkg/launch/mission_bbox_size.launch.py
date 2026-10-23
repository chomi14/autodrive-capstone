"""Latest 1010 track pipeline + opt-in bbox-area lane change; no LiDAR."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ===== EDIT HERE: 면적 회피 기본값 (나머지 주행값은 기존 bbox/track 설정 유지) =====
SIZE_DEFAULTS = {
    'initial_target_lane': '2',         # 출발 차선: 1 또는 2. 현재 차선의 반대로 전환.
    'obstacle_trigger_mode': '1',       # 1: 면적 기준 / 0: 기존 하단 기준
    'obstacle_min_area_ratio': '0.02',  # 화면의 2% (640x480에서 6144px²)
    'obstacle_near_y': '300',           # 기존 하단 근접 조건 유지. 480px 기준.
    'obstacle_confirm_frames': '1',     # 면적 모드는 첫 프레임에 전환 요청
    'avoid_speed': '80',                # 회피 PWM 상한 (기본 주행보다 올리지 않음)
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
