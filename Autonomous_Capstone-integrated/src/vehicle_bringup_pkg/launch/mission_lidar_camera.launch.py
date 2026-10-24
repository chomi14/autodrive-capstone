"""Separate opt-in mission, reusing common controller, tuner and vehicle gate."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from vehicle_bringup_pkg.configuration import config_argument
from vehicle_bringup_pkg.mode_launch import common_arguments, hardware_enabled, as_bool, lidar_node
from vehicle_bringup_pkg.tuning_configuration import tuning_launch_arguments
from vehicle_bringup_pkg.perception_policy import perception_arguments
from vehicle_bringup_pkg.tuned_modes import assemble
from skku_track_drive_pkg.lidar_camera_fusion import CALIBRATION_PATH, SCAN_TOPIC

# 첫 실차 테스트 PWM. speed:= 인자로 변경 가능.
TEST_SPEED = '30'
FUSION_TUNING_PATH = str(Path.home() / '.config/autodrive/mission_lidar_camera_tuning.yaml')  # 별도 미션 P 저장


def fusion_assemble(context):
    actions = assemble(context, 'mission')
    # Replace only the opt-in launch's mission action; shared assembly is unchanged.
    index = next(i for i, action in enumerate(actions)
                 if isinstance(action, Node) and action.node_executable == 'mission_controller_node')
    # Build the same action with its public launch parameters/remappings.
    from vehicle_bringup_pkg.tuning_configuration import resolve_tuning, _profile
    from vehicle_bringup_pkg.mode_launch import command_topic, preview_remappings
    from vehicle_bringup_pkg.recording import analysis_enabled
    from launch_ros.parameter_descriptions import ParameterValue
    settings, _, _, _ = resolve_tuning(context, 'mission')
    settings.update(model_path=LaunchConfiguration('model_path'), device=LaunchConfiguration('device'),
        image_topic='/mission/image_raw', debug_topic='/mission/debug_image', image_reliability='reliable',
        publish_debug=True, publish_bev_debug=True, publish_analysis=analysis_enabled(context),
        start_enabled=True, max_steering=7.0, allow_speed_tuning=True,
        cmd_topic=command_topic(context), calibration_path=LaunchConfiguration('calibration_path'),
        steering_sign=ParameterValue(LaunchConfiguration('steering_sign'), value_type=float))
    actions[index] = Node(package='skku_track_drive_pkg', executable='fusion_mission_controller_node',
        name=_profile('mission')[0], output='screen', parameters=[settings], remappings=preview_remappings(context))
    if hardware_enabled(context):
        actions.append(lidar_node(SCAN_TOPIC))
    if as_bool(context, 'lidar_gui'):
        actions.append(Node(package='skku_track_drive_pkg', executable='lidar_calibration_node', output='screen',
            parameters=[{'calibration_path': LaunchConfiguration('calibration_path')}]))
    return actions


def generate_launch_description():
    model = str(Path(get_package_share_directory('skku_track_drive_pkg'), 'models/mission/best.pt'))
    tuning = tuning_launch_arguments('mission')
    # Override the new launch default without changing the original mission tuning.
    tuning = [a for a in tuning if a.name not in ('speed', 'saved_tuning_path')]
    return LaunchDescription([config_argument(), *common_arguments(camera=True), *perception_arguments('mission'),
        *tuning, DeclareLaunchArgument('speed', default_value=TEST_SPEED),
        DeclareLaunchArgument('saved_tuning_path', default_value=FUSION_TUNING_PATH),
        DeclareLaunchArgument('model_path', default_value=model),
        DeclareLaunchArgument('steering_sign', default_value='1.0'),
        DeclareLaunchArgument('calibration_path', default_value=CALIBRATION_PATH),
        DeclareLaunchArgument('lidar_gui', default_value='true'),
        OpaqueFunction(function=fusion_assemble)])
