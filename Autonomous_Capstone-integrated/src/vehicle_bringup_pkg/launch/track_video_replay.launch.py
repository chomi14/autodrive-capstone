"""Offline track-pipeline test: video input, controller output, no actuators."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from vehicle_bringup_pkg.tuning_configuration import (
    resolve_tuning,
    tuning_launch_arguments,
)
from vehicle_bringup_pkg.mode_launch import vehicle_remappings


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('1', 'true', 'yes', 'on'):
        return True
    if value in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false, got: {value}')


def _launch_nodes(context):
    raw_video_path = LaunchConfiguration('video_path').perform(context).strip()
    if not raw_video_path:
        raise RuntimeError('video_path is required and must point to a video file')
    video_path = Path(raw_video_path).expanduser()
    if not video_path.is_file():
        raise RuntimeError(f'video_path does not exist or is not a file: {video_path}')
    tuning_parameters, selected_config, save_path, config_source = resolve_tuning(context)
    enable_visualization = any((
        _as_bool(context, 'enable_visualization'),
        _as_bool(context, 'publish_debug'),
        _as_bool(context, 'debug'),
    ))
    show_bev = _as_bool(context, 'show_bev')

    replay = Node(
        package='sensor_bringup_pkg',
        executable='video_replay_node',
        name='video_replay_node',
        output='screen',
        parameters=[{
            'video_path': str(video_path),
            'loop': ParameterValue(LaunchConfiguration('loop'), value_type=bool),
            'playback_fps': ParameterValue(LaunchConfiguration('playback_fps'), value_type=float),
            'topic': '/camera/front/image_raw',
            'frame_id': 'front_camera_frame',
            'width': 640,
            'height': 480,
            'wait_for_subscriber': True,
            'max_frames': ParameterValue(LaunchConfiguration('max_frames'), value_type=int),
        }],
    )
    controller_parameters = dict(tuning_parameters)
    controller_parameters.update({
        'image_topic': '/camera/front/image_raw',
        'image_reliability': 'reliable',
        'cmd_topic': '/dry_run/topic_control_signal',
        'device': LaunchConfiguration('device'),
        'start_enabled': True,
        'publish_debug': enable_visualization,
        'publish_bev_debug': show_bev,
        'debug_log': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
        'profile': ParameterValue(LaunchConfiguration('profile'), value_type=bool),
        'profile_warmup_frames': ParameterValue(
            LaunchConfiguration('profile_warmup_frames'), value_type=int
        ),
        'profile_report_interval': ParameterValue(
            LaunchConfiguration('profile_report_interval'), value_type=int
        ),
        'profile_input_fps': ParameterValue(
            LaunchConfiguration('playback_fps'), value_type=float
        ),
    })
    controller = Node(
        package='skku_track_drive_pkg',
        executable='track_controller_node',
        name='track_controller_node',
        output='screen',
        parameters=[controller_parameters],
        remappings=vehicle_remappings('/dry_run'),
    )
    nodes = [
        LogInfo(msg=f'Using {config_source} tuning config: {selected_config}'),
        controller,
        replay,
    ]
    enable_tuner = _as_bool(context, 'enable_tuner')
    if enable_tuner:
        nodes.append(Node(
            package='skku_track_drive_pkg',
            executable='track_tuner_node',
            name='track_tuner_node',
            output='screen',
            parameters=[{
                'target_node': '/track_controller_node',
                'cmd_topic': '/dry_run/topic_control_signal',
                'debug_topic': '/track_debug_image',
                'save_path': str(save_path),
                'loaded_config_path': str(selected_config),
            }],
            remappings=vehicle_remappings('/dry_run'),
        ))
    stop_when_replay_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=replay,
            on_exit=[EmitEvent(event=Shutdown(reason='video replay finished'))],
        )
    )
    nodes.append(stop_when_replay_exits)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_path',
            default_value='',
            description='Absolute or relative path to the video file (required)',
        ),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument(
            'playback_fps',
            default_value='0.0',
            description='0 uses the video FPS metadata; a positive value overrides it',
        ),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('enable_tuner', default_value='false'),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Publish and display Track Debug View when the tuner runs',
        ),
        DeclareLaunchArgument(
            'show_bev',
            default_value='true',
            description='Append the Lane BEV side panel to Track Debug View',
        ),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument(
            'publish_debug',
            default_value='false',
            description='Legacy alias for enable_visualization',
        ),
        DeclareLaunchArgument('profile', default_value='false'),
        DeclareLaunchArgument('profile_warmup_frames', default_value='10'),
        DeclareLaunchArgument('profile_report_interval', default_value='50'),
        DeclareLaunchArgument(
            'max_frames',
            default_value='0',
            description='0 replays the full video; positive values stop after N published frames',
        ),
        *tuning_launch_arguments(),
        OpaqueFunction(function=_launch_nodes),
    ])
