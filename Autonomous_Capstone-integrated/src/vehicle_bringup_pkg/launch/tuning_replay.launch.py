"""Reuse the canonical launch/GUI with dry_run forced and input-only replay."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def assemble(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    mode, kind, path = arg('mode'), arg('input_kind'), Path(arg('input_path')).expanduser()
    names = {'track': 'track_drive_tuning', 'mission': 'mission_drive_tuning',
             'perpendicular': 'perpendicular_parking', 'parallel': 'parallel_parking',
             'calibration': 'parking_calibration'}
    if mode not in names or kind not in ('video', 'bag') or not path.exists():
        raise ValueError('Choose a valid mode, input_kind=video|bag, and existing input_path')
    roots = {'track':'track_controller_node','mission':'mission_controller_node',
             'perpendicular':'perpendicular_parking_controller_node','parallel':'parallel_parking_controller_node',
             'calibration':'parking_calibration_controller_node'}
    camera = mode in ('track', 'mission')
    input_topic = '/track/image_raw' if mode == 'track' else '/mission/image_raw' if mode == 'mission' else '/parking/scan'
    arguments = {'dry_run': 'true', 'gui': arg('gui'), 'analysis': 'true',
                 'record_dir': arg('record_dir'), 'tuning_config': arg('tuning_config')}
    if arg('saved_tuning_path'):
        arguments['saved_tuning_path'] = arg('saved_tuning_path')
    if camera:
        arguments['device'] = arg('device')
    launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('vehicle_bringup_pkg'),
        'launch', names[mode]+'.launch.py'))), launch_arguments=arguments.items())
    if kind == 'video':
        if not camera:
            raise ValueError('Video input is for track/mission; use a LiDAR bag for parking')
        source = Node(package='sensor_bringup_pkg', executable='video_replay_node', parameters=[{
            'video_path': str(path), 'topic': input_topic, 'wait_for_subscriber': True,
            'loop': arg('loop').lower() == 'true', 'playback_fps': float(arg('playback_fps')),
            'max_frames': int(arg('max_frames')), 'reliability': 'reliable', 'required_subscriber_node': roots[mode],
            'wait_for_recorder': bool(arg('record_dir'))}], output='screen')
    else:
        source = Node(package='skku_track_drive_pkg', executable='tuning_bag_replay_node', output='screen',
            parameters=[{'bag_path': str(path), 'input_topic': input_topic, 'mode': mode, 'target_node': roots[mode],
                         'rate': float(arg('rate')), 'loop': arg('loop').lower() == 'true',
                         'processed_csv': arg('processed_csv'), 'wait_for_recorder': bool(arg('record_dir'))}])
    return [launch, source, RegisterEventHandler(OnProcessExit(target_action=source,
            on_exit=[EmitEvent(event=Shutdown(reason='offline input replay finished'))]))]


def generate_launch_description():
    defaults = {'mode': 'track', 'input_kind': 'bag', 'input_path': '', 'gui': 'true',
                'device': 'cuda:0', 'rate': '1.0', 'playback_fps': '0.0', 'max_frames': '0',
                'loop': 'false', 'record_dir': '', 'tuning_config': '', 'processed_csv': '', 'saved_tuning_path': ''}
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()],
                              OpaqueFunction(function=assemble)])
