"""General lane keeping only. No serial process unless sensors_only:=false."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from vehicle_bringup_pkg.configuration import config_argument, VehicleDefault
from vehicle_bringup_pkg.tuning_configuration import resolve_tuning, tuning_launch_arguments


def assemble(context):
    def value(k): return LC(k).perform(context)
    def yes(k): return value(k).lower() in ('true', '1')
    params, selected, saved, source = resolve_tuning(context)
    if not yes('sensors_only') and not value('speed').strip():
        raise ValueError('Serial mode requires explicit speed:=PWM after bench calibration; PWM is not measured velocity')
    prefix = '/vehicle' if not yes('sensors_only') else '/sensors_only/vehicle'
    cmd = '/topic_control_signal' if not yes('sensors_only') else '/sensors_only/topic_control_signal'
    remap = [(f'vehicle/{name}', f'{prefix}/{name}') for name in ('drive_key','tuner_heartbeat','controller_heartbeat','command_status','arm_challenge','drive_state')]
    nodes = [LogInfo(msg=f'General driving only; {source}: {selected}; serial={not yes("sensors_only")}'),
        Node(package='sensor_bringup_pkg', executable='camera_publisher_node', parameters=[dict(
            device=value('camera_device'), topic='/track/image_raw', width=640,height=480,fps=30.,fourcc='MJPG',buffer_size=1,show=False)]),
        Node(package='skku_track_drive_pkg', executable='track_controller_node', name='track_controller_node', remappings=remap,
             parameters=[{**params, 'image_topic':'/track/image_raw','cmd_topic':cmd,'device':value('device'),
                          'start_enabled':True,'allow_speed_tuning':False,'max_steering':7.,'steering_sign':1.,
                          'publish_debug':yes('gui'),'publish_bev_debug':yes('gui'),'publish_analysis':True}])]
    if yes('gui'):
        nodes.append(Node(package='skku_track_drive_pkg', executable='track_tuner_node', remappings=remap,
            parameters=[dict(target_node='/track_controller_node',cmd_topic=cmd,debug_topic='/track_debug_image',
                             save_path=str(saved),loaded_config_path=str(selected),allow_speed_tuning=False)]))
    if not yes('sensors_only'):
        nodes += [Node(package='vehicle_io_pkg', executable='serial_sender_node_v2',parameters=[dict(
            port=value('arduino_port'),baud=int(value('arduino_baud')),topic=cmd,auto_calibrate=False,
            command_timeout=0.,ui_timeout=.75,require_tuner_heartbeat=True,require_controller_heartbeat=True,
            require_perception_progress=True,perception_stop_s=float(value('perception_stop_s')))]),
            Node(package='vehicle_io_pkg',executable='drive_arm_node',emulate_tty=True,parameters=[dict(show_window=True)])]
    if value('record_dir'):
        nodes.append(Node(package='skku_track_drive_pkg',executable='tuning_recorder_node',parameters=[dict(
            output_dir=value('record_dir'),mode='track',input_topic='/e2e/source_frame',command_topic=cmd,
            vehicle_prefix=prefix,controller_root='track_controller_node')]))
    return nodes


def generate_launch_description():
    share=Path(get_package_share_directory('vehicle_bringup_pkg'))
    return LaunchDescription([config_argument(),
        DeclareLaunchArgument('camera_device',default_value='/dev/video2'),
        DeclareLaunchArgument('device',default_value='cpu'),
        DeclareLaunchArgument('sensors_only',default_value='true'),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('arduino_port',default_value=VehicleDefault('arduino.port','/dev/arduino')),
        DeclareLaunchArgument('arduino_baud',default_value='115200'),
        DeclareLaunchArgument('perception_stop_s',default_value='6.0'),
        DeclareLaunchArgument('record_dir',default_value=''),
        *tuning_launch_arguments(),OpaqueFunction(function=assemble)])
