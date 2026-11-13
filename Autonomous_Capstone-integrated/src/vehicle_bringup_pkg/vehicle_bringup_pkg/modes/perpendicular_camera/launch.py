"""Explicit preview/live entry points; all new settings stay in one YAML."""
from pathlib import Path
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from vehicle_bringup_pkg.mode_launch import vehicle_remappings, vehicle_io_nodes
from skku_track_drive_pkg.modes.perpendicular_camera.configuration import load_config
from skku_track_drive_pkg.modes.perpendicular_camera.core import ControlConfig
from skku_track_drive_pkg.modes.perpendicular_camera.model import model_path


def _bool(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(name + ' must be true/false')
    return value == 'true'


def assemble(context, preview):
    path = str(Path(LaunchConfiguration('config_file').perform(context)).expanduser().resolve())
    data = load_config(path)
    source = LaunchConfiguration('source').perform(context)
    if source not in ('camera', 'topics', 'replay'):
        raise ValueError('source must be camera/topics/replay')
    if not preview and source == 'replay':
        raise ValueError('Replay is only permitted in the preview launch')
    if not preview and _bool(context, 'use_sim_time'):
        raise ValueError('Live camera parking requires the real ROS clock')
    gui = _bool(context, 'gui')
    if not preview and not gui:
        raise ValueError('Live launch requires the viewer/tuner heartbeat (gui:=true)')
    if not preview and not ControlConfig(**data['control']).calibration_confirmed:
        raise ValueError('Set control.calibration_confirmed only after camera/steering commissioning')
    # Validate both model FILES before starting sensors or serial. Actual class
    # names/task are checked by each model process; controller waits for both.
    overrides = {}
    devices = {}
    for role in ('front', 'rear'):
        overrides[role] = model_path(LaunchConfiguration(role + '_model').perform(context) or data[role]['model_path'])
        devices[role] = LaunchConfiguration(role + '_camera_device').perform(context) or data[role]['device']
    if source == 'camera':
        a, b = (os.path.realpath(devices[role]) for role in ('front', 'rear'))
        if a == b or (os.path.exists(a) and os.path.exists(b) and os.path.samefile(a, b)):
            raise ValueError('Front/rear camera device resolves to the same device')
    # Isolation is tied to the preview entry point, never to an overridable flag.
    remappings = vehicle_remappings('/sensors_only') if preview else []
    namespace = 'parking_camera_preview' if preview else ''
    if preview:
        # A preview must not feed geometry to a simultaneously running live
        # controller, or send reset/save to its identically named services.
        for role in ('front', 'rear'):
            for suffix in ('oriented', 'segmentation', 'bev', 'observation', 'perception_status'):
                remappings.append((f'/parking_camera/{role}/{suffix}', f'/parking_camera_preview/{role}/{suffix}'))
            if source == 'camera':
                remappings.append((data[role]['image_topic'], f'/parking_camera_preview/{role}/image_raw'))
        for suffix in ('status', 'preview_enabled'):
            remappings.append(('/parking_camera/' + suffix, '/parking_camera_preview/' + suffix))
        for suffix in ('reset', 'save'):
            remappings.append((f'/perpendicular_camera_controller/{suffix}',
                               f'/parking_camera_preview/perpendicular_camera_controller/{suffix}'))
    topic = '/sensors_only/topic_control_signal' if preview else 'topic_control_signal'
    actions = []
    watched = []
    if source == 'camera':
        for role in ('front', 'rear'):
            view = data[role]
            camera = Node(package='sensor_bringup_pkg', executable='camera_publisher_node',
                          name=f'parking_{role}_camera', namespace=namespace, remappings=remappings,
                          output='screen', parameters=[{
                              'device': devices[role], 'topic': view['image_topic'],
                              'width': view['width'], 'height': view['height'], 'fps': float(view['fps']),
                              'fourcc': 'MJPG', 'buffer_size': 1, 'reliability': 'reliable',
                              'frame_id': f'parking_{role}_optical', 'show': False}])
            actions.append(camera)
            watched.append(camera)
    for role in ('front', 'rear'):
        perception = Node(package='skku_track_drive_pkg', executable='camera_parking_perception_node',
                          name=f'parking_{role}_perception', namespace=namespace, remappings=remappings,
                          output='screen', parameters=[{
                              'config_file': path, 'camera_role': role, 'model_override': overrides[role]}])
        actions.append(perception)
        watched.append(perception)
    session_overrides = json.dumps({role: {
        'model_path': str(Path(overrides[role]).expanduser().resolve()),
        'device': devices[role]} for role in ('front', 'rear')})
    controller = Node(package='skku_track_drive_pkg', executable='camera_parking_controller_node',
                      name='perpendicular_camera_controller', namespace=namespace, output='screen', remappings=remappings,
                      parameters=[{'config_file': path, 'preview': preview, 'cmd_topic': topic,
                                   'session_overrides_json': session_overrides}])
    actions.append(controller)
    watched.append(controller)
    if gui:
        viewer = Node(package='skku_track_drive_pkg', executable='camera_parking_viewer_node',
                      name='perpendicular_camera_viewer', namespace=namespace, output='screen', remappings=remappings,
                      parameters=[{'preview': preview}])
        actions.append(viewer)
        watched.append(viewer)
    if not preview:
        io = vehicle_io_nodes()
        actions.extend(io)
        watched.extend(io)
    # A model/camera/UI/controller process failure shuts down this entire mode.
    # The existing sender's health/command leases also expire independently.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='Camera parking component exited'))])) for node in watched]
    return handlers + actions


def description(preview):
    default = str(Path(get_package_share_directory('vehicle_bringup_pkg'), 'config', 'perpendicular_camera.yaml'))
    args = [DeclareLaunchArgument('config_file', default_value=default),
            DeclareLaunchArgument('source', default_value='camera', description='camera opens devices; topics/replay use external Image topics'),
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('command_timeout', default_value='0.5'),
            DeclareLaunchArgument('ui_timeout', default_value='0.75'),
            DeclareLaunchArgument('arduino_port', default_value='/dev/arduino'),
            DeclareLaunchArgument('arduino_baud', default_value='115200'),
            DeclareLaunchArgument('auto_calibrate', default_value='false'),
            DeclareLaunchArgument('calibration_tolerance', default_value='35')]
    for role in ('front', 'rear'):
        args.extend([DeclareLaunchArgument(role + '_camera_device', default_value=''),
                     DeclareLaunchArgument(role + '_model', default_value='')])
    return LaunchDescription(args + [SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
                                    OpaqueFunction(function=lambda context: assemble(context, preview))])
