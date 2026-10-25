"""Size launch -> existing single-camera/single-controller pipeline, no hardware."""
import importlib.util
from pathlib import Path
import tempfile
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[2]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.mark.parametrize('dry,sensors', [('true','false'),('false','true'),('false','false')])
def test_size_launch_is_opt_in_and_has_one_command_owner(dry,sensors):
    import vehicle_bringup_pkg.configuration as config
    import vehicle_bringup_pkg.tuning_configuration as tuning
    import vehicle_bringup_pkg.tuned_modes as modes
    import vehicle_bringup_pkg.perception_policy as policy

    def share(name):
        return str(ROOT / 'src' / name)

    with patch('ament_index_python.packages.get_package_share_directory',share), \
         patch.object(config,'get_package_share_directory',share), \
         patch.object(tuning,'get_package_share_directory',share), \
         patch.object(modes,'get_package_share_directory',share), \
         patch.object(policy,'get_package_share_directory',share), tempfile.TemporaryDirectory() as tmp:
        size=module('size_launch','src/vehicle_bringup_pkg/launch/mission_bbox_size.launch.py')
        base=module('size_base_launch','src/vehicle_bringup_pkg/launch/mission_bbox_only.launch.py')
        ctx=LaunchContext()
        ctx.launch_configurations.update(dry_run=dry,sensors_only=sensors,
            speed='30',track_tuning_path=str(Path(tmp)/'no_track.yaml'))
        actions=size.generate_launch_description().entities
        for action in actions:
            if isinstance(action,DeclareLaunchArgument):
                action.execute(ctx)
        include=next(action for action in actions if isinstance(action,IncludeLaunchDescription))
        included=include.launch_description_source.get_launch_description(ctx)
        assert any(isinstance(a,DeclareLaunchArgument) and a.name=='camera_device'
                   for a in included.entities)
        for action in base.generate_launch_description().entities:
            if isinstance(action,DeclareLaunchArgument):
                action.execute(ctx)
        settings=base.resolve_bbox_settings(ctx)
        assert settings['speed']==30
        assert settings['obstacle_trigger_mode']==1
        assert settings['obstacle_stop_enabled']==0
        assert settings['obstacle_min_area_ratio']==.02
        assert settings['obstacle_near_y']==300
        assert settings['lane_change_cooldown_s']==2
        assert ctx.launch_configurations['saved_tuning_path'].endswith('mission_bbox_size_tuning.yaml')
        nodes=[a for a in base.bbox_assemble(ctx) if isinstance(a,Node)]
        names=[node.node_executable for node in nodes]
        assert names.count('bbox_mission_controller_node')==1
        assert names.count('serial_sender_node_v2')==(1 if dry==sensors=='false' else 0)
        assert names.count('camera_publisher_node')==(0 if dry=='true' else 1)
        assert not any('lidar' in name for name in names)
        controller=next(n for n in nodes if n.node_executable=='bbox_mission_controller_node')
        params=evaluate_parameters(ctx,controller._Node__parameters)[0]
        assert params['obstacle_trigger_mode']==1 and params['speed']==30
        assert params['initial_target_lane']==2
        # Explicit command-line/GUI-compatible values override defaults.
        ctx.launch_configurations.update(obstacle_min_area_ratio='0.035',obstacle_near_y='320',
                                         initial_target_lane='1')
        assert base.resolve_bbox_settings(ctx)['obstacle_min_area_ratio']==.035
        assert base.resolve_bbox_settings(ctx)['obstacle_near_y']==320
        nodes=[a for a in base.bbox_assemble(ctx) if isinstance(a,Node)]
        controller=next(n for n in nodes if n.node_executable=='bbox_mission_controller_node')
        assert evaluate_parameters(ctx,controller._Node__parameters)[0]['initial_target_lane']==1
