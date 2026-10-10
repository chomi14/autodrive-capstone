"""ROS launch assembly check without starting any nodes or opening hardware."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys

ROOT = Path(__file__).resolve().parents[2]
for package in ('skku_track_drive_pkg', 'lidar_perception_pkg', 'vehicle_bringup_pkg'):
    sys.path.insert(0, str(ROOT / 'src' / package))

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node



class LaunchTests(unittest.TestCase):
    def test_track_tuning_opt_in_forwards_current_parameters_without_duplicate_nodes(self):
        from types import SimpleNamespace
        import json
        import vehicle_bringup_pkg.configuration as configuration
        import vehicle_bringup_pkg.tuning_configuration as tuning
        import vehicle_bringup_pkg.tuned_modes as modes
        import vehicle_bringup_pkg.perception_policy as policy
        def share(name):
            return str(ROOT/'src'/name)
        def module(path, name):
            spec = importlib.util.spec_from_file_location(name, ROOT/path)
            result = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(result)
            return result
        with patch.object(configuration, 'get_package_share_directory', share), \
             patch.object(tuning, 'get_package_share_directory', share), \
             patch.object(modes, 'get_package_share_directory', share), \
             patch.object(policy, 'get_package_share_directory', share), \
             patch('ament_index_python.packages.get_package_share_directory', share), tempfile.TemporaryDirectory() as tmp:
            track = module('src/vehicle_bringup_pkg/launch/track_drive_tuning.launch.py', 'track_launch')
            bbox = module('src/vehicle_bringup_pkg/launch/mission_bbox_only.launch.py', 'bbox_launch')
            context = LaunchContext()
            for action in track.generate_launch_description().entities:
                if isinstance(action, DeclareLaunchArgument):
                    context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
            context.launch_configurations.update(dry_run='true', saved_tuning_path=str(Path(tmp)/'track.yaml'))
            normal = track._launch_nodes(context)
            self.assertEqual(sum(isinstance(a, Node) and a.node_executable == 'track_controller_node' for a in normal), 1)
            saved = Path(tmp)/'track.yaml'
            saved.write_text(json.dumps({'track_controller_node': {'ros__parameters': {'speed': 77, 'roi_cut': 280, 'bev_src_tl_x': 210}}}))
            context.launch_configurations.update(bbox_obstacle_avoidance='true', speed='30', tuning_config=str(saved))
            with patch.object(track, 'IncludeLaunchDescription', side_effect=lambda source, launch_arguments: SimpleNamespace(args=dict(launch_arguments))):
                actions = track._launch_nodes(context)
            self.assertEqual(len(actions), 1)
            forwarded = actions[0].args
            self.assertEqual(forwarded['tuning_config'], '')
            self.assertEqual(forwarded['track_tuning_path'], str(saved))
            self.assertNotEqual(forwarded['saved_tuning_path'], str(saved))
            child = LaunchContext()
            child.launch_configurations.update(forwarded)
            for action in bbox.generate_launch_description().entities:
                if isinstance(action, DeclareLaunchArgument) and action.name not in child.launch_configurations:
                    child.launch_configurations[action.name] = perform_substitutions(child, action.default_value)
            settings = bbox.resolve_bbox_settings(child)
            self.assertEqual(settings['speed'], 30)
            self.assertEqual(settings['roi_cut'], 280)
            self.assertEqual(settings['bev_src_tl_x'], 210)
            nodes = bbox.bbox_assemble(child)
            self.assertEqual(sum(isinstance(a, Node) and a.node_executable == 'bbox_mission_controller_node' for a in nodes), 1)
            self.assertFalse(any(isinstance(a, Node) and a.node_executable in ('track_controller_node', 'lidar_publisher_node_v2') for a in nodes))

    def test_new_launch_sensor_isolation_and_single_command_source(self):
        import vehicle_bringup_pkg.configuration as configuration
        import vehicle_bringup_pkg.tuning_configuration as tuning
        import vehicle_bringup_pkg.tuned_modes as modes
        import vehicle_bringup_pkg.perception_policy as policy
        def share(name):
            return str(ROOT / 'src' / name)
        with patch.object(configuration, 'get_package_share_directory', share), \
             patch.object(tuning, 'get_package_share_directory', share), \
             patch.object(modes, 'get_package_share_directory', share), \
             patch.object(policy, 'get_package_share_directory', share):
            file = ROOT / 'src/vehicle_bringup_pkg/launch/mission_bbox_only.launch.py'
            spec = importlib.util.spec_from_file_location('fusion_launch', file)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            with patch.object(module, 'get_package_share_directory', share), tempfile.TemporaryDirectory() as tmp:
                for dry, sensors in (('true', 'false'), ('false', 'true'), ('false', 'false')):
                    context = LaunchContext()
                    for action in module.generate_launch_description().entities:
                        if isinstance(action, DeclareLaunchArgument):
                            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
                    context.launch_configurations.update(dry_run=dry, sensors_only=sensors,
                        saved_tuning_path=str(Path(tmp)/'mission.yaml'), track_tuning_path=str(Path(tmp)/'track.yaml'))
                    actions = module.bbox_assemble(context)
                    nodes = [a for a in actions if isinstance(a, Node)]
                    names = [perform_substitutions(context, normalize_to_list_of_substitutions(n.node_executable)) for n in nodes]
                    self.assertEqual(names.count('bbox_mission_controller_node'), 1)
                    self.assertNotIn('mission_controller_node', names)
                    self.assertNotIn('lidar_publisher_node_v2', names)
                    self.assertEqual(names.count('serial_sender_node_v2'), 1 if dry == sensors == 'false' else 0)
                    self.assertEqual(module.resolve_bbox_settings(context)['speed'], 250)
                    # EDIT HERE must reach effective controller settings, not just comments.
                    with patch.dict(module.DRIVING_DEFAULTS, {'speed': 47}), patch.dict(module.TUNING_DEFAULTS, {'speed': 47}):
                        self.assertEqual(module.resolve_bbox_settings(context)['speed'], 47)
                    context.launch_configurations.update(speed='23', roi_cut='280')
                    self.assertEqual(module.resolve_bbox_settings(context)['speed'], 23)
                    self.assertEqual(module.resolve_bbox_settings(context)['roi_cut'], 280)
                    context.launch_configurations.update(speed='', roi_cut='')
                # An existing P-save must not silently defeat edits at the top.
                import json
                saved = Path(tmp)/'mission.yaml'
                saved.write_text(json.dumps({'mission_controller_node': {'ros__parameters': {'speed': 55, 'obstacle_near_y': 350}}}))
                context.launch_configurations['load_saved_tuning'] = 'false'
                self.assertEqual(module.resolve_bbox_settings(context)['speed'], 250)
                self.assertEqual(module.resolve_bbox_settings(context)['obstacle_near_y'], 300)
                context.launch_configurations['load_saved_tuning'] = 'true'
                self.assertEqual(module.resolve_bbox_settings(context)['speed'], 55)
                self.assertEqual(module.resolve_bbox_settings(context)['obstacle_near_y'], 350)
                context.launch_configurations['speed'] = '21'
                self.assertEqual(module.resolve_bbox_settings(context)['speed'], 21)
                context.launch_configurations.update(speed='', tuning_config=str(saved), load_saved_tuning='false')
                self.assertEqual(module.resolve_bbox_settings(context)['speed'], 55)
                self.assertEqual(set(module.TUNING_DEFAULTS), tuning._profile('mission')[3])
                # Port uses the real 1009 track defaults and its live BEV parameters.
                track_defaults = tuning._validate_parameter_file(ROOT/'src/vehicle_bringup_pkg/config/track_tuning.yaml')
                for name, value in track_defaults.items():
                    self.assertEqual(module.TUNING_DEFAULTS[name], value, name)
                track = Path(tmp)/'track.yaml'
                track.write_text(json.dumps({'track_controller_node': {'ros__parameters': {
                    'speed': 37, 'stanley_gain': 0.033, 'bev_src_tl_x': 210, 'roi_cut': 280}}}))
                context.launch_configurations.update(tuning_config='', load_saved_tuning='false')
                actual = module.resolve_bbox_settings(context)
                self.assertEqual(actual['speed'], 37)
                self.assertEqual(actual['stanley_gain'], 0.033)
                self.assertEqual(actual['bev_src_tl_x'], 210)
                self.assertEqual(actual['roi_cut'], 280)
                self.assertEqual(actual['obstacle_near_y'], 300)
                context.launch_configurations['bev_src_tl_x'] = '230'
                self.assertEqual(module.resolve_bbox_settings(context)['bev_src_tl_x'], 230)
                context.launch_configurations.update(bev_src_tl_x='', load_saved_track_tuning='false')
                self.assertEqual(module.resolve_bbox_settings(context)['speed'], 250)


if __name__ == '__main__':
    unittest.main()
