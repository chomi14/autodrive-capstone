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
                        saved_tuning_path=str(Path(tmp)/'mission.yaml'))
                    actions = module.bbox_assemble(context)
                    nodes = [a for a in actions if isinstance(a, Node)]
                    names = [perform_substitutions(context, normalize_to_list_of_substitutions(n.node_executable)) for n in nodes]
                    self.assertEqual(names.count('bbox_mission_controller_node'), 1)
                    self.assertNotIn('mission_controller_node', names)
                    self.assertNotIn('lidar_publisher_node_v2', names)
                    self.assertEqual(names.count('serial_sender_node_v2'), 1 if dry == sensors == 'false' else 0)
                    self.assertEqual(module.resolve_bbox_settings(context)['speed'], 30)
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
                self.assertEqual(module.resolve_bbox_settings(context)['speed'], 30)
                self.assertEqual(module.resolve_bbox_settings(context)['obstacle_near_y'], 300)
                context.launch_configurations['load_saved_tuning'] = 'true'
                self.assertEqual(module.resolve_bbox_settings(context)['speed'], 55)
                self.assertEqual(module.resolve_bbox_settings(context)['obstacle_near_y'], 350)
                context.launch_configurations['speed'] = '21'
                self.assertEqual(module.resolve_bbox_settings(context)['speed'], 21)
                context.launch_configurations.update(speed='', tuning_config=str(saved), load_saved_tuning='false')
                self.assertEqual(module.resolve_bbox_settings(context)['speed'], 55)
                self.assertEqual(set(module.TUNING_DEFAULTS), tuning._profile('mission')[3])


if __name__ == '__main__':
    unittest.main()
