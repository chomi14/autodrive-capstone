"""Mission must preserve every packaged track-driving tuning default."""
from pathlib import Path
import unittest

import yaml


class CommonDrivingDefaultsTest(unittest.TestCase):
    def test_mission_matches_track_defaults(self):
        config = Path(__file__).resolve().parents[2] / 'src/vehicle_bringup_pkg/config'
        with (config / 'track_tuning.yaml').open(encoding='utf-8') as stream:
            track = yaml.safe_load(stream)['track_controller_node']['ros__parameters']
        with (config / 'mission_tuning.yaml').open(encoding='utf-8') as stream:
            mission = yaml.safe_load(stream)['mission_controller_node']['ros__parameters']
        for name, value in track.items():
            with self.subTest(parameter=name):
                self.assertIn(name, mission)
                self.assertEqual(mission[name], value)
        self.assertEqual(mission['obstacle_near_y'], 300)
        self.assertEqual(mission['avoid_speed'], 80)
        self.assertEqual(mission['red_confirm_frames'], 3)


if __name__ == '__main__':
    unittest.main()
