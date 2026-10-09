"""Run with unittest; uses synthetic scans/images, never opens hardware."""
import unittest
from types import SimpleNamespace
import numpy as np

from skku_track_drive_pkg import lidar_camera_fusion as cfg
from skku_track_drive_pkg.messages import Detection, DetectionArray, BoundingBox2D, Pose2D, Point2D, Vector2, Mask, MotionCommand
from skku_track_drive_pkg.mission_core import MissionCore
from skku_track_drive_pkg.fusion_mission_core import FusionCore


def obstacle(x=320):
    return Detection(class_name='obstacle', bbox=BoundingBox2D(Pose2D(Point2D(x, 360)), Vector2(80, 80)))


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.p = cfg.validate({})
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        mask = np.zeros((480, 640), np.uint8)
        mask[:, 50:160] = 1
        self.alternate = Detection(class_name='lane1', mask=Mask(bitmap=mask))
        self.path = [[320, 200], [320, 450]]

    def core(self, xy, reason=''):
        owner = SimpleNamespace(fusion_settings=self.p, scan_for_image=lambda: (reason, np.asarray(xy).reshape(-1, 2)))
        return FusionCore(MissionCore(), owner)

    def test_association_distance(self):
        result = cfg.associate(obstacle(), np.array([[1, -.01], [1, 0], [1, .01]]), self.p)
        self.assertAlmostEqual(result['distance_m'], 1, places=3)

    def test_unrelated_bearing_is_not_match(self):
        self.assertIsNone(cfg.associate(obstacle(), np.array([[1, 1], [1, 1.01], [1, .99]]), self.p)['distance_m'])

    def test_sparse_and_multiple_depth_clusters_are_unknown(self):
        for xy in ([[1,0]], [[1,0]]*3 + [[3,0]]*3):
            self.assertIsNone(cfg.associate(obstacle(), np.array(xy), self.p)['distance_m'])

    def test_invalid_scan_points_and_angle_correction(self):
        scan = SimpleNamespace(ranges=[1, float('inf'), float('nan'), .01], angle_min=0, angle_increment=.1, range_min=.1, range_max=12)
        xy = cfg.points(scan, {**self.p, 'yaw_deg': 90})
        np.testing.assert_allclose(xy, [[0, 1]], atol=1e-10)

    def test_camera_path_far_obstacle_preserves_baseline(self):
        core = self.core([[3,0]]*3)
        stop, limit = core.decide(DetectionArray([obstacle(), self.alternate]), self.frame, self.path, 1)
        command = MotionCommand(2, 30, 30)
        self.assertFalse(stop)
        self.assertIs(core.apply(command, stop, limit), command)

    def test_near_obstacle_only_on_other_path_does_not_switch(self):
        core = self.core([[1, .4]]*3)
        stop, _ = core.decide(DetectionArray([obstacle(100), self.alternate]), self.frame, self.path, 1)
        self.assertFalse(stop)
        self.assertEqual(core.target_lane, 2)

    def test_current_path_near_obstacle_switches_after_confirmation(self):
        core = self.core([[1,0]]*3)
        for now in range(3):
            stop, _ = core.decide(DetectionArray([obstacle(), self.alternate]), self.frame, self.path, now)
            self.assertTrue(stop)
        self.assertEqual(core.target_lane, 1)
        self.assertEqual(core.state, 'AVOIDING')

    def test_unknown_or_too_close_stops_without_switch(self):
        for xy, reason in (([], ''), ([[.3,0]]*3, ''), ([], 'SCAN_STALE')):
            core = self.core(xy, reason)
            for now in range(5):
                stop, _ = core.decide(DetectionArray([obstacle(), self.alternate]), self.frame, self.path, now)
                self.assertTrue(stop)
                self.assertEqual(core.target_lane, 2)

    def test_red_latch_survives_missing_lidar_and_unknown_light(self):
        core = self.core([], 'SCAN_STALE')
        core.red_latched = True
        stop, _ = core.decide(DetectionArray(), self.frame, self.path, 1)
        self.assertTrue(stop)
        self.assertTrue(core.red_latched)

    def test_invalid_distance_order_rejected(self):
        with self.assertRaises(ValueError):
            cfg.validate({'stop_m': 2, 'near_m': 1})


if __name__ == '__main__':
    unittest.main()
