"""BBox decisions use 1009's current live BEV geometry and track command."""
from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from rclpy.parameter import Parameter
from skku_track_drive_pkg.bbox_mission_controller_node import BboxMissionControllerNode
from skku_track_drive_pkg.lane_processing import LaneInfoExtractor
from skku_track_drive_pkg.mission_core import MissionCore
from skku_track_drive_pkg.messages import (
    DetectionArray, Detection, BoundingBox2D, Pose2D, Point2D, Vector2,
    MotionCommand, PathPlanningResult,
)


def controller():
    node = object.__new__(BboxMissionControllerNode)
    node.lane = LaneInfoExtractor(show_image=False, bev_top_shift=-8, roi_cut=300)
    node.mission = MissionCore()
    node.status_pub = Mock()
    node._parameter_lock = RLock()
    node.allow_speed_tuning = True
    node.speed = 250
    node.get_logger = Mock(return_value=Mock())
    return node


def test_no_obstacle_keeps_the_exact_1009_stanley_command():
    node = controller()
    baseline = MotionCommand(-3, 250, 250)
    path = PathPlanningResult([320, 320], [20, 160])
    frame = np.zeros((480, 640, 3), np.uint8)
    assert node.adjust_command(baseline, DetectionArray(), path, frame) is baseline


def test_live_bev_changes_the_bbox_collision_path_and_clear_lane_stops():
    node = controller()
    baseline = MotionCommand(2, 250, 250)
    path = PathPlanningResult([320, 320], [20, 160])
    frame = np.zeros((480, 640, 3), np.uint8)
    node.adjust_command(baseline, DetectionArray(), path, frame)
    previous = np.array(node.mission.status['camera_path'])
    params = [Parameter(f'bev_src_{corner}_x', value=x-60)
              for corner, (x, _) in zip(('tl', 'tr', 'br', 'bl'), node.lane.bev_source_points)]
    assert node.on_tuning_parameters(params).successful
    node.adjust_command(baseline, DetectionArray(), path, frame)
    current = np.array(node.mission.status['camera_path'])
    np.testing.assert_allclose(current[:, 0], previous[:, 0]-60, atol=.001)
    np.testing.assert_allclose(current[:, 1], previous[:, 1], atol=.001)
    x, y = current[-1]
    obstacle = Detection(class_name='obstacle',
        bbox=BoundingBox2D(Pose2D(Point2D(float(x), float(y-20))), Vector2(20, 40)))
    command = node.adjust_command(baseline, DetectionArray([obstacle]), path, frame)
    assert node.mission.status['path_blocked']
    assert command.left_speed == command.right_speed == 0
    # Opposite lane is not visible; consecutive detection must not switch blindly.
    for _ in range(3):
        command = node.adjust_command(baseline, DetectionArray([obstacle]), path, frame)
    assert node.mission.state == 'BLOCKED_STOP'
    assert node.mission.target_lane == 2
    assert command.left_speed == command.right_speed == 0
