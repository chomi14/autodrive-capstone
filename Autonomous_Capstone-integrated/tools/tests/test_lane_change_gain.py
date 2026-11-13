"""Exercise mission gain timing, actual steering, GUI controls and persistence without I/O."""
import json
from unittest.mock import Mock, patch

import numpy as np
import pytest
from rclpy.parameter import Parameter

from skku_track_drive_pkg.mode_parameters import MISSION, trackbars
from skku_track_drive_pkg.messages import (
    Detection, DetectionArray, Mask, BoundingBox2D, Pose2D, Point2D, Vector2,
    MotionCommand, PathPlanningResult,
)
from skku_track_drive_pkg.mission_controller_node import MissionControllerNode
from skku_track_drive_pkg.track_controller_node import TrackControllerNode
from skku_track_drive_pkg.track_tuner_node import TrackTunerNode, MISSION_PRIMARY_CONTROLS
from vehicle_bringup_pkg.tuning_configuration import _profile, _validate_parameter_file
from test_tuning_revision import bare_controller


def prepare(node, now):
    with patch('skku_track_drive_pkg.modes.bbox_mission.mission_controller_node.time.monotonic', return_value=now):
        node.prepare_lane(DetectionArray(), None)


@pytest.mark.parametrize('source', ['obstacle', 'manual'])
def test_gain_starts_with_new_lane_and_expires_independently_of_cooldown(source):
    node = bare_controller(MissionControllerNode, 250)
    node.mission.p.update(lane_change_gain=.014, lane_change_gain_duration_s=1.5)
    prepare(node, 9)
    assert node.motion.effective_stanley_gain == .027
    if source == 'manual':
        node.mission.toggle_target_lane(10)
    else:
        mask = np.zeros((480, 640), np.uint8)
        mask[:, 380:460] = 1
        objects = DetectionArray([
            Detection(class_name='lane2', mask=Mask(bitmap=mask)),
            Detection(class_name='obstacle', bbox=BoundingBox2D(Pose2D(Point2D(420, 360)), Vector2(80, 80))),
        ])
        node.mission.decide(objects, np.zeros((480, 640, 3), np.uint8), [], 10)
    assert node.mission.target_lane == 1
    for now in (10, 10.5, 11.499):
        prepare(node, now)
        assert node._processed_lane == 1
        assert node.motion.effective_stanley_gain == .014
        assert node.motion.stanley_gain == .027  # Saved normal tuning stays intact.
        assert node._lane_change_gain_remaining_s == pytest.approx(11.5-now)
    prepare(node, 11.5)
    assert node.motion.stanley_gain_override is None
    assert node.motion.effective_stanley_gain == .027
    assert node.mission.cooldown_remaining(11.5) == 3.5


def test_live_tuning_restart_disable_and_restore_latest_normal_gain():
    node = bare_controller(MissionControllerNode, 250)
    node.mission.toggle_target_lane(10)
    assert node.on_tuning_parameters([
        Parameter('lane_change_gain', value=.012),
        Parameter('lane_change_gain_duration_s', value=2.0),
        Parameter('stanley_gain', value=.04),
    ]).successful
    prepare(node, 11)
    assert node.motion.effective_stanley_gain == .012
    node.mission.toggle_target_lane(11.5)
    prepare(node, 12)
    assert node._lane_change_gain_remaining_s == 1.5
    assert node.motion.effective_stanley_gain == .012
    prepare(node, 13.5)
    assert node.motion.effective_stanley_gain == .04
    # A configured gain above normal must never amplify the command.
    node.mission.toggle_target_lane(14)
    assert node.on_tuning_parameters([Parameter('stanley_gain', value=.01)]).successful
    prepare(node, 14.1)
    assert node.motion.effective_stanley_gain == .01
    assert node.on_tuning_parameters([Parameter('lane_change_gain_duration_s', value=0.0)]).successful
    prepare(node, 14.2)
    assert node.motion.stanley_gain_override is None
    assert node._lane_change_gain_remaining_s == 0


def test_override_reduces_actual_steering_and_reports_gain_used_for_command():
    node = bare_controller(MissionControllerNode, 250)
    node.motion.stanley_gain = .04
    node.motion.lookahead_steps = 1
    path = PathPlanningResult([385]*6, [160, 130, 100, 70, 40, 10])
    normal = node.motion.plan(DetectionArray(), path)
    node.mission.toggle_target_lane(10)
    prepare(node, 10.2)
    reduced = node.motion.plan(DetectionArray(), path)
    assert 0 < abs(reduced.steering) < abs(normal.steering)
    assert (reduced.left_speed, reduced.right_speed) == (250, 250)
    assert node.motion.heading_gain == .55
    frame = np.zeros((480, 640, 3), np.uint8)
    with patch('skku_track_drive_pkg.modes.bbox_mission.mission_controller_node.time.monotonic', return_value=10.2):
        result = node.adjust_command(reduced, DetectionArray(), path, frame)
    assert result is reduced
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['effective_stanley_gain'] == .014
    assert status['lane_change_gain_active'] is True
    assert status['lane_change_gain_remaining_s'] == pytest.approx(1.3)
    # Gain adjustment must not bypass an existing red stop.
    node.mission.red_latched = True
    assert node.adjust_command(reduced, DetectionArray(), path, frame) == MotionCommand(0, 0, 0)


@pytest.mark.parametrize('name,value', [
    ('lane_change_gain', -.001), ('lane_change_gain', .201),
    ('lane_change_gain', float('nan')), ('lane_change_gain_duration_s', -.1),
    ('lane_change_gain_duration_s', 10.1), ('lane_change_gain_duration_s', 1),
])
def test_invalid_gain_settings_are_rejected_atomically(name, value):
    node = bare_controller(MissionControllerNode, 250)
    original = dict(node.mission.p)
    assert not node.on_tuning_parameters([Parameter(name, value=value)]).successful
    assert node.mission.p == original


def test_gui_places_both_controls_on_main_panel_and_saves_reloadable_values(tmp_path):
    tuner = object.__new__(TrackTunerNode)
    tuner.tuner_window = 'Mission Tuner'
    tuner.mode_controls_separate = True
    tuner.trackbars = trackbars({name: MISSION[name] for name in MISSION_PRIMARY_CONTROLS})
    tuner.current_values = tuner.desired_values = {
        'obstacle_near_y': 300, 'lane_change_gain': .028, 'lane_change_gain_duration_s': 1.7,
    }
    tuner.control_windows, tuner.parameter_windows = [], {}
    with patch('cv2.namedWindow'), patch('cv2.resizeWindow'), patch('cv2.createTrackbar') as create:
        tuner._create_ui()
    assert create.call_count == 3
    for call in create.call_args_list:
        assert call.args[1] == 'Mission Tuner'
    for name, (_, _, decode, encode) in tuner.trackbars.items():
        assert decode(encode(tuner.current_values[name])) == tuner.current_values[name]
    tuner._set_future = None
    tuner.managed_parameters = tuple(tuner.trackbars)
    tuner.loaded_config_path = ''
    tuner.parameter_root = 'mission_controller_node'
    tuner.save_path = tmp_path/'mission.yaml'
    tuner.get_logger = Mock(return_value=Mock())
    tuner.save()
    saved = _validate_parameter_file(tuner.save_path, tuner.parameter_root, _profile('mission')[3])
    assert saved == tuner.current_values


def test_debug_image_shows_effective_gain_and_remaining_time():
    node = bare_controller(MissionControllerNode, 250)
    node.mission.toggle_target_lane(10)
    prepare(node, 10.5)
    frame = np.zeros((480, 640, 3), np.uint8)
    path = PathPlanningResult([320, 320], [20, 160])
    node.adjust_command(MotionCommand(1, 250, 250), DetectionArray(), path, frame)
    with patch.object(TrackControllerNode, '_make_debug', return_value=frame), \
         patch('skku_track_drive_pkg.modes.bbox_mission.mission_controller_node.add_traffic_light_panel', return_value=frame):
        image = node._make_debug(frame, DetectionArray())
    assert image.shape == (508, 640, 3)
    assert np.any(image[:28] != 20)  # Text is drawn into the gain status band.
    np.testing.assert_array_equal(image[28:], frame)
