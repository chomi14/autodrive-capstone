"""Vehicle coordinates stay fixed in full BEV while the ROI origin changes."""
from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
import pytest
from rclpy.parameter import Parameter

from skku_track_drive_pkg.bbox_mission_controller_node import BboxMissionControllerNode
from skku_track_drive_pkg.lane_processing import LaneInfoExtractor
from skku_track_drive_pkg.messages import DetectionArray, PathPlanningResult
from skku_track_drive_pkg.mission_controller_node import MissionControllerNode
from skku_track_drive_pkg.mission_core import MissionCore
from skku_track_drive_pkg.motion_planner import MotionPlanner
from skku_track_drive_pkg.path_planner import PathPlanner
from skku_track_drive_pkg.track_controller_node import TrackControllerNode
from skku_track_drive_pkg.track_tuner_node import TrackTunerNode
from vehicle_bringup_pkg.tuning_configuration import _validate_parameter_file


def controller(cls=TrackControllerNode, roi=300, car_y=179., reference_roi=None):
    node = object.__new__(cls)
    node.lane = LaneInfoExtractor(show_image=False, capture_debug=True, roi_cut=roi)
    node._initialize_vehicle_reference(car_y, roi if reference_roi is None else reference_roi)
    node.motion = MotionPlanner(car_center_x=325., car_center_y=node._vehicle_y_in_roi(roi),
                                stanley_gain=.02, heading_gain=.6, heading_step=3,
                                max_steering=7., max_steering_angle_deg=50.,
                                lookahead_steps=10, stop_on_red=False)
    node.path_planner = PathPlanner((node.motion.car_center_x, node.motion.car_center_y))
    node._parameter_lock = RLock()
    node.allow_speed_tuning = True
    node.enabled = False
    node.publish_bev_debug = True
    node.get_logger = Mock(return_value=Mock())
    if cls is not TrackControllerNode:
        node.mission = MissionCore()
    return node


@pytest.mark.parametrize('cls', [TrackControllerNode, MissionControllerNode, BboxMissionControllerNode])
def test_roi_updates_keep_full_vehicle_position_and_stanley_geometry(cls):
    node = controller(cls)
    # The same physical path, expressed in each crop's local coordinates.
    full_y = np.linspace(471., 479., 100)
    full_x = 280. + .3 * (full_y - 400.)
    baseline = None
    for roi in (300, 250, 0, 470, 350, 300):
        assert node.on_tuning_parameters([Parameter('roi_cut', value=roi)]).successful
        assert node.motion.car_center_y == 479. - roi
        assert node.path_planner.car_center_point == (325., 479. - roi)
        path = PathPlanningResult(full_x.tolist(), (full_y - roi).tolist())
        command = node.motion.plan(DetectionArray(), path)
        current = (node.motion.last_cte, node.motion.last_heading_deg, command.steering)
        if baseline is None:
            baseline = current
        assert current == pytest.approx(baseline, abs=1e-9)


def test_camera_and_bev_vehicle_markers_do_not_follow_roi_cut():
    node = controller()
    frame = np.zeros((480, 640, 3), np.uint8)
    markers = []
    draw_marker = cv2.drawMarker

    def capture_marker(image, point, *args, **kwargs):
        markers.append(tuple(point))
        return draw_marker(image, point, *args, **kwargs)

    baseline = None
    for roi in (300, 250, 100, 470):
        assert node.on_tuning_parameters([Parameter('roi_cut', value=roi)]).successful
        info = node.lane.process(DetectionArray())
        markers.clear()
        with patch.object(cv2, 'drawMarker', side_effect=capture_marker):
            node._make_debug(frame, DetectionArray(), info, PathPlanningResult(),
                             SimpleNamespace(steering=0, left_speed=0, right_speed=0),
                             0., False, False)
        assert len(markers) == 2
        assert markers[1] == (325, 479)
        if baseline is None:
            baseline = list(markers)
        assert markers == baseline

    # A parameter callback can run after extraction but before rendering.
    # Even with the previous frame's crop metadata, draw the fixed BEV point.
    assert node.on_tuning_parameters([Parameter('roi_cut', value=250)]).successful
    markers.clear()
    with patch.object(cv2, 'drawMarker', side_effect=capture_marker):
        node._make_debug(frame, DetectionArray(), info, PathPlanningResult(),
                         SimpleNamespace(steering=0, left_speed=0, right_speed=0),
                         0., False, False)
    assert markers == baseline


def test_legacy_initial_tuning_is_preserved_and_explicit_vehicle_update_is_applied():
    node = controller(roi=250, car_y=160.)
    assert node.motion.car_center_y == 160.
    assert node.on_tuning_parameters([Parameter('roi_cut', value=300)]).successful
    assert node.motion.car_center_y == 110.
    assert node.on_tuning_parameters([
        Parameter('roi_cut', value=200), Parameter('car_center_y', value=170.),
    ]).successful
    assert node.motion.car_center_y == 220.
    assert node.path_planner.car_center_point == (325., 220.)


def test_invalid_roi_batch_does_not_move_vehicle_reference():
    node = controller()
    result = node.on_tuning_parameters([
        Parameter('roi_cut', value=471), Parameter('car_center_y', value=120.),
    ])
    assert not result.successful
    assert node.lane.roi_cut == 300
    assert node.motion.car_center_y == 179.
    assert node._vehicle_y_in_roi(300) == 179.


def test_gui_saves_reference_and_reload_keeps_vehicle_at_same_full_bev_y(tmp_path):
    parameters = {}

    def declare(node, name, value):
        parameters[name] = str(tmp_path/'track.yaml') if name == 'save_path' else value

    with patch('skku_track_drive_pkg.track_tuner_node.Node.__init__'), \
         patch.object(TrackTunerNode, 'declare_parameter', declare), \
         patch.object(TrackTunerNode, 'get_parameter', lambda node, name: SimpleNamespace(value=parameters[name])), \
         patch.object(TrackTunerNode, 'get_logger', return_value=Mock()), \
         patch.object(TrackTunerNode, 'create_publisher', return_value=Mock()), \
         patch.object(TrackTunerNode, 'create_subscription', return_value=Mock()), \
         patch.object(TrackTunerNode, 'create_client', return_value=Mock()), \
         patch.object(TrackTunerNode, 'create_timer', return_value=Mock()):
        tuner = TrackTunerNode()
    assert {'car_center_y', 'car_center_reference_roi_cut'} <= set(tuner.managed_parameters)
    # Simulate accepted service values after moving ROI, then exercise P/save.
    tuner.managed_parameters = ('roi_cut', 'car_center_y', 'car_center_reference_roi_cut')
    tuner.trackbars = {'roi_cut': None}
    tuner.current_values = tuner.desired_values = {
        'roi_cut': 250, 'car_center_y': 179., 'car_center_reference_roi_cut': 300,
    }
    tuner.get_logger = Mock(return_value=Mock())
    tuner.save()
    saved = _validate_parameter_file(tuner.save_path)
    restarted = controller(roi=saved['roi_cut'], car_y=saved['car_center_y'],
                           reference_roi=saved['car_center_reference_roi_cut'])
    assert restarted.motion.car_center_y == 229.
    assert restarted.motion.car_center_y + restarted.lane.roi_cut == 479.
