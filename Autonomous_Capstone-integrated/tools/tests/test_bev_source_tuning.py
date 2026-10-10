"""Camera-space calibration can change live without retaining an old BEV path."""

from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
from rclpy.parameter import Parameter

from skku_track_drive_pkg.bev_geometry import BEV_DEFAULT_POINTS
from skku_track_drive_pkg.lane_processing import LaneInfoExtractor
from skku_track_drive_pkg.messages import Detection, DetectionArray, Mask
from skku_track_drive_pkg.track_controller_node import TrackControllerNode


def controller():
    node = object.__new__(TrackControllerNode)
    node.lane = LaneInfoExtractor(show_image=False, capture_debug=True, bev_top_shift=-8)
    node._parameter_lock = RLock()
    node.allow_speed_tuning = True
    node.speed = 80
    node.get_logger = Mock(return_value=Mock())
    return node


def lane_detection():
    bitmap = np.zeros((480, 640), dtype=np.uint8)
    cv2.fillConvexPoly(bitmap, np.array(BEV_DEFAULT_POINTS, dtype=np.int32), 255)
    return DetectionArray([
        Detection(class_name='lane2', mask=Mask(bitmap=bitmap, width=640, height=480))
    ])


def test_live_points_change_the_homography_and_debug_uses_effective_coordinates():
    node = controller()
    detections = lane_detection()
    node.lane.process(detections)
    before = node.lane.last_debug['inverse_transform'].copy()

    result = node.on_tuning_parameters([
        Parameter('bev_src_tl_x', value=210),
        Parameter('bev_src_tr_y', value=330),
    ])
    assert result.successful
    node.lane.process(detections)

    assert node.lane._src_mat() == [[210, 308], [402, 322], [501, 476], [155, 476]]
    np.testing.assert_array_equal(node.lane.last_debug['source_points'], node.lane._src_mat())
    assert not np.allclose(before, node.lane.last_debug['inverse_transform'])


def test_invalid_shape_rejects_the_whole_update_and_keeps_previous_points():
    node = controller()
    before = node.lane._src_mat()
    for changes in (
        {'bev_src_tl_x': 500, 'speed': 100},
        {'bev_src_tl_y': 479},
        {'bev_src_tl_y': 0},  # The existing top shift would move it out of frame.
        {'bev_src_bl_x': 550},
    ):
        result = node.on_tuning_parameters([
            Parameter(name, value=value) for name, value in changes.items()
        ])
        assert not result.successful
        assert node.lane._src_mat() == before
        assert node.speed == 80


def test_old_lane_is_not_held_after_source_geometry_changes():
    node = controller()
    assert node.lane.process(lane_detection()).valid
    assert node.lane.process(DetectionArray()).valid
    assert node.on_tuning_parameters([Parameter('bev_src_tl_x', value=220)]).successful

    result = node.lane.process(DetectionArray())

    assert not result.valid
    assert not result.target_points
    assert node.lane.smoothed_fit is None


def test_camera_overlay_is_present_without_detections_or_bev_panel():
    node = controller()
    node.enabled = node.publish_bev_debug = False
    node.motion = SimpleNamespace(last_heading_deg=0., last_cte=0.)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    labels = []
    put_text = cv2.putText

    def capture_label(image, label, *args, **kwargs):
        labels.append(label)
        return put_text(image, label, *args, **kwargs)

    with patch.object(cv2, 'putText', side_effect=capture_label):
        rendered = node._make_debug(
            frame, DetectionArray(), SimpleNamespace(target_points=[], source='invalid', confidence=0.),
            SimpleNamespace(x_points=[], y_points=[]),
            SimpleNamespace(steering=0, left_speed=0, right_speed=0), 0., False, False,
        )

    assert rendered.shape == frame.shape
    assert 'P1 TL (238,308)' in labels
    assert 'P2 TR (402,305)' in labels
    assert 'P3 BR (501,476)' in labels
    assert 'P4 BL (155,476)' in labels
    assert tuple(rendered[308, 238]) == (0, 165, 255)
    assert not np.any(frame)  # Rendering leaves the camera input unchanged.
