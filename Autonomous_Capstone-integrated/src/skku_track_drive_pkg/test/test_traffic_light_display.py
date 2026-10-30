"""Raw-pixel diagnostics and camera layout; no ROS nodes or hardware started."""
from copy import deepcopy
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from skku_track_drive_pkg.mode_parameters import MISSION, defaults
from skku_track_drive_pkg.traffic_light_display import describe_traffic_boxes, add_traffic_light_panel
from skku_track_drive_pkg.track_controller_node import TrackControllerNode


def status(box=(10, 10, 30, 30), eligible=True):
    return {'traffic_color': 'Unknown', 'red_latched': False, 'traffic_boxes': [
        {'box': list(box), 'eligible': eligible, 'used_for_color': False, 'color': 'Unknown'}]}


@pytest.mark.parametrize('bgr,rgb,hsv', [
    ((0, 0, 255), [255, 0, 0], [0, 255, 255]),
    ((0, 255, 0), [0, 255, 0], [60, 255, 255]),
    ((0, 255, 255), [255, 255, 0], [30, 255, 255]),
])
def test_raw_rgb_hsv_and_coordinates_without_mutating_decision(bgr, rgb, hsv):
    frame = np.zeros((480, 640, 3), np.uint8)
    frame[10:30, 10:30] = bgr
    evidence = status()
    before = deepcopy(evidence)
    item, = describe_traffic_boxes(frame, evidence, defaults(MISSION))
    assert evidence == before
    assert item['rgb_mean'] == item['rgb_sample'] == rgb
    assert item['hsv_sample'] == hsv
    x, y = item['sample_xy']
    assert np.array_equal(frame[y, x, ::-1], rgb)
    assert item['sv_valid_ratio'] == 1


def test_no_qualified_pixels_still_shows_brightest_sample_and_rejected_box_reason():
    frame = np.full((480, 640, 3), 200, np.uint8)
    evidence = status(box=(10, 61, 18, 80), eligible=False)
    item, = describe_traffic_boxes(frame, evidence, defaults(MISSION))
    assert item['rgb_sample'] == [200, 200, 200]
    assert item['hsv_sample'] == [0, 0, 200]
    assert item['sv_valid_ratio'] == 0
    assert 'narrow' in item['sample_reason'] and 'bottom' in item['sample_reason']
    item, = describe_traffic_boxes(frame, status(), defaults(MISSION))
    assert 'No pixels pass S/V' in item['sample_reason']


def test_red_hue_wrap_is_not_averaged_and_outside_box_is_empty():
    hsv = np.zeros((480, 640, 3), np.uint8)
    hsv[10:30, 10:30] = (179, 255, 255)
    hsv[10:20, 10:30] = (0, 255, 255)
    frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    item, = describe_traffic_boxes(frame, status(), defaults(MISSION))
    assert item['hsv_sample'][0] in (0, 179)
    assert item['ratios']['Red'] == 1
    item, = describe_traffic_boxes(frame, status(box=(-30, -30, -10, -10)), defaults(MISSION))
    assert item['sample_reason'] == 'Empty / outside image'
    assert 'rgb_sample' not in item


@pytest.mark.parametrize('width', [320, 640, 1280])
def test_panel_preserves_image_and_status(width):
    image = np.full((480, width, 3), 75, np.uint8)
    evidence = status()
    evidence['traffic_boxes'] = describe_traffic_boxes(image, evidence, defaults(MISSION))
    before = deepcopy(evidence)
    rendered = add_traffic_light_panel(image, evidence, camera_frame=image)
    assert evidence == before
    assert rendered.shape[1] == width
    assert np.array_equal(rendered[-480:], image)
    assert np.all(image == 75)


@pytest.mark.parametrize('bev', [False, True])
def test_driving_header_does_not_shade_camera_pixels(bev):
    frame = np.full((480, 640, 3), 175, np.uint8)
    node = SimpleNamespace(
        enabled=True, publish_bev_debug=bev,
        lane=SimpleNamespace(last_debug=None, _src_mat=lambda: [[238, 316], [402, 313], [501, 476], [155, 476]]),
        motion=SimpleNamespace(last_heading_deg=1., last_cte=2.),
        _lane_polygons=lambda *args: [], _draw_detection=lambda *args: None,
        _draw_bev_source=lambda *args: None,
        _make_bev_debug=lambda *args: np.zeros_like(frame))
    lane = SimpleNamespace(source='observed', confidence=1.)
    command = SimpleNamespace(steering=1, left_speed=80, right_speed=80)
    rendered = TrackControllerNode._make_debug(node, frame, None, lane, None, command, 30., True, True)
    assert rendered.shape == (584, 1280 if bev else 640, 3)
    assert np.array_equal(rendered[104:, :640], frame)
    assert np.any(rendered[:104] != 20)
