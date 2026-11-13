"""Synthetic geometric fixtures verify BEV/image math, not trained accuracy."""
import copy
from pathlib import Path
import math

import cv2
import numpy as np
import pytest

from skku_track_drive_pkg.modes.perpendicular_camera.configuration import load_config, save_config_atomic, validate_config
from skku_track_drive_pkg.modes.perpendicular_camera.core import ControlConfig
from skku_track_drive_pkg.modes.perpendicular_camera.model import model_path
from skku_track_drive_pkg.modes.perpendicular_camera.observations import axis_error
from skku_track_drive_pkg.modes.perpendicular_camera.perception import extract, orient_image, rear_center

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'src/vehicle_bringup_pkg/config/perpendicular_camera.yaml'
IDENTITY = [0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0]


def cfg(role):
    result = copy.deepcopy(load_config(CONFIG)[role])
    result['persp_src'] = result['persp_dst'] = IDENTITY
    return result


def rectangle(x1, y1, x2, y2, w, h):
    return np.float32([[x1*w, y1*h], [x2*w, y1*h], [x2*w, y2*h], [x1*w, y2*h]])


@pytest.mark.parametrize('w,h', [(640, 480), (1280, 960), (320, 240)])
def test_normalized_front_edge_and_end_stripe_across_resolutions(w, h):
    frame = np.full((h, w, 3), 40, np.uint8)
    front, seg, bev = extract(frame, {'parking_lot': [rectangle(.2, .1, .8, .95, w, h)]}, cfg('front'), 'front', 123)
    assert front.line.found and abs(front.line.x-.2) < .01
    assert abs(axis_error(front.line.yaw)) < .01
    rear, _, _ = extract(frame, {'end_line': [rectangle(.25, .79, .75, .82, w, h)]}, cfg('rear'), 'rear', 124)
    assert rear.end.found and abs(rear.end.y-.82) < .01
    assert seg.shape == bev.shape == frame.shape


@pytest.mark.parametrize('w,h', [(640, 480), (1280, 960)])
def test_rear_two_painted_boundaries_produce_center(w, h):
    frame = np.full((h, w, 3), 40, np.uint8)
    for x in (.3, .7):
        cv2.line(frame, (int(x*w), int(.1*h)), (int(x*w), h-1), (255, 255, 255), max(3, w//160))
    roi = np.zeros((h, w), np.uint8)
    cv2.fillPoly(roi, [rectangle(.2, 0, .8, 1, w, h).astype(np.int32)], 255)
    line = rear_center(frame, roi, cfg('rear'))
    assert line.found and abs(line.x-.5) < .025
    assert abs(axis_error(line.yaw)) < .05


def test_single_right_boundary_uses_configured_width_and_empty_roi_fails():
    frame = np.full((480, 640, 3), 40, np.uint8)
    cv2.line(frame, (448, 20), (448, 479), (255, 255, 255), 4)
    roi = np.zeros((480, 640), np.uint8)
    roi[:, 128:512] = 255
    line = rear_center(frame, roi, cfg('rear'))
    assert line.found and abs(line.x-.5) < .025
    assert not rear_center(frame, roi*0, cfg('rear')).found


def test_no_visual_edges_does_not_invent_center_from_space_mask():
    frame = np.full((480, 640, 3), 40, np.uint8)
    obs, _, _ = extract(frame, {'parking_space': [rectangle(.2, .1, .8, .9, 640, 480)]}, cfg('rear'), 'rear', 123)
    assert obs.space_found and not obs.space.found


def test_rotation_then_flip_is_explicit_and_applies_before_geometry():
    frame = np.zeros((2, 3, 3), np.uint8)
    frame[0, 0] = (1, 2, 3)
    result = orient_image(frame, 90, True)
    assert result.shape == (3, 2, 3)
    assert tuple(result[0, 0]) == (1, 2, 3)


def test_save_load_preserves_whole_camera_schema(tmp_path):
    data = load_config(CONFIG)
    data['rear']['rotate_deg'] = 180
    data['front']['model_path'] = '/tmp/my front.pt'
    path = tmp_path/'settings.yaml'
    save_config_atomic(path, data, ControlConfig(**data['control']))
    assert load_config(path) == data


@pytest.mark.parametrize('mutation', ['classes', 'bev', 'types', 'topic'])
def test_invalid_camera_settings_fail(mutation):
    data = load_config(CONFIG)
    if mutation == 'classes':
        data['rear']['classes']['end_line'] = 'parking_space'
    elif mutation == 'bev':
        data['rear']['persp_src'] = [0.0]*8
    elif mutation == 'types':
        data['rear']['flip_horizontal'] = 'false'
    else:
        data['rear']['image_topic'] = data['front']['image_topic']
    with pytest.raises(ValueError):
        validate_config(data)


def test_missing_model_is_a_clear_error_and_no_weights_are_downloaded(tmp_path):
    with pytest.raises(FileNotFoundError, match='Parking segmentation model missing'):
        model_path('')
    with pytest.raises(FileNotFoundError):
        model_path(str(tmp_path/'missing.pt'))
