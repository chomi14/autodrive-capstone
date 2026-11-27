"""One camera-parking-only YAML schema; never shares legacy numeric tuning."""
from dataclasses import fields
from pathlib import Path
import copy
import math
import os
import tempfile

import yaml

from .core import ControlConfig

CAPTURE_FIELDS = ('device', 'width', 'height', 'fps')
TUNING_TOPIC = '/parking_camera/tuning'
MODEL_TOPIC = '/parking_camera/model'
PARALLEL_BEV_FIELDS = ('persp_src', 'persp_dst')
VISION_FIELDS = ('rotate_deg', 'flip_horizontal', 'confidence', 'persp_src', 'persp_dst',
                 'min_area_ratio', 'morph_kernel', 'min_longitudinal_span_ratio')
FRONT_LIMIT_DEFAULTS = {
    'limit_x_min': .32, 'limit_x_max': .68, 'limit_y_min': .45, 'limit_y_max': .92,
    'limit_white_min': 165, 'limit_min_span': .12,
}
FRONT_VISION_FIELDS = tuple(FRONT_LIMIT_DEFAULTS)
REAR_VISION_FIELDS = ('roi_band_ratio', 'parking_width_ratio', 'pair_min_separation_ratio',
                      'pair_max_angle_rad')
LEGACY_CONTROL_FIELDS = ('confirm_frames', 'align_confirm_frames', 'align_yaw_min_rad',
                         'align_yaw_error_max_rad', 'outline_finish_y',
                         'final_straight_s', 'exit_position_gain', 'exit_angle_gain')


def _finite(values):
    return all(type(x) in (int, float) and math.isfinite(x) for x in values)


def load_config(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    validate_config(data)
    for key in LEGACY_CONTROL_FIELDS:
        data['control'].pop(key, None)
    data['front'] = {**FRONT_LIMIT_DEFAULTS, **data['front']}
    for item in fields(ControlConfig):
        data['control'].setdefault(item.name, item.default)
    data['rear'].setdefault('roi_band_ratio', 0.0)
    data.setdefault('parallel', {'model_path': ''})
    return data


def _validate_perspective(view, role):
    for key in PARALLEL_BEV_FIELDS:
        points = view[key]
        if not isinstance(points, list) or len(points) != 8 or not _finite(points) or not all(0 <= x <= 1 for x in points):
            raise ValueError(f'{role}.{key}: eight normalized finite coordinates required')
        xy = list(zip(points[::2], points[1::2]))
        crosses = []
        for i in range(4):
            a, b, c = xy[i], xy[(i+1) % 4], xy[(i+2) % 4]
            crosses.append((b[0]-a[0])*(c[1]-b[1]) - (b[1]-a[1])*(c[0]-b[0]))
        if not (all(x > 1e-5 for x in crosses) or all(x < -1e-5 for x in crosses)):
            raise ValueError(f'{role}.{key}: corners must be convex and ordered')
        if key == 'persp_src':
            source_cross = crosses[0]
        elif source_cross*crosses[0] < 0:
            raise ValueError(f'{role}: BEV source and destination must have the same corner order')


def validate_config(data):
    required = {'control', 'front', 'rear', 'save_path'}
    if not isinstance(data, dict) or not required <= set(data) <= required | {'parallel'}:
        raise ValueError('Camera parking YAML requires control/front/rear/save_path and optionally parallel')
    control = ControlConfig(**{key: value for key, value in data['control'].items()
                               if key not in LEGACY_CONTROL_FIELDS})
    if not isinstance(data['save_path'], str) or not data['save_path']:
        raise ValueError('save_path must be a nonempty string')
    keys = {'device', 'image_topic', 'model_path', 'model_device', 'confidence',
            'width', 'height', 'fps', 'rotate_deg', 'flip_horizontal', 'classes',
            'persp_src', 'persp_dst', 'min_area_ratio', 'morph_kernel',
            'parking_width_ratio', 'pair_min_separation_ratio', 'pair_max_angle_rad',
            'min_longitudinal_span_ratio'}
    for role in ('front', 'rear'):
        view = data[role]
        optional = {'roi_band_ratio'} if role == 'rear' else set(FRONT_VISION_FIELDS)
        if not isinstance(view, dict) or not keys <= set(view) <= keys | optional:
            raise ValueError(f'{role}: missing/unknown camera configuration keys')
        band = view.get('roi_band_ratio', 0.0)
        if not _finite([band]) or not 0 <= band < 1:
            raise ValueError('rear.roi_band_ratio must be in [0,1)')
        for key in ('device', 'image_topic', 'model_device'):
            if not isinstance(view[key], str) or not view[key]:
                raise ValueError(f'{role}.{key} must be a nonempty string')
        if not isinstance(view['model_path'], str):
            raise ValueError('model_path must be a string (empty until model acquired)')
        for key in ('width', 'height', 'morph_kernel'):
            if type(view[key]) is not int or view[key] <= 0:
                raise ValueError(f'{role}.{key} must be a positive integer')
        if view['morph_kernel'] % 2 != 1:
            raise ValueError('morph_kernel must be odd')
        if type(view['rotate_deg']) is not int or view['rotate_deg'] not in (0, 90, 180, 270):
            raise ValueError('rotate_deg must be 0/90/180/270 clockwise')
        if type(view['flip_horizontal']) is not bool:
            raise ValueError('flip_horizontal must be boolean')
        for key in ('confidence', 'min_area_ratio', 'parking_width_ratio',
                    'pair_min_separation_ratio', 'min_longitudinal_span_ratio'):
            if not _finite([view[key]]) or not 0 < view[key] < 1:
                raise ValueError(f'{role}.{key} must be in (0,1)')
        if not _finite([view['fps'], view['pair_max_angle_rad']]) or view['fps'] <= 0 or not 0 < view['pair_max_angle_rad'] < math.pi / 2:
            raise ValueError('Invalid fps or line pair angular tolerance')
        _validate_perspective(view, role)
        expected = {'parking_lot', 'parking_space', 'out_line'} if role == 'front' else {'parking_space', 'end_line'}
        if not isinstance(view['classes'], dict) or set(view['classes']) != expected:
            raise ValueError(f'{role}.classes must map {sorted(expected)} to model class NAMES')
        names = list(view['classes'].values())
        if any(not isinstance(x, str) or not x for x in names) or len(set(names)) != len(names):
            raise ValueError('Class names must be nonempty and unique')
    front = {**FRONT_LIMIT_DEFAULTS, **data['front']}
    x_min, x_max, y_min, y_max, white, span = (front[key] for key in FRONT_VISION_FIELDS)
    if (not _finite([x_min, x_max, y_min, y_max, span]) or not 0 <= x_min < x_max <= 1
            or not 0 <= y_min < control.front_limit_stop_y < y_max <= 1
            or not 0 < span < x_max - x_min or type(white) is not int or not 1 <= white <= 255):
        raise ValueError('Invalid front limit ROI, white level, span or stop row')
    if data['front']['image_topic'] == data['rear']['image_topic']:
        raise ValueError('Front/rear image topics must differ')
    parallel = data.get('parallel', {'model_path': ''})
    if (not isinstance(parallel, dict) or set(parallel) - {'model_path', 'front', 'rear'}
            or not isinstance(parallel.get('model_path'), str)):
        raise ValueError('parallel requires model_path and permits camera BEV overrides')
    for role in ('front', 'rear'):
        override = parallel.get(role, {})
        if not isinstance(override, dict) or set(override) - set(PARALLEL_BEV_FIELDS):
            raise ValueError(f'parallel.{role} supports persp_src and persp_dst overrides')
        _validate_perspective(dict(data[role], **override), f'parallel.{role}')


def save_config_atomic(path, data, control):
    """Persist the entire schema, including camera strings/lists; no share writes."""
    values = copy.deepcopy(data)
    values['control'] = {item.name: getattr(control, item.name) for item in fields(control)}
    validate_config(values)
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=destination.parent, prefix='.camera_parking_', suffix='.yaml')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(values, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
