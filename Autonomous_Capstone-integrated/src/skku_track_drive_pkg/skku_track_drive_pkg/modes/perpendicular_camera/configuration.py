"""One camera-parking-only YAML schema; never shares legacy numeric tuning."""
from dataclasses import fields
from pathlib import Path
import copy
import math
import os
import tempfile

import yaml

from .core import ControlConfig


def _finite(values):
    return all(type(x) in (int, float) and math.isfinite(x) for x in values)


def load_config(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    validate_config(data)
    return data


def validate_config(data):
    if not isinstance(data, dict) or set(data) != {'control', 'front', 'rear', 'save_path'}:
        raise ValueError('Camera parking YAML requires control/front/rear/save_path only')
    ControlConfig(**data['control'])
    if not isinstance(data['save_path'], str) or not data['save_path']:
        raise ValueError('save_path must be a nonempty string')
    keys = {'device', 'image_topic', 'model_path', 'model_device', 'confidence',
            'width', 'height', 'fps', 'rotate_deg', 'flip_horizontal', 'classes',
            'persp_src', 'persp_dst', 'min_area_ratio', 'morph_kernel',
            'parking_width_ratio', 'pair_min_separation_ratio', 'pair_max_angle_rad',
            'min_longitudinal_span_ratio'}
    for role in ('front', 'rear'):
        view = data[role]
        if not isinstance(view, dict) or set(view) != keys:
            raise ValueError(f'{role}: missing/unknown camera configuration keys')
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
        for key in ('persp_src', 'persp_dst'):
            points = view[key]
            if not isinstance(points, list) or len(points) != 8 or not _finite(points) or not all(0 <= x <= 1 for x in points):
                raise ValueError(f'{role}.{key}: eight normalized finite coordinates required')
            # Consecutive corners must form a nondegenerate convex quadrilateral.
            xy = list(zip(points[::2], points[1::2]))
            crosses = []
            for i in range(4):
                a, b, c = xy[i], xy[(i+1) % 4], xy[(i+2) % 4]
                crosses.append((b[0]-a[0])*(c[1]-b[1]) - (b[1]-a[1])*(c[0]-b[0]))
            if not (all(x > 1e-5 for x in crosses) or all(x < -1e-5 for x in crosses)):
                raise ValueError(f'{role}.{key}: corners must be convex and ordered')
        expected = {'parking_lot', 'parking_space', 'out_line'} if role == 'front' else {'parking_space', 'end_line'}
        if not isinstance(view['classes'], dict) or set(view['classes']) != expected:
            raise ValueError(f'{role}.classes must map {sorted(expected)} to model class NAMES')
        names = list(view['classes'].values())
        if any(not isinstance(x, str) or not x for x in names) or len(set(names)) != len(names):
            raise ValueError('Class names must be nonempty and unique')
    if data['front']['image_topic'] == data['rear']['image_topic']:
        raise ValueError('Front/rear image topics must differ')


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
