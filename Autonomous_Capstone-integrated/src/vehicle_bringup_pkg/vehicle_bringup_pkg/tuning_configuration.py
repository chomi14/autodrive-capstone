"""Shared launch helpers for layered track-tuning parameter files."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from skku_track_drive_pkg.bev_geometry import BEV_SOURCE_DEFAULTS


TUNING_TYPES = {
    **{name: int for name in BEV_SOURCE_DEFAULTS},
    'speed': int,
    'stanley_gain': float,
    'heading_gain': float,
    'lookahead_index': int,
    'confidence': float,
    'bev_top_shift': int,
    'roi_cut': int,
    'look_shift': int,
    'ema_alpha': float,
    'virtual_lane_width': int,
    'max_steering_angle': float,
}
ADVANCED_TUNING_PARAMETERS = {
    'stanley_softening',
    'heading_step',
    'bev_pad',
    'car_center_x',
    'car_center_y',
    'center_ema_alpha',
    'max_center_jump_px',
    'max_missed_frames',
    'min_component_area',
}
ALLOWED_TUNING_PARAMETERS = set(TUNING_TYPES) | ADVANCED_TUNING_PARAMETERS
COMMON_TRACK_PARAMETERS = ALLOWED_TUNING_PARAMETERS - {'speed'}


def _profile(mode):
    if mode == 'track':
        return 'track_controller_node', 'track_tuning.yaml', TUNING_TYPES, ALLOWED_TUNING_PARAMETERS
    from skku_track_drive_pkg.mode_parameters import MISSION, PERPENDICULAR, PARALLEL
    if mode == 'mission':
        types = {**TUNING_TYPES, **{key: type(spec[0]) for key, spec in MISSION.items()}}
        return 'mission_controller_node', 'mission_tuning.yaml', types, ALLOWED_TUNING_PARAMETERS | set(MISSION)
    if mode == 'calibration':
        from skku_track_drive_pkg.calibration_controller_node import CALIBRATION
        types = {key: type(spec[0]) for key, spec in CALIBRATION.items()}
        return 'parking_calibration_controller_node', 'parking_calibration.yaml', types, set(types)
    specs = {'perpendicular': PERPENDICULAR, 'parallel': PARALLEL}[mode]
    types = {key: type(spec[0]) for key, spec in specs.items()}
    return f'{mode}_parking_controller_node', f'{mode}_parking.yaml', types, set(specs)


def tuning_launch_arguments(mode='track'):
    _root, filename, types, _allowed = _profile(mode)
    arguments = [
        DeclareLaunchArgument(
            'tuning_config',
            default_value='',
            description=(
                'Explicit tuning YAML. Empty selects the saved user YAML when it exists, '
                'otherwise the package default.'
            ),
        ),
        DeclareLaunchArgument(
            'saved_tuning_path',
            default_value=str(Path('~/.config/autodrive', filename).expanduser()),
            description='P-key save path and automatically loaded user tuning YAML',
        ),
    ]
    arguments.extend(
        DeclareLaunchArgument(
            name,
            default_value='',
            description=f'Optional explicit override for tuning parameter {name}',
        )
        for name in types
    )
    return arguments


def _validate_parameter_file(path, root='track_controller_node', allowed=None):
    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    try:
        parameters = data[root]['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f'{path}: expected {root}.ros__parameters mapping; another mode cannot be loaded/overwritten'
        ) from exc
    if not isinstance(parameters, dict):
        raise ValueError(f'{path}: ros__parameters must be a mapping')
    unknown = sorted(set(parameters) - (ALLOWED_TUNING_PARAMETERS if allowed is None else allowed))
    if unknown:
        raise ValueError(
            f'{path}: non-tuning parameters are not allowed: {", ".join(unknown)}'
        )
    return parameters


def resolve_tuning(context, mode='track'):
    root, filename, types, allowed = _profile(mode)
    default_path = Path(
        get_package_share_directory('vehicle_bringup_pkg'),
        'config',
        filename,
    )
    saved_path = Path(
        LaunchConfiguration('saved_tuning_path').perform(context)
    ).expanduser()
    explicit_text = LaunchConfiguration('tuning_config').perform(context).strip()

    if not default_path.is_file():
        raise RuntimeError(f'Package default tuning config not found: {default_path}')
    default_parameters = _validate_parameter_file(default_path, root, allowed)
    if saved_path.is_file():
        _validate_parameter_file(saved_path, root, allowed)

    if explicit_text:
        selected_path = Path(explicit_text).expanduser()
        source = 'explicit'
        if not selected_path.is_file():
            raise RuntimeError(f'Explicit tuning config not found: {selected_path}')
    elif saved_path.is_file():
        selected_path = saved_path
        source = 'saved user'
    else:
        selected_path = default_path
        source = 'package default'

    selected_parameters = _validate_parameter_file(selected_path, root, allowed)
    parameters = dict(default_parameters)
    parameters.update(selected_parameters)
    parameters['loaded_tuning_config'] = str(selected_path)
    for name, value_type in types.items():
        raw_value = LaunchConfiguration(name).perform(context).strip()
        if not raw_value:
            continue
        try:
            parameters[name] = value_type(raw_value)
        except ValueError as exc:
            raise ValueError(f'Invalid launch override {name}: {raw_value}') from exc
    return parameters, selected_path, saved_path, source
