"""Shared launch helpers for layered track-tuning parameter files."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


TUNING_TYPES = {
    'speed': int,
    'stanley_gain': float,
    'heading_gain': float,
    'lookahead_index': int,
    'confidence': float,
    'bev_top_shift': int,
    'look_shift': int,
    'ema_alpha': float,
    'virtual_lane_width': int,
    'max_steering_angle': float,
}
ADVANCED_TUNING_PARAMETERS = {
    'stanley_softening',
    'heading_step',
    'roi_cut',
    'bev_pad',
    'car_center_x',
    'car_center_y',
}
ALLOWED_TUNING_PARAMETERS = set(TUNING_TYPES) | ADVANCED_TUNING_PARAMETERS


def tuning_launch_arguments():
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
            default_value=str(Path('~/.config/autodrive/track_tuning.yaml').expanduser()),
            description='P-key save path and automatically loaded user tuning YAML',
        ),
    ]
    arguments.extend(
        DeclareLaunchArgument(
            name,
            default_value='',
            description=f'Optional explicit override for tuning parameter {name}',
        )
        for name in TUNING_TYPES
    )
    return arguments


def _validate_parameter_file(path):
    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    try:
        parameters = data['track_controller_node']['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f'{path}: expected track_controller_node.ros__parameters mapping'
        ) from exc
    if not isinstance(parameters, dict):
        raise ValueError(f'{path}: ros__parameters must be a mapping')
    unknown = sorted(set(parameters) - ALLOWED_TUNING_PARAMETERS)
    if unknown:
        raise ValueError(
            f'{path}: non-tuning parameters are not allowed: {", ".join(unknown)}'
        )
    return parameters


def resolve_tuning(context):
    default_path = Path(
        get_package_share_directory('vehicle_bringup_pkg'),
        'config',
        'track_tuning.yaml',
    )
    saved_path = Path(
        LaunchConfiguration('saved_tuning_path').perform(context)
    ).expanduser()
    explicit_text = LaunchConfiguration('tuning_config').perform(context).strip()

    if not default_path.is_file():
        raise RuntimeError(f'Package default tuning config not found: {default_path}')
    default_parameters = _validate_parameter_file(default_path)

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

    selected_parameters = _validate_parameter_file(selected_path)
    parameters = dict(default_parameters)
    parameters.update(selected_parameters)
    parameters['loaded_tuning_config'] = str(selected_path)
    for name, value_type in TUNING_TYPES.items():
        raw_value = LaunchConfiguration(name).perform(context).strip()
        if not raw_value:
            continue
        try:
            parameters[name] = value_type(raw_value)
        except ValueError as exc:
            raise ValueError(f'Invalid launch override {name}: {raw_value}') from exc
    return parameters, selected_path, saved_path, source
