"""Measured, mode-specific perception silence policy, separate from UI health."""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


def measured_policy(mode):
    path = Path(get_package_share_directory('vehicle_bringup_pkg'), 'config/perception_delay_policy.yaml')
    with path.open() as stream:
        data = yaml.safe_load(stream)
    if data.get('schema_version') != 2:
        raise ValueError(f'{path}: unsupported perception policy schema')
    values = data['profiles'][mode]
    result = {name: float(values[name]) for name in ('perception_stop_s',)}
    if not all(math.isfinite(v) and v > 0 for v in result.values()):
        raise ValueError(f'{path}: stop age must be finite and positive')
    return result


def perception_arguments(mode):
    return [DeclareLaunchArgument(name, default_value=str(value),
            description='Existing result-silence stop age; hold command unchanged, then X and require new W')
            for name, value in measured_policy(mode).items()]
