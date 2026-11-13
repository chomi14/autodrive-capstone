"""Launch-time YAML defaults, overridden by existing launch arguments."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def config_argument():
    path = Path(get_package_share_directory('vehicle_bringup_pkg')) / 'config/vehicle.yaml'
    return DeclareLaunchArgument('vehicle_config', default_value=str(path))


class VehicleDefault(Substitution):
    """Read shared defaults plus an optional profile without touching hardware."""

    def __init__(self, key, fallback):
        super().__init__()
        self.key = key
        self.fallback = fallback

    def perform(self, context):
        shared = Path(get_package_share_directory('vehicle_bringup_pkg')) / 'config/vehicle.yaml'
        selected = Path(LaunchConfiguration('vehicle_config').perform(context)).expanduser()
        value = self.fallback
        for path in dict.fromkeys([shared, selected]):
            # Missing explicitly selected files / invalid YAML fail visibly.
            with path.open(encoding='utf-8') as stream:
                data = yaml.safe_load(stream)
            if data is None:
                data = {}
            if not isinstance(data, dict):
                raise ValueError(f'{path}: expected a YAML mapping')
            for part in self.key.split('.'):
                if not isinstance(data, dict) or part not in data:
                    data = None
                    break
                data = data[part]
            if data is not None:
                if not isinstance(data, (str, int, float, bool)):
                    raise ValueError(f'{path}: {self.key} must be scalar')
                value = data
        return str(value).lower() if isinstance(value, bool) else str(value)
