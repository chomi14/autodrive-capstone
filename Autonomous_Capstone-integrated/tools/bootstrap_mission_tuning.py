#!/usr/bin/env python3
"""One-time shared-parameter snapshot; never overwrite a saved mission YAML."""
import argparse
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from vehicle_bringup_pkg.tuning_configuration import COMMON_TRACK_PARAMETERS, ALLOWED_TUNING_PARAMETERS, _profile, _validate_parameter_file
from skku_track_drive_pkg.track_tuner_node import TrackTunerNode


def bootstrap(track_default, mission_default, track_saved, mission_saved):
    track = _validate_parameter_file(Path(track_default))
    track_saved, mission_saved = Path(track_saved), Path(mission_saved)
    source = Path(track_default)
    if track_saved.is_file():
        track.update(_validate_parameter_file(track_saved))
        source = track_saved
    root, _filename, _types, allowed = _profile('mission')
    mission = _validate_parameter_file(Path(mission_default), root, allowed)
    existing = mission_saved.is_file()
    if existing:
        mission.update(_validate_parameter_file(mission_saved, root, allowed))
    shared = {name: value for name, value in track.items() if name in COMMON_TRACK_PARAMETERS}
    differences = {name: {'track': value, 'mission': mission.get(name)} for name, value in shared.items() if mission.get(name) != value}
    if not existing:
        TrackTunerNode.write_yaml_atomic(mission_saved, {**mission, **shared}, root=root)
    return {'created': not existing, 'track_source': str(source), 'mission_path': str(mission_saved),
            'shared': shared, 'previous_differences': differences,
            'note': 'Existing mission file preserved' if existing else 'Independent one-time snapshot created; speed and mission thresholds retained'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track-saved', default='~/.config/autodrive/track_tuning.yaml')
    parser.add_argument('--mission-saved', default='~/.config/autodrive/mission_tuning.yaml')
    options = parser.parse_args()
    config = Path(get_package_share_directory('vehicle_bringup_pkg'), 'config')
    result = bootstrap(config/'track_tuning.yaml', config/'mission_tuning.yaml',
        Path(options.track_saved).expanduser(), Path(options.mission_saved).expanduser())
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
