"""Portable deployment paths must resolve before a launch starts any hardware."""
from pathlib import Path
import sys
from types import ModuleType

import pytest

from skku_track_drive_pkg.modes.perpendicular_camera.model import model_path


def test_installed_model_uri_uses_package_share_not_current_directory(monkeypatch, tmp_path):
    share = tmp_path/'installed/share/skku_track_drive_pkg'
    weights = share/'models/parking/parking_front.pt'
    weights.parent.mkdir(parents=True)
    weights.write_bytes(b'path resolution fixture; not loaded')
    package = ModuleType('ament_index_python.packages')
    package.get_package_share_directory = lambda name: str(share)
    package.PackageNotFoundError = LookupError
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', package)
    monkeypatch.chdir(tmp_path)
    assert Path(model_path('package://skku_track_drive_pkg/models/parking/parking_front.pt')) == weights


@pytest.mark.parametrize('suffix', ['../best_v3.pt', 'models/parking/../../../best.pt', '/tmp/parking.pt'])
def test_package_uri_cannot_escape_parking_weights(suffix):
    with pytest.raises(ValueError, match='models/parking'):
        model_path('package://skku_track_drive_pkg/'+suffix)
