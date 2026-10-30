"""Compatibility imports; implementation: skku_track_drive_pkg.modes.parking_common.parking_core."""
from .modes.parking_common.parking_core import *  # noqa: F401,F403
from .modes.parking_common import parking_core as _implementation

def __getattr__(name):
    return getattr(_implementation, name)
