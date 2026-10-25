"""Compatibility imports; implementation: skku_track_drive_pkg.modes.parking_common.parking_controller_node."""
from .modes.parking_common.parking_controller_node import *  # noqa: F401,F403
from .modes.parking_common import parking_controller_node as _implementation

def __getattr__(name):
    return getattr(_implementation, name)
