"""Compatibility imports; implementation: skku_track_drive_pkg.modes.bbox_mission.mission_controller_node."""
from .modes.bbox_mission.mission_controller_node import *  # noqa: F401,F403
from .modes.bbox_mission import mission_controller_node as _implementation

def __getattr__(name):
    return getattr(_implementation, name)
