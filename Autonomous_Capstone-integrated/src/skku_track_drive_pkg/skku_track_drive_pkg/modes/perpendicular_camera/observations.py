"""Immutable results from ONE source image; no ROS dependencies."""
from dataclasses import asdict, dataclass, field
import json
import math


@dataclass(frozen=True)
class Line:
    found: bool = False
    x: float = 0.0  # bottom intersection / image width (longitudinal lines)
    y: float = 0.0  # bottom anchor / image height (end/exit stripes)
    yaw: float = 0.0  # radians in BEV pixel axes: +x right, +y down

    def __post_init__(self):
        if type(self.found) is not bool or not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError('Line must have a boolean found and finite coordinates')
        if not (0 <= self.x <= 1 and 0 <= self.y <= 1 and -math.pi <= self.yaw <= math.pi):
            raise ValueError('Line needs normalized x/y and yaw in [-pi, pi]')


@dataclass(frozen=True)
class Observation:
    stamp_ns: int
    camera: str
    frame_id: str = ''
    valid: bool = True  # successful inference, including genuine no-detection
    space_found: bool = False
    line: Line = field(default_factory=Line)
    space: Line = field(default_factory=Line)
    end: Line = field(default_factory=Line)
    outline: Line = field(default_factory=Line)
    error: str = ''

    def __post_init__(self):
        if self.camera not in ('front', 'rear') or type(self.stamp_ns) is not int or self.stamp_ns <= 0:
            raise ValueError('Observation needs front/rear and a positive source stamp')
        if type(self.valid) is not bool or type(self.space_found) is not bool:
            raise ValueError('Observation flags must be booleans')
        if not all(isinstance(getattr(self, k), Line) for k in ('line', 'space', 'end', 'outline')):
            raise ValueError('Invalid observation geometry')

    def to_json(self):
        return json.dumps(asdict(self), allow_nan=False)

    @classmethod
    def from_json(cls, payload):
        values = json.loads(payload)
        for key in ('line', 'space', 'end', 'outline'):
            values[key] = Line(**values[key])
        return cls(**values)


def axis_error(yaw, target=math.pi / 2):
    """Unoriented lines are pi-periodic; vertical up and down are identical."""
    return (yaw - target + math.pi / 2) % math.pi - math.pi / 2
