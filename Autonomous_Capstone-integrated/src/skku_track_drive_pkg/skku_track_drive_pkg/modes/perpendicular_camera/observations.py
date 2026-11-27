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
class ParallelBay:
    number: int
    entry_y: float
    occupied: bool = False
    target_mask: bool = False
    between_vehicles: bool = False

    def __post_init__(self):
        if type(self.number) is not int or not 0 <= self.number <= 4 or not math.isfinite(self.entry_y):
            raise ValueError('Invalid parallel bay')
        if any(type(value) is not bool for value in (self.occupied, self.target_mask, self.between_vehicles)):
            raise ValueError('Invalid parallel bay flags')


@dataclass(frozen=True)
class ParallelView:
    roadside: Line = field(default_factory=Line)
    depth: float | None = None
    yaw_error: float | None = None
    bays: tuple[ParallelBay, ...] = ()
    route_target: int = 0
    far_gap: float | None = None
    far_yaw_error: float | None = None

    def __post_init__(self):
        if not isinstance(self.roadside, Line) or any(not isinstance(bay, ParallelBay) for bay in self.bays):
            raise ValueError('Invalid parallel geometry')
        if self.route_target not in (-1, 0, 2, 3):
            raise ValueError('Invalid parallel target')
        if any(value is not None and not math.isfinite(value)
               for value in (self.depth, self.yaw_error, self.far_gap, self.far_yaw_error)):
            raise ValueError('Invalid parallel pose')
        if (self.far_gap is None) != (self.far_yaw_error is None):
            raise ValueError('Far line distance and heading must be reported together')


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
    tuning_revision: int = 0
    front_limit: Line = field(default_factory=Line)
    model: str = 'perpendicular'
    parallel: ParallelView | None = None

    def __post_init__(self):
        if self.camera not in ('front', 'rear') or type(self.stamp_ns) is not int or self.stamp_ns <= 0:
            raise ValueError('Observation needs front/rear and a positive source stamp')
        if type(self.valid) is not bool or type(self.space_found) is not bool:
            raise ValueError('Observation flags must be booleans')
        if type(self.tuning_revision) is not int or self.tuning_revision < 0:
            raise ValueError('Invalid camera tuning revision')
        if not all(isinstance(getattr(self, k), Line) for k in ('line', 'space', 'end', 'outline', 'front_limit')):
            raise ValueError('Invalid observation geometry')
        if self.model not in ('perpendicular', 'parallel') or (
                self.parallel is not None and (self.model != 'parallel' or not isinstance(self.parallel, ParallelView))):
            raise ValueError('Invalid parking observation model')
        if self.model == 'parallel' and self.valid and self.parallel is None:
            raise ValueError('Valid parallel frame needs a parallel observation')

    def to_json(self):
        return json.dumps(asdict(self), allow_nan=False)

    @classmethod
    def from_json(cls, payload):
        values = json.loads(payload)
        for key in ('line', 'space', 'end', 'outline'):
            values[key] = Line(**values[key])
        if 'front_limit' in values:
            values['front_limit'] = Line(**values['front_limit'])
        if values.get('parallel') is not None:
            parallel = values['parallel']
            parallel['roadside'] = Line(**parallel['roadside'])
            parallel['bays'] = tuple(ParallelBay(**item) for item in parallel['bays'])
            values['parallel'] = ParallelView(**parallel)
        return cls(**values)


def axis_error(yaw, target=math.pi / 2):
    """Unoriented lines are pi-periodic; vertical up and down are identical."""
    return (yaw - target + math.pi / 2) % math.pi - math.pi / 2
