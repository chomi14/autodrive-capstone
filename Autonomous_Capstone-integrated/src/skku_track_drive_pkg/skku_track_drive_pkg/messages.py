from dataclasses import dataclass, field
from typing import List

@dataclass
class Point2D:
    x: float = 0.0
    y: float = 0.0

@dataclass
class Pose2D:
    position: Point2D = field(default_factory=Point2D)
    theta: float = 0.0

@dataclass
class Vector2:
    x: float = 0.0
    y: float = 0.0

@dataclass
class BoundingBox2D:
    center: Pose2D = field(default_factory=Pose2D)
    size: Vector2 = field(default_factory=Vector2)

@dataclass
class Mask:
    data: List[Point2D] = field(default_factory=list)
    height: int = 0
    width: int = 0

@dataclass
class Detection:
    class_id: int = -1
    class_name: str = ""
    score: float = 0.0
    bbox: BoundingBox2D = field(default_factory=BoundingBox2D)
    mask: Mask = field(default_factory=Mask)

@dataclass
class DetectionArray:
    detections: List[Detection] = field(default_factory=list)

@dataclass
class TargetPoint:
    target_x: int = 0
    target_y: int = 0

@dataclass
class LaneInfo:
    slope: float = 0.0
    target_points: List[TargetPoint] = field(default_factory=list)

@dataclass
class PathPlanningResult:
    x_points: List[float] = field(default_factory=list)
    y_points: List[float] = field(default_factory=list)

@dataclass
class MotionCommand:
    steering: int = 0
    left_speed: int = 0
    right_speed: int = 0
