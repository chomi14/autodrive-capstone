"""Left-side parallel-parking path in a measured, static 2D scene.

The pose is the rear axle center. World x follows the parking slot forward;
world y is left of x; positive steering turns the front wheels left.
"""

from dataclasses import dataclass
import heapq
import itertools
import math


def wrap(angle):
    return (angle + math.pi) % math.tau - math.pi


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    def point(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y


@dataclass(frozen=True)
class Box:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def contains(self, point):
        x, y = point
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax


@dataclass(frozen=True)
class Vehicle:
    wheelbase: float
    front_overhang: float
    rear_overhang: float
    width: float
    steer_left_deg: float
    steer_right_deg: float

    def corners(self, pose, padding=0.0):
        front = self.wheelbase + self.front_overhang + padding
        back = -self.rear_overhang - padding
        half = self.width / 2 + padding
        return tuple(pose.point(x, y) for x, y in ((front, half), (front, -half),
                                                    (back, -half), (back, half)))


@dataclass(frozen=True)
class Scene:
    slot: Box
    bounds: Box
    obstacles: tuple
    margin: float
    segment_m: float
    heading_tolerance_deg: float


@dataclass(frozen=True)
class Move:
    direction: int
    steering_deg: float
    poses: tuple


def advance(pose, distance, curvature):
    if abs(curvature) < 1e-10:
        x, y = pose.point(distance, 0)
        return Pose(x, y, pose.yaw)
    next_yaw = pose.yaw + distance * curvature
    return Pose(pose.x + (math.sin(next_yaw) - math.sin(pose.yaw)) / curvature,
                pose.y + (math.cos(pose.yaw) - math.cos(next_yaw)) / curvature,
                wrap(next_yaw))


def overlaps(corners, box, yaw):
    """Separating axis check for a rotated vehicle and axis-aligned obstacle."""
    cx, cy = (box.xmin + box.xmax) / 2, (box.ymin + box.ymax) / 2
    hx, hy = (box.xmax - box.xmin) / 2, (box.ymax - box.ymin) / 2
    c, s = math.cos(yaw), math.sin(yaw)
    for ax, ay in ((1, 0), (0, 1), (c, s), (-s, c)):
        projected = [x * ax + y * ay for x, y in corners]
        center = cx * ax + cy * ay
        radius = abs(ax) * hx + abs(ay) * hy
        if max(projected) < center - radius or min(projected) > center + radius:
            return False
    return True


def is_clear(pose, vehicle, scene, padding):
    corners = vehicle.corners(pose, padding)
    return (all(scene.bounds.contains(point) for point in corners)
            and not any(overlaps(corners, obstacle, pose.yaw) for obstacle in scene.obstacles))


def sample_move(pose, direction, steering, vehicle, scene):
    curvature = math.tan(math.radians(steering)) / vehicle.wheelbase
    # Bound corner motion between samples, then pad the sampled footprint.
    radius = math.hypot(max(vehicle.wheelbase + vehicle.front_overhang, vehicle.rear_overhang)
                        + scene.margin, vehicle.width / 2 + scene.margin)
    corner_rate = 1 + radius * abs(curvature)
    steps = max(4, math.ceil(scene.segment_m * corner_rate / (scene.margin / 2)))
    if steps > 128:
        raise ValueError("segment_m is too large relative to the margin and vehicle")
    padding = scene.margin + scene.segment_m * corner_rate / (2 * steps)
    samples = tuple(advance(pose, direction * scene.segment_m * i / steps, curvature)
                    for i in range(steps + 1))
    return samples if all(is_clear(p, vehicle, scene, padding) for p in samples) else ()


def parked(pose, vehicle, scene):
    return (abs(wrap(pose.yaw)) <= math.radians(scene.heading_tolerance_deg)
            and all(scene.slot.contains(point) for point in vehicle.corners(pose, scene.margin)))


def plan(start, vehicle, scene, max_states=6000):
    """Return a checked preview path, [] if already parked, or None on failure."""
    if not is_clear(start, vehicle, scene, scene.margin):
        return None
    if parked(start, vehicle, scene):
        return []
    # The entry version of this mode only applies with the slot behind and left.
    slot_x = (scene.slot.xmin + scene.slot.xmax) / 2 - start.x
    slot_y = (scene.slot.ymin + scene.slot.ymax) / 2 - start.y
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    if slot_x * c + slot_y * s >= 0 or -slot_x * s + slot_y * c <= 0:
        return None

    left, right = vehicle.steer_left_deg, -vehicle.steer_right_deg
    first = sample_move(start, -1, left, vehicle, scene)
    if not first:
        return None

    def rear_inside(pose):
        return scene.slot.contains(pose.point(-vehicle.rear_overhang, 0))

    goal_x = (scene.slot.xmin + scene.slot.xmax - vehicle.wheelbase
              - vehicle.front_overhang + vehicle.rear_overhang) / 2
    goal_y = (scene.slot.ymin + scene.slot.ymax) / 2
    grid = scene.segment_m / 2

    def score(pose):
        return math.hypot(pose.x - goal_x, pose.y - goal_y) + 0.5 * vehicle.wheelbase * abs(pose.yaw)

    def key(pose, direction, entered):
        return (round(pose.x / grid), round(pose.y / grid),
                round(wrap(pose.yaw) / math.radians(5)), direction, entered)

    # Store parent indexes so reconstructing a route uses the exact poses checked.
    nodes, best, queue = [], {}, []
    order = itertools.count()

    def push(move, entered, cost, parent):
        pose = move.poses[-1]
        identity = key(pose, move.direction, entered)
        if cost >= best.get(identity, math.inf):
            return
        best[identity] = cost
        nodes.append((move, entered, cost, parent, identity))
        heapq.heappush(queue, (cost + 1.5 * score(pose), next(order), len(nodes) - 1))

    push(Move(-1, left, first), any(map(rear_inside, first)), scene.segment_m, None)
    expanded = 0
    while queue and expanded < max_states:
        _, _, index = heapq.heappop(queue)
        move, entered, cost, parent, identity = nodes[index]
        if cost > best[identity]:
            continue
        expanded += 1
        pose = move.poses[-1]
        if entered and parked(pose, vehicle, scene):
            result = [move]
            while parent is not None:
                previous, _, _, parent, _ = nodes[parent]
                result.append(previous)
            return result[::-1]
        # Until the back enters, keep backing in. After entry, short corrections
        # may go either way. A direction change costs more than a same-gear step.
        for direction in (-1, 1) if entered else (-1,):
            for steer in (left, 0., right):
                samples = sample_move(pose, direction, steer, vehicle, scene)
                if samples:
                    switch_penalty = scene.segment_m * 1.5 if direction != move.direction else 0
                    push(Move(direction, steer, samples), entered or any(map(rear_inside, samples)),
                         cost + scene.segment_m + switch_penalty, index)
    return None
