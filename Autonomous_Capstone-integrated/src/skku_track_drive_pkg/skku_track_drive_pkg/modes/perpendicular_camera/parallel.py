"""Parallel parking geometry from the seven semantic segmentation classes."""
import math

import cv2
import numpy as np

from .observations import Observation, ParallelBay, ParallelView, axis_error
from .perception import _bottom_line, _fit, bev_matrix, mask_overlay


PARALLEL_CLASSES = (
    'parallel_roadside_line', 'parallel_far_line', 'parallel_divider',
    'parallel_row_start', 'parallel_row_end', 'parallel_parking_space', 'vehicle',
)
ROADSIDE, FAR, DIVIDER, START, END, TARGET, VEHICLE = PARALLEL_CLASSES


def _clip(poly, w, h):
    """Clip a polygon to the BEV canvas without moving offscreen vertices to its edge."""
    for axis, bound, sign in ((0, 0, 1), (1, 0, 1), (0, w-1, -1), (1, h-1, -1)):
        clipped = []
        if len(poly) < 3:
            return np.empty((0, 2), np.float32)
        previous = poly[-1]
        prev_inside = sign * (previous[axis] - bound) >= 0
        for point in poly:
            inside = sign * (point[axis] - bound) >= 0
            if inside != prev_inside:
                clipped.append(previous + (point-previous) * ((bound-previous[axis]) /
                                                              (point[axis]-previous[axis])))
            if inside:
                clipped.append(point)
            previous, prev_inside = point, inside
        poly = clipped
    return np.float32(poly).reshape(-1, 2)


def _axis(polygons, h, min_span):
    fits = []
    for points in polygons:
        fit = _fit(points, True) or _fit(points, False)
        if fit is not None:
            span = np.ptp(points @ np.array(fit[:2]))
            if span >= h*min_span:
                vx, vy, x, y = fit
                fits.append((span, (-vx, -vy, x, y) if vy < 0 else fit))
    return max(fits, key=lambda item: item[0])[1] if fits else None


def _ground_point(points):
    """Use the lower silhouette near the tires for a ground-plane position."""
    ymax = np.max(points[:, 1])
    bottom = points[points[:, 1] >= ymax - max(2, .08*np.ptp(points[:, 1]))]
    return np.float32([np.median(bottom[:, 0]), ymax])


def _marks(projected, origin, axis, normal, width):
    marks = []
    for kind in (DIVIDER, START, END):
        for poly in projected[kind]:
            xy = np.array([(poly-origin) @ axis, (poly-origin) @ normal]).T
            fit = _fit(xy, True)
            if fit is None:
                continue
            # In station/lateral coordinates a short transverse line extends
            # across lateral z and has little station change.
            ds, dz, s, z = fit
            if abs(dz) < .80 or np.ptp(xy[:, 1]) < .18*width:
                continue
            low, high = np.min(xy[:, 1]), np.max(xy[:, 1])
            if low > .45*width or high < .18*width or high > 1.45*width:
                continue
            marks.append((float(s - (ds/dz)*z), kind))
    return sorted(marks)


def _bay_number(station, marks, slot_length, order):
    evidence = set()
    for reference, kind in marks:
        if kind not in (START, END):
            continue
        inward = order if kind == START else -order
        from_edge = (station-reference)*inward
        neighbors = sorted((mark-reference)*inward for mark, _kind in marks
                           if (mark-reference)*inward > .25*slot_length)
        previous = 0.0
        for index, neighbor in enumerate(neighbors[:4]):
            if not .55*slot_length <= neighbor-previous <= 1.55*slot_length:
                break
            if previous < from_edge < neighbor:
                evidence.add(index+1 if kind == START else 4-index)
                break
            previous = neighbor
    return evidence.pop() if len(evidence) == 1 else 0


def _bays(projected, road, axis, normal, width, ref, vehicles, role, h, slot_length):
    origin = np.asarray(road[2:])
    marks = _marks(projected, origin, axis, normal, width)
    order = -1 if role == 'front' else 1  # 1→4 points away in front, toward in rear
    seen = [(float((point-origin) @ axis), float((point-origin) @ normal)) for point in vehicles]
    cars = [station for station, lateral in seen if -.03*width <= lateral <= 1.08*width]
    votes = set()
    for station in cars:
        number = _bay_number(station, marks, slot_length, order)
        if number:
            votes.add(2 if number % 2 else 3)
    vote = votes.pop() if len(votes) == 1 else (-1 if votes else 0)
    reference_station = (ref-origin) @ axis
    target_intervals = []
    for poly in projected[TARGET]:
        station, lateral = (poly-origin) @ axis, (poly-origin) @ normal
        low, high = float(np.min(station)), float(np.max(station))
        if (.40*slot_length <= high-low <= 2.80*slot_length
                and .30*width <= np.ptp(lateral) <= 1.45*width
                and np.min(lateral) >= -.25*width and np.max(lateral) <= 1.30*width
                and .20*width <= np.median(lateral) <= .80*width):
            target_intervals.append((low, high))
    def matches_target(low, high):
        return any(min(b, high) - max(a, low) >= .55*max(b-a, high-low)
                   for a, b in target_intervals)

    intervals = [(a[0], b[0]) for a, b in zip(marks, marks[1:])
                 if .55*slot_length <= b[0]-a[0] <= 1.45*slot_length or matches_target(a[0], b[0])]
    # If one transverse mark is lost, the target mask may supply the other.
    # Require a real mark at the side used to stage the vehicle for reverse.
    for low, high in target_intervals:
        edge = low if role == 'front' else high
        nearby = [mark for mark, _kind in marks if abs(mark-edge) <= .20*slot_length]
        if nearby and not any(abs((a+b-low-high)/2) <= .25*slot_length for a, b in intervals):
            mark = min(nearby, key=lambda value: abs(value-edge))
            intervals.append((mark, high) if role == 'front' else (low, mark))
    bays = []
    for low, high in intervals:
        center = (low+high)/2
        number = _bay_number(center, marks, slot_length, order)
        matched = matches_target(low, high)
        before = any(low-1.2*slot_length <= car < low for car in cars)
        after = any(high < car <= high+1.2*slot_length for car in cars)
        entry = low if role == 'front' else high
        bays.append(ParallelBay(number, float(1 + (entry-reference_station)/h),
                                any(low <= car <= high for car in cars), matched, before and after))
    return tuple(bays), vote


def extract_parallel(frame, polygons, cfg, p, role, stamp_ns, frame_id=''):
    h, w = frame.shape[:2]
    matrix = bev_matrix(cfg, w, h)
    projected = {name: [] for name in PARALLEL_CLASSES}
    vehicles = []
    for name in PARALLEL_CLASSES:
        for raw in polygons.get(name, []):
            poly = np.float32(raw)
            if poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3 or not np.isfinite(poly).all():
                continue
            points = cv2.perspectiveTransform(poly.reshape(-1, 1, 2), matrix).reshape(-1, 2)
            visible = _clip(points, w, h)
            if len(visible) >= 3 and cv2.contourArea(visible) >= max(4, w*h*.00002):
                projected[name].append(visible)
            if name == VEHICLE:
                point = cv2.perspectiveTransform(_ground_point(poly).reshape(1, 1, 2), matrix)[0, 0]
                if 0 <= point[0] < w and 0 <= point[1] < h:
                    vehicles.append(point)
    road, far = (_axis(projected[name], h, cfg['min_longitudinal_span_ratio']) for name in (ROADSIDE, FAR))
    roadside = _bottom_line(road if road is not None and abs(road[1]) >= .5 else None, w, h)
    fit = road or far
    bays, route_target, yaw, depth = (), 0, None, None
    far_gap = far_yaw = None
    if fit is not None:
        axis = np.array(fit[:2])
        normal = np.array([axis[1], -axis[0]]) * (-1 if role == 'front' else 1)
        width = p.parallel_row_width_ratio*w
        yaw = axis_error(math.atan2(axis[1], axis[0]))
        reference_y = p.parallel_rear_reference_y if role == 'rear' else 1.0
        ref = np.array([w*.5, (h-1)*reference_y])
        if road is not None and far is not None:
            measured = (np.asarray(far[2:]) - np.asarray(road[2:])) @ normal
            if abs(np.dot(road[:2], far[:2])) >= .90 and .50*width <= abs(measured) <= 2.40*width:
                # Side-on frames can help identify the bay even when the
                # fitted line has no usable bottom intersection for steering.
                if measured < 0 and abs(axis[1]) < .65:
                    axis, normal, measured = -axis, -normal, -measured
                if measured > 0:
                    width = measured
                else:
                    far = None
            else:
                far = None
        depth = float((ref-np.asarray(fit[2:])) @ normal / width + (0 if road is not None else 1))
        if far is not None:
            far_axis = np.array(far[:2])
            if far_axis @ axis < 0:
                far_axis = -far_axis
            far_normal = np.array([far_axis[1], -far_axis[0]]) * (-1 if role == 'front' else 1)
            far_gap = float((np.asarray(far[2:])-ref) @ far_normal / width)
            far_yaw = axis_error(math.atan2(far_axis[1], far_axis[0]))
        if road is not None:
            bays, route_target = _bays(projected, road, axis, normal, width, ref, vehicles,
                                       role, h, p.parallel_slot_length_ratio*h)
    view = ParallelView(roadside, depth, yaw, bays, route_target, far_gap, far_yaw)
    obs = Observation(stamp_ns, role, frame_id, model='parallel', parallel=view)
    bev = mask_overlay(cv2.warpPerspective(frame, matrix, (w, h)), projected)
    return obs, mask_overlay(frame, polygons), bev
