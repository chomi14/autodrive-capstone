"""Image-space evidence for the bay between the two reference vehicles."""

from dataclasses import dataclass, replace
import itertools
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class Bay:
    center: tuple
    axis: tuple
    far_end_y: float
    aim_x: float
    angle_error: float
    corners: tuple = ()
    anchored: bool = True


def _bay_from_corners(corners, shape):
    """Corners are in pairs across the far and near ends."""
    h, w = shape[:2]
    far, near = corners[:2].mean(axis=0), corners[2:].mean(axis=0)
    length = np.linalg.norm(near - far)
    if length < .10 * min(h, w):
        return None
    t = (near - far) / length
    if t[1] < 0:
        t, far = -t, near
    center = corners.mean(axis=0)
    offset = np.clip((.75*h - center[1]) / max(t[1], .15), -length/2, length/2)
    aim_x = (center[0] + t[0] * offset) / w
    angle = math.atan2(t[1], t[0]) % math.pi - math.pi/2
    return Bay(tuple(center / [w, h]), tuple(t), float(far[1]/h), float(aim_x), angle,
               tuple(tuple(point) for point in corners))


def _bands(lines, axis, along, short):
    result = []
    for line in lines:
        a, b = line
        delta = b - a
        length = np.linalg.norm(delta)
        if length < short * .13 or abs(float(delta @ along)) / length < .88:
            continue
        interval = sorted(line @ along)
        offset = float(line.mean(axis=0) @ axis)
        if all(abs(offset - item[0]) > short * .022 for item in result):
            result.append((offset, *interval))
    return result


def find_bay(shape, references, lines):
    """Find one rectangle between the reference contacts; all units are pixels."""
    if len(references) != 2 or not lines:
        return None
    h, w = shape[:2]
    short = min(h, w)
    refs = np.asarray(references, dtype=float)
    delta = refs[1] - refs[0]
    span = np.linalg.norm(delta)
    if not np.isfinite(refs).all() or span < .24 * short:
        return None
    t = delta / span
    if t[1] < 0 or (t[1] == 0 and t[0] < 0):
        t = -t
    n = np.array((t[1], -t[0]))
    a, b = sorted(refs @ t)
    nr = refs @ n
    # Sort longest first so tape's two edges do not become two boundaries.
    segments = sorted((np.asarray(line, float).reshape(2, 2) for line in lines),
                      key=lambda q: -np.linalg.norm(q[1] - q[0]))[:36]
    longitudinal = _bands(segments, n, t, short)
    transverse = _bands(segments, t, n, short)
    transverse = sorted(item for item in transverse if a + .10 * span < item[0] < b - .10 * span)
    results = []
    for (s0, n0, n1), (s1, m0, m1) in itertools.combinations(transverse, 2):
        length = s1 - s0
        if not .28 * span <= length <= .85 * span:
            continue
        for (r0, u0, u1), (r1, v0, v1) in itertools.combinations(sorted(longitudinal), 2):
            width = r1 - r0
            if not (.28 <= width / length <= 1.15 and r0 - .12 * width <= min(nr)
                    and max(nr) <= r1 + .12 * width):
                continue
            if (min(u1, v1) < s1 - .18 * length or max(u0, v0) > s0 + .18 * length
                    or max(n0, m0) > r0 + .25 * width or min(n1, m1) < r1 - .25 * width):
                continue
            corners = np.array([t*s + n*r for s in (s0, s1) for r in (r0, r1)])
            if (np.any(corners < 0) or np.any(corners >= [w, h])):
                continue
            center = corners.mean(axis=0)
            mid_s, mid_r = (s0+s1)/2, (r0+r1)/2
            score = abs(mid_s-(a+b)/2)/span + abs(mid_r-nr.mean())/width + .35*abs(width/length - .57)
            bay = _bay_from_corners(corners, shape)
            if bay is not None:
                results.append((score, center, bay))
    if not results:
        return None
    results.sort(key=lambda result: result[0])
    score, center, bay = results[0]
    if any(other_score - score < .10 and np.linalg.norm(other - center) > .065*short
           for other_score, other, _ in results[1:]):
        return None
    return bay


def image_blocked(boxes, shape, wide=False):
    """Conservative near-image veto, not a measurement of bumper distance."""
    h, w = shape[:2]
    margin = .14 if wide else .27
    for x0, y0, x1, y1 in boxes:
        bw, bh = (x1 - x0) / w, (y1 - y0) / h
        across = x0 / w < 1 - margin and x1 / w > margin
        if across and (y1 / h > .73 or bw * bh > .085):
            return True
        if y1 / h > .91 and bw > .075:
            return True
    return False


def _rear_view(image, boxes, camera):
    h, w = image.shape[:2]
    dims = np.float32((w - 1, h - 1))
    src = np.asarray(camera['persp_src'], np.float32).reshape(4, 2) * dims
    dst = np.asarray(camera['persp_dst'], np.float32).reshape(4, 2) * dims
    transform = cv2.getPerspectiveTransform(src, dst)
    white = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (0, 0, 150), (179, 72, 255))
    hull = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(hull, src.astype(np.int32), 255)
    for x0, y0, x1, y1 in boxes:
        cv2.rectangle(hull, (int(x0), int(y0)), (int(x1), int(y1)), 0, -1)
    white = cv2.warpPerspective(white & hull, transform, (w, h), flags=cv2.INTER_NEAREST)
    floor = cv2.warpPerspective(hull, transform, (w, h), flags=cv2.INTER_NEAREST)
    gray = cv2.warpPerspective(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), transform, (w, h))
    return src, transform, white, floor, gray


def _detect_bay(shape, boxes, src, transform, white):
    if len(boxes) != 2:
        return None
    contacts = [((x0+x1)/2, y1) for x0, _, x1, y1 in boxes]
    if any(cv2.pointPolygonTest(src, point, False) < 0 for point in contacts):
        return None
    refs = cv2.perspectiveTransform(np.float32([contacts]), transform)[0]
    short = min(shape[:2])
    lines = cv2.HoughLinesP(cv2.Canny(white, 50, 150),
                            1, math.pi/180, 28, minLineLength=max(20, round(.13*short)),
                            maxLineGap=max(6, round(.055*short)))
    return None if lines is None else find_bay(shape, refs, lines.tolist())


class RearBayTracker:
    """Keep the selected bay briefly while the visible far stripe supports it."""

    def __init__(self):
        self.previous = None

    def observe(self, image, boxes, camera, stamp_ns, hold_still=False):
        src, matrix, white, floor, gray = _rear_view(image, boxes, camera)
        bay = _detect_bay(image.shape, boxes, src, matrix, white)
        if bay is not None:
            self.previous = (gray, floor, bay, stamp_ns)
            return bay
        if self.previous is None:
            return None
        previous, old_floor, old_bay, full_stamp = self.previous
        limit = 8_000_000_000 if hold_still else 2_000_000_000
        if not 0 < stamp_ns - full_stamp <= limit or previous.shape != gray.shape:
            self.previous = None
            return None
        corners = np.asarray(old_bay.corners, dtype=np.float32)
        roi = np.zeros_like(old_floor)
        cv2.fillConvexPoly(roi, cv2.convexHull(corners).astype(np.int32), 255)
        roi = cv2.dilate(roi, np.ones((21, 21), np.uint8)) & old_floor
        points = cv2.goodFeaturesToTrack(previous, 80, .015, 5, mask=roi)
        if points is None or len(points) < 8:
            return None
        moved, good, _ = cv2.calcOpticalFlowPyrLK(previous, gray, points, None)
        if moved is None:
            return None
        back, check, _ = cv2.calcOpticalFlowPyrLK(gray, previous, moved, None)
        if back is None:
            return None
        valid = ((good.ravel() == 1) & (check.ravel() == 1)
                 & (np.linalg.norm((back - points).reshape(-1, 2), axis=1) < 1.5))
        if np.count_nonzero(valid) < 8:
            return None
        transform, inliers = cv2.estimateAffinePartial2D(points[valid], moved[valid], method=cv2.RANSAC,
                                                        ransacReprojThreshold=2.0)
        if transform is None or np.count_nonzero(inliers) < 8:
            return None
        if not .87 <= math.hypot(transform[0, 0], transform[1, 0]) <= 1.15:
            return None
        corners = cv2.transform(corners[None], transform)[0]
        h, w = white.shape
        if not np.isfinite(corners).all() or not 0 <= corners[2:].mean(axis=0)[1] < 1.65*h:
            return None
        samples = np.linspace(corners[0], corners[1], 18)[3:-3]
        hits = 0
        for px, py in np.rint(samples).astype(int):
            if 0 <= px < w and 0 <= py < h and np.any(white[max(0, py-5):min(h, py+6),
                                                                  max(0, px-5):min(w, px+6)]):
                hits += 1
        if hits < 7:
            return None
        bay = _bay_from_corners(corners, gray.shape)
        if bay is not None:
            bay = replace(bay, anchored=False)
            self.previous = (gray, floor, bay, full_stamp)
        return bay
