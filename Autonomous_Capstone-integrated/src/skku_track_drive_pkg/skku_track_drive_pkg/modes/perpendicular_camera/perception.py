"""BEV mask geometry, independent of ROS and of the inference backend.

Reimplements the reference's left boundary / rear Hough center / end stripe
ideas. It intentionally rejects ambiguous/missing geometry rather than using
old values. All Line x/y outputs are normalized; yaw uses BEV pixel axes.
"""
import math

import cv2
import numpy as np

from .observations import Line, Observation, axis_error


def orient_image(frame, rotate_deg, flip_horizontal):
    if rotate_deg:
        rotations = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                     270: cv2.ROTATE_90_COUNTERCLOCKWISE}
        frame = cv2.rotate(frame, rotations[rotate_deg])
    return cv2.flip(frame, 1) if flip_horizontal else frame


def _fit(points, vertical):
    if len(points) < 2:
        return None
    vx, vy, cx, cy = map(float, cv2.fitLine(np.float32(points), cv2.DIST_L2, 0, .01, .01).flatten())
    # Eliminate fitLine's arbitrary vector sign before averaging line pairs.
    if (vertical and vy < 0) or (not vertical and vx < 0):
        vx, vy = -vx, -vy
    if vertical and abs(vy) < .25:
        return None
    if not vertical and abs(vx) < .25:
        return None
    return vx, vy, cx, cy


def _bottom_line(fit, w, h):
    if fit is None:
        return Line()
    vx, vy, cx, cy = fit
    x = cx + vx / vy * (h - 1 - cy)
    # A projected center outside the image is not trustworthy for steering.
    if not 0 <= x < w:
        return Line()
    return Line(True, x / w, 1.0, math.atan2(vy, vx))


def _largest(mask, min_ratio):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return np.zeros_like(mask)
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < mask.size * min_ratio:
        return np.zeros_like(mask)
    result = np.zeros_like(mask)
    cv2.drawContours(result, [c], -1, 255, -1)
    return result


def front_boundary(mask, cfg):
    h, w = mask.shape
    # Left edge of each occupied row, as in the reference's left-wall tracker.
    rows = []
    for y in range(h):
        xs = np.flatnonzero(mask[y])
        if xs.size:
            rows.append((xs[0], y))
    if len(rows) < h * cfg['min_longitudinal_span_ratio']:
        return Line()
    return _bottom_line(_fit(rows, True), w, h)


def rear_center(frame_bev, roi, cfg):
    """Find painted side edges inside segmentation ROI; infer a missing side.

    A one-line reconstruction uses a CONFIGURED BEV pixel-width ratio, not a
    measured world width. Commission that ratio or reject one-line results.
    """
    h, w = roi.shape
    if not np.any(roi):
        return Line()
    gray = cv2.cvtColor(frame_bev, cv2.COLOR_BGR2GRAY)
    valid = cv2.erode(np.uint8(gray > 1) * 255, np.ones((5, 5), np.uint8), iterations=2)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
    edges = cv2.bitwise_and(edges, cv2.bitwise_and(valid, roi))
    scale = min(w, h) / 720
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, max(12, round(50 * scale)),
                            minLineLength=max(10, round(30 * scale)),
                            maxLineGap=max(5, round(20 * scale)))
    if lines is None:
        return Line()
    segments = []
    for x1, y1, x2, y2 in lines.reshape(-1, 4):
        if abs(y2-y1) < h*cfg['min_longitudinal_span_ratio'] or abs(x2-x1) > abs(y2-y1)*2:
            continue
        segments.append((float(x1), float(y1), float(x2), float(y2)))
    if not segments:
        return Line()
    m = cv2.moments(roi)
    cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
    slope = float(np.median([(b[2]-b[0])/(b[3]-b[1]) for b in segments]))
    groups = [[], []]
    for x1, y1, x2, y2 in segments:
        right = (x1+x2)/2 >= cx + slope*((y1+y2)/2-cy)
        groups[int(right)].extend(((x1, y1), (x2, y2)))
    fits = [_fit(group, True) for group in groups]
    left, right = fits
    if left is not None and right is not None:
        if abs(axis_error(math.atan2(left[1], left[0]), math.atan2(right[1], right[0]))) > cfg['pair_max_angle_rad']:
            return Line()
        # Compare at the SAME row, rather than at fitLine's different centroids.
        lx = left[2] + left[0]/left[1]*(cy-left[3])
        rx = right[2] + right[0]/right[1]*(cy-right[3])
        if rx-lx >= w*cfg['pair_min_separation_ratio']:
            center = tuple((a+b)/2 for a, b in zip(left, right))
            return _bottom_line(center, w, h)
    selected = right if right is not None else left
    if selected is None:
        return Line()
    vx, vy, x, y = selected
    shift = -.5 if right is not None else .5
    return _bottom_line((vx, vy, x + shift*cfg['parking_width_ratio']*w, y), w, h)


def stripe(mask, gray=None):
    h, w = mask.shape
    if gray is not None and np.any(mask):
        # The reference fits the brightest painted stripe inside the front mask.
        threshold, _ = cv2.threshold(gray[mask > 0], 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        white = np.uint8((gray > max(50, threshold)) & (mask > 0)) * 255
        kernel = np.ones((3, 3), np.uint8)
        white = cv2.dilate(cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel), kernel)
        white = _largest(cv2.bitwise_and(white, mask), 0)
        if cv2.countNonZero(white) >= 4:
            mask = white
    ys, xs = np.nonzero(mask)
    if len(xs) < 4:
        return Line()
    fit = _fit(np.column_stack((xs, ys)), False)
    if fit is None:
        return Line()
    vx, vy, _, _ = fit
    # Reject longitudinal blobs accidentally labeled as a horizontal stripe.
    if abs(vy) > abs(vx):
        return Line()
    bottom = int(np.argmax(ys))
    return Line(True, float(xs[bottom]/w), float(ys[bottom]/h), math.atan2(vy, vx))


def front_limit_geometry(bev, exit_mask, cfg):
    """Find transverse white paint in the front bumper's BEV corridor."""
    h, w = bev.shape[:2]
    left, right = (round(cfg[key] * (w - 1)) for key in ('limit_x_min', 'limit_x_max'))
    top, bottom = (round(cfg[key] * (h - 1)) for key in ('limit_y_min', 'limit_y_max'))
    roi = np.s_[top:bottom+1, left:right+1]
    minimum_white = (cfg['limit_white_min'],) * 3
    white = cv2.inRange(bev[roi], minimum_white, (255, 255, 255))
    edges = cv2.Canny(white, 50, 150)
    # The model labels a separate exit stripe. Its painted edges are not
    # candidates for the parallel-parking bay boundary.
    exit_region = cv2.dilate(exit_mask[roi], np.ones((5, 5), np.uint8))
    edges[exit_region > 0] = 0
    min_span = max(2, round(w * cfg['limit_min_span']))
    segments = cv2.HoughLinesP(edges, 1, math.pi / 180, max(8, min_span // 3),
                              minLineLength=min_span, maxLineGap=max(2, round(w * .02)))
    nearest = Line()
    endpoints = None
    if segments is not None:
        for raw_segment in segments.reshape(-1, 4):
            x1, y1, x2, y2 = map(int, raw_segment)
            if x1 > x2:
                x1, y1, x2, y2 = x2, y2, x1, y1
            dx, dy = x2-x1, y2-y1
            if dx < min_span or abs(dy) > dx:
                continue
            x, y = (x2, y2) if y2 >= y1 else (x1, y1)
            depth = (y+top) / (h-1)
            if not nearest.found or depth > nearest.y:
                nearest = Line(True, (x+left)/(w-1), depth, math.atan2(dy, dx))
                endpoints = ((x1+left, y1+top), (x2+left, y2+top))
    return nearest, endpoints


def bev_matrix(cfg, w, h):
    scale = np.float32([w-1, h-1])
    return cv2.getPerspectiveTransform(np.float32(cfg['persp_src']).reshape(4, 2)*scale,
                                       np.float32(cfg['persp_dst']).reshape(4, 2)*scale)


def mask_overlay(frame, polygons):
    overlay = frame.copy()
    for i, items in enumerate(polygons.values()):
        color = ((60+i*65) % 255, (220-i*45) % 255, (80+i*80) % 255)
        for poly in items:
            cv2.polylines(overlay, [np.int32(poly)], True, color, 2)
    return overlay


def extract(frame, polygons, cfg, role, stamp_ns, frame_id=''):
    """polygons: canonical class -> list of polygons in this oriented image."""
    h, w = frame.shape[:2]
    matrix = bev_matrix(cfg, w, h)
    bev = cv2.warpPerspective(frame, matrix, (w, h))
    masks = {}
    for name in cfg['classes']:
        mask = np.zeros((h, w), np.uint8)
        for polygon in polygons.get(name, []):
            points = np.asarray(polygon, dtype=np.float32)
            if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.isfinite(points).all():
                continue
            points = np.clip(points, [0, 0], [w-1, h-1]).astype(np.int32)
            cv2.fillPoly(mask, [points], 255)
        mask = cv2.warpPerspective(mask, matrix, (w, h), flags=cv2.INTER_NEAREST)
        kernel = np.ones((cfg['morph_kernel'], cfg['morph_kernel']), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        masks[name] = _largest(mask, cfg['min_area_ratio'])
    space_found = bool(np.any(masks['parking_space']))
    limit_segment = None
    if role == 'front':
        # Joining the space to the lot follows the reference's left-wall ROI.
        lot = cv2.bitwise_or(masks['parking_lot'], masks['parking_space'])
        limit, limit_segment = front_limit_geometry(bev, masks['out_line'], cfg)
        obs = Observation(stamp_ns, role, frame_id, space_found=space_found,
                          line=front_boundary(lot, cfg),
                          outline=stripe(masks['out_line'], cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)),
                          front_limit=limit)
    else:
        roi = masks['parking_space']
        if cfg.get('roi_band_ratio', 0):
            size = max(3, int(min(w, h) * cfg['roi_band_ratio']) | 1)
            roi = cv2.morphologyEx(roi, cv2.MORPH_GRADIENT, np.ones((size, size), np.uint8))
        obs = Observation(stamp_ns, role, frame_id, space_found=space_found,
                          space=rear_center(bev, roi, cfg), end=stripe(masks['end_line']))
    segmentation = mask_overlay(frame, polygons)
    overlay = bev.copy()
    for i, mask in enumerate(masks.values()):
        color = np.array(((80+i*60) % 255, 180, (30+i*80) % 255), np.uint8)
        overlay[mask > 0] = (overlay[mask > 0]*.65 + color*.35).astype(np.uint8)
    for geometry in (obs.line, obs.space):
        if geometry.found:
            bottom = geometry.x*w
            top = bottom - math.cos(geometry.yaw)/math.sin(geometry.yaw)*(h-1)
            cv2.line(overlay, (int(top), 0), (int(bottom), h-1), (255, 255, 255), 2)
    for geometry in (obs.end, obs.outline):
        if geometry.found:
            y = int(geometry.y*h)
            cv2.line(overlay, (0, y), (w-1, y), (0, 0, 255), 2)
    if limit_segment is not None:
        cv2.line(overlay, *limit_segment, (140, 255, 65), 3)
    return obs, segmentation, overlay
