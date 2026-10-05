"""Scan geometry about the REAR AXLE: x forward, y left, angles CCW.

The driver's rotation_offset_deg is applied once upstream. scan_angle_sign and
lidar_yaw_deg here account for measured handedness/yaw, not a second driver
rotation. No-return rays are unknown, not proof of a parking space.
"""
import math
import numpy as np


def scan_points(scan, p):
    distances = np.asarray(scan.ranges, dtype=float)
    angles = (scan.angle_min + np.arange(len(distances)) * scan.angle_increment) * p['scan_angle_sign']
    angles += math.radians(p['lidar_yaw_deg'])
    headings = np.degrees(np.arctan2(np.sin(angles), np.cos(angles)))
    valid = np.isfinite(distances) & (distances >= max(scan.range_min, p['range_min_m']))
    valid &= distances <= min(scan.range_max, p['range_max_m'])
    valid &= (headings >= p['visible_min_deg']) & (headings <= p['visible_max_deg'])
    return np.column_stack((p['lidar_x_m'] + distances[valid] * np.cos(angles[valid]),
                            p['lidar_y_m'] + distances[valid] * np.sin(angles[valid]),
                            headings[valid]))


def sector_distance(points, center, half_width, p):
    if len(points) == 0:
        return None
    difference = (points[:, 2] - center + 180) % 360 - 180
    selected = points[np.abs(difference) <= half_width]
    if len(selected) == 0:
        return None
    return float(np.min(np.hypot(selected[:, 0] - p['lidar_x_m'], selected[:, 1] - p['lidar_y_m'])))


def collision(points, p):
    if len(points) == 0 or p['vehicle_width_m'] <= 0:
        return False
    margin = p['collision_margin_m']
    x, y = points[:, 0], points[:, 1]
    # Exclude returns on the measured chassis itself. Any outside point inside
    # the inflated boundary blocks movement in either direction.
    rear = -p['rear_overhang_m']
    front = p['wheelbase_m'] + p['front_overhang_m']
    half = p['vehicle_width_m'] / 2
    body = (x >= rear) & (x <= front) & (np.abs(y) <= half)
    boundary = (x >= rear-margin) & (x <= front+margin) & (np.abs(y) <= half+margin)
    return bool(np.any(boundary & ~body))


def side_line_angle(points, side, p):
    if len(points) < 4:
        return None
    subset = points[(points[:, 1] * side > p['vehicle_width_m']/2)
                    & (points[:, 1] * side < p['range_max_m'])]
    if len(subset) < 4 or np.ptp(subset[:, 0]) < .2:
        return None
    # Fit nearby side-boundary returns; reject a line scattered over objects.
    median = np.median(subset[:, 1])
    subset = subset[np.abs(subset[:, 1] - median) < .15]
    if len(subset) < 4 or np.ptp(subset[:, 0]) < .2:
        return None
    slope, _ = np.polyfit(subset[:, 0], subset[:, 1], 1)
    return math.degrees(math.atan(float(slope)))
