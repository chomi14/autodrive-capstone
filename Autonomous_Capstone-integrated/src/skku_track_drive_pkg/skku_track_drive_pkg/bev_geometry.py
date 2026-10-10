"""Camera-space BEV source points, in TL/TR/BR/BL order (640x480)."""

import cv2
import numpy as np


BEV_CORNERS = ('tl', 'tr', 'br', 'bl')
BEV_DEFAULT_POINTS = ((238, 316), (402, 313), (501, 476), (155, 476))
BEV_SOURCE_DEFAULTS = {
    f'bev_src_{corner}_{axis}': point[index]
    for corner, point in zip(BEV_CORNERS, BEV_DEFAULT_POINTS)
    for index, axis in enumerate(('x', 'y'))
}


def source_points(values):
    return [
        [values[f'bev_src_{corner}_x'], values[f'bev_src_{corner}_y']]
        for corner in BEV_CORNERS
    ]


def validate_source_points(points, top_shift):
    effective = np.array(points, dtype=np.float32)
    effective[:2, 1] += top_shift
    if (np.any(effective[:, 0] < 0) or np.any(effective[:, 0] > 639)
            or np.any(effective[:, 1] < 0) or np.any(effective[:, 1] > 479)):
        return 'BEV source points including BEV Top Shift must be inside 640x480'
    if effective[0, 0] >= effective[1, 0] or effective[3, 0] >= effective[2, 0]:
        return 'BEV left points must stay left of their right points'
    if np.max(effective[:2, 1]) + 10 > np.min(effective[2:, 1]):
        return 'BEV bottom points must be at least 10px below the top points'
    if not cv2.isContourConvex(effective) or cv2.contourArea(effective) < 100:
        return 'BEV source must be a convex quadrilateral with area >= 100px'
    return ''
