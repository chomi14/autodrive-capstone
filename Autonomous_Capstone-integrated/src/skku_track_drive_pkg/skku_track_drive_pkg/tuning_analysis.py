"""Read-only control evidence; all distances here are BEV pixels, not metres."""
import numpy as np


def path_change(before, after):
    if len(before) < 2 or len(after) < 2:
        return {'path_delta_mean_px': None, 'path_delta_max_px': None}
    a, b = sorted(before, key=lambda p: p[1]), sorted(after, key=lambda p: p[1])
    lo, hi = max(a[0][1], b[0][1]), min(a[-1][1], b[-1][1])
    if hi <= lo:
        return {'path_delta_mean_px': None, 'path_delta_max_px': None}
    ys = np.linspace(lo, hi, 40)
    delta = np.abs(np.interp(ys, [p[1] for p in a], [p[0] for p in a]) -
                   np.interp(ys, [p[1] for p in b], [p[0] for p in b]))
    return {'path_delta_mean_px': float(np.mean(delta)), 'path_delta_max_px': float(np.max(delta))}


def frame_analysis(controller, path, command, previous=None):
    mission = getattr(getattr(controller, 'mission', None), 'status', {})
    used_lane = getattr(controller, '_processed_lane', 2)
    points = [[float(x), float(y)] for x, y in zip(path.x_points, path.y_points)]
    record = {
        'analysis_schema': 1, 'controller': controller.get_name(),
        'path_lane': used_lane, 'target_lane': mission.get('target_lane', used_lane),
        'path_valid': len(points) >= 2, 'lane_currently_visible': controller.lane.missed_frames == 0,
        'path_points_bev_px': points, 'cte_px': float(controller.motion.last_cte),
        'heading_error_deg': float(controller.motion.last_heading_deg),
        'error_reference_valid': controller.motion.last_reference_point is not None,
        'steering': int(command.steering), 'left_pwm': int(command.left_speed), 'right_pwm': int(command.right_speed),
        'mission_state': mission.get('state', 'TRACK'), 'reason': mission.get('reason', 'TRACK_ONLY'),
        'mission': mission, 'lane_switch_requested': mission.get('lane_switch_requested', False),
        'lane_switch_applied': previous is not None and previous['path_lane'] != used_lane,
    }
    if previous is not None:
        record['change_from_previous'] = {
            'previous_frame_stamp_ns': previous.get('frame_stamp_ns'),
            'previous_path_lane': previous['path_lane'], 'previous_steering': previous['steering'],
            'steering_delta_steps': record['steering'] - previous['steering'],
            'cte_delta_px': record['cte_px'] - previous['cte_px'],
            'heading_delta_deg': (record['heading_error_deg'] - previous['heading_error_deg'] + 180) % 360 - 180,
            **path_change(previous['path_points_bev_px'], points),
        }
    return record
