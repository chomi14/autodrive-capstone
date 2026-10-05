"""Numeric mode settings shared by controller validation and tuning panels.

Each specification is (default, minimum, maximum, slider step). SI units are
used for parking. Zero geometry values mean unmeasured, never guessed dimensions.
"""

MISSION = {
    'obstacle_confirm_frames': (3, 1, 30, 1),
    'obstacle_clear_frames': (5, 1, 60, 1),
    'obstacle_near_y': (300, 0, 480, 1),
    'path_margin_px': (35, 0, 200, 1),
    'alternate_min_area_px': (250, 50, 20000, 50),
    'image_timeout_s': (0.75, 0.1, 3.0, 0.05),
    'avoid_hold_s': (2.0, 0.0, 10.0, 0.05),
    'avoid_speed': (80, 0, 255, 1),
    'yellow_speed': (60, 0, 255, 1),
    'red_confirm_frames': (3, 1, 30, 1),
    'green_confirm_frames': (3, 1, 30, 1),
    'wait_for_green': (0, 0, 1, 1),
    'traffic_min_width': (20, 1, 640, 1),
    'traffic_min_bottom_y': (80, 0, 480, 1),
    'traffic_min_color_ratio': (0.02, 0.0, 1.0, 0.005),
    'hsv_s_min': (100, 0, 255, 1),
    'hsv_v_min': (95, 0, 255, 1),
    'red_h_low_max': (10, 0, 30, 1),
    'red_h_high_min': (160, 140, 179, 1),
    'green_h_min': (40, 31, 90, 1),
    'green_h_max': (90, 40, 139, 1),
    'yellow_h_min': (20, 11, 30, 1),
    'yellow_h_max': (30, 20, 39, 1),
}

PARKING_COMMON = {
    'geometry_confirmed': (0, 0, 1, 1),
    'lidar_x_m': (0.0, -2.0, 2.0, 0.01),
    'lidar_y_m': (0.0, -1.0, 1.0, 0.01),
    'lidar_yaw_deg': (0.0, -180.0, 180.0, 1.0),
    'scan_angle_sign': (1, -1, 1, 2),
    'visible_min_deg': (-135.0, -180.0, 0.0, 1.0),
    'visible_max_deg': (135.0, 0.0, 180.0, 1.0),
    'range_min_m': (0.15, 0.05, 1.0, 0.01),
    'range_max_m': (5.0, 1.0, 12.0, 0.1),
    'scan_timeout_s': (0.5, 0.1, 3.0, 0.05),
    'forward_pwm': (85, 0, 255, 1),
    'reverse_pwm': (60, 0, 255, 1),
    'steer_step': (7, 1, 7, 1),
    'steering_sign': (1, -1, 1, 2),
    'side': (-1, -1, 1, 2),
    'side_center_deg': (90.0, 30.0, 130.0, 1.0),
    'side_half_width_deg': (10.0, 1.0, 40.0, 1.0),
    'side_detect_m': (1.5, 0.2, 3.0, 0.05),
    'confirm_scans': (3, 1, 20, 1),
    'pause_s': (5.0, 0.0, 10.0, 0.1),
    'settle_s': (0.5, 0.1, 2.0, 0.05),
    'search_timeout_s': (60.0, 1.0, 120.0, 1.0),
    'maneuver_timeout_s': (30.0, 1.0, 120.0, 1.0),
    'collision_margin_m': (0.15, 0.05, 0.5, 0.01),
    'wheelbase_m': (0.0, 0.0, 2.0, 0.01),
    'front_overhang_m': (0.0, 0.0, 1.0, 0.01),
    'rear_overhang_m': (0.0, 0.0, 1.0, 0.01),
    'vehicle_width_m': (0.0, 0.0, 2.0, 0.01),
    'forward_mps': (0.0, 0.0, 1.0, 0.01),
    'reverse_mps': (0.0, 0.0, 1.0, 0.01),
    'max_wheel_angle_deg': (0.0, 0.0, 60.0, 1.0),
}

PERPENDICULAR = {
    **PARKING_COMMON,
    'maneuver_timeout_s': (60.0, 1.0, 120.0, 1.0),
    'start_detect_m': (2.5, 0.2, 3.0, 0.05),
    'case1_max_m': (1.0, 0.2, 2.0, 0.05),
    'case2_max_m': (1.5, 0.3, 2.5, 0.05),
    'case3_max_m': (2.0, 0.4, 3.0, 0.05),
    'fwd_straight_out_s': (1.5, 0.0, 10.0, 0.1),
    'fwd_turn_out_s': (12.0, 0.0, 30.0, 0.1),
    'finish_s': (30.0, 0.0, 60.0, 0.1),
    'exit_enabled': (0, 0, 1, 1),
}
# Preserve the four calibrated timing profiles from the existing ver2 node.
from lidar_perception_pkg.parking_timing import (
    TIMING_LE_1_0M, TIMING_1_0_TO_1_5M, TIMING_1_5_TO_2_0M, TIMING_2_0_TO_2_5M,
)
for index, timing in enumerate((TIMING_LE_1_0M, TIMING_1_0_TO_1_5M,
                                TIMING_1_5_TO_2_0M, TIMING_2_0_TO_2_5M), 1):
    times = (timing.fwd_left, timing.rev_straight_a, timing.rev_right)
    for name, value in zip(('fwd_turn', 'rev_straight', 'rev_turn'), times):
        PERPENDICULAR[f'case{index}_{name}_s'] = (value, 0.0, 30.0, 0.1)

PARALLEL = {
    **PARKING_COMMON,
    'gap_margin_m': (0.3, 0.1, 1.0, 0.01),
    'slot_depth_m': (0.0, 0.0, 3.0, 0.01),
    'entry_advance_m': (0.0, 0.0, 2.0, 0.01),
    'entry_lateral_m': (0.0, 0.0, 2.0, 0.01),
    'align_tolerance_deg': (3.0, 1.0, 10.0, 0.5),
    'center_tolerance_m': (0.08, 0.02, 0.3, 0.01),
}

def defaults(specs):
    return {name: spec[0] for name, spec in specs.items()}

def limits(specs):
    return {name: (type(spec[0]), spec[1], spec[2]) for name, spec in specs.items()}

def trackbars(specs):
    result = {}
    for name, (default, minimum, maximum, step) in specs.items():
        value_type = type(default)
        result[name] = (
            name, int(round((maximum - minimum) / step)),
            lambda pos, lo=minimum, inc=step, typ=value_type: typ(round(lo + pos * inc, 6)),
            lambda val, lo=minimum, inc=step: int(round((val - lo) / inc)),
        )
    return result
