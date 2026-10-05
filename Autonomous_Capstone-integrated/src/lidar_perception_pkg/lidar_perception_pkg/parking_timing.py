"""Existing ver2 perpendicular parking timing presets, shared with the gated controller."""
from dataclasses import dataclass

@dataclass(frozen=True)
class ParkingTiming:
    fwd_left: float
    rev_straight_a: float
    rev_right: float
    fwd_straight_c: float
    fwd_right: float
    finish: float


# 1구간: 우측 거리 1.0m 이하
TIMING_LE_1_0M = ParkingTiming(
    fwd_left=10.0,        # (S1) -7 조향 전진
    rev_straight_a=2.5,  # (S3)  0 조향 후진
    rev_right=2.0,       # (S4) +7 조향 후진
    fwd_straight_c=1.5,  # (S7)  0 조향 전진
    fwd_right=12.0,      # (S8) +7 조향 전진
    finish=30.0,         # (S9)  0 조향 전진 후 도착선 통과
)

# 2구간: 우측 거리 1.0m 초과 ~ 1.5m 이하
TIMING_1_0_TO_1_5M = ParkingTiming(
    fwd_left=9.0,        # (S1) -7 조향 전진
    rev_straight_a=2.0,  # (S3)  0 조향 후진
    rev_right=3.0,       # (S4) +7 조향 후진
    fwd_straight_c=1.5,  # (S7)  0 조향 전진
    fwd_right=12.0,      # (S8) +7 조향 전진
    finish=30.0,         # (S9)  0 조향 전진 후 도착선 통과
)

# 3구간: 우측 거리 1.5m 초과 ~ 2.0m 이하
TIMING_1_5_TO_2_0M = ParkingTiming(
    fwd_left=9.0,        # (S1) -7 조향 전진
    rev_straight_a=2.0,  # (S3)  0 조향 후진
    rev_right=3.0,       # (S4) +7 조향 후진
    fwd_straight_c=1.5,  # (S7)  0 조향 전진
    fwd_right=12.0,      # (S8) +7 조향 전진
    finish=30.0,         # (S9)  0 조향 전진 후 도착선 통과
)

# 4구간: 우측 거리 2.0m 초과 ~ 2.5m 이하
TIMING_2_0_TO_2_5M = ParkingTiming(
    fwd_left=8.0,        # (S1) -7 조향 전진
    rev_straight_a=2.0,  # (S3)  0 조향 후진
    rev_right=2.5,       # (S4) +7 조향 후진
    fwd_straight_c=1.5,  # (S7)  0 조향 전진
    fwd_right=12.0,      # (S8) +7 조향 전진
    finish=30.0,         # (S9)  0 조향 전진 후 도착선 통과
)

