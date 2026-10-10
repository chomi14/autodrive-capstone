"""Camera-only obstacle mission using the existing track driving pipeline.

The inherited MissionCore selects only obstacle boxes that overlap the current
projected path.  Nearness is determined from the bottom edge of each bbox.
No LaserScan subscription or LiDAR correction is used by this executable.
"""
from .controller_liveness import run_controller
from .mission_controller_node import MissionControllerNode

# 사용자 조정값은 기존 mission_tuning.yaml 및 Mission Tuner GUI에서 관리합니다.
# 자주 조정하는 bbox 파라미터:
# obstacle_near_y       : 640x480 영상 기준 가까운 장애물의 bbox 하단 y
# path_margin_px        : bbox와 주행 경로를 비교할 때 좌우 여유 픽셀
# obstacle_confirm_frames: 차선 변경 전 연속 검출 프레임 수
# obstacle_clear_frames : 장애물이 사라졌다고 판단할 연속 프레임 수
# alternate_min_area_px : 대체 차선이 보인다고 판단할 최소 mask 면적
# avoid_hold_s / avoid_speed: 회피 차선 유지 시간 / 회피 중 PWM 제한


class BboxMissionControllerNode(MissionControllerNode):
    """Named opt-in mode for the existing bbox-only obstacle logic."""

    def initialize_mode(self):
        super().initialize_mode()
        self.get_logger().info(
            'BBox-only mission enabled: camera path overlap + bbox bottom; LiDAR disabled'
        )


def main(args=None):
    run_controller(BboxMissionControllerNode, args)
