"""Camera-only obstacle mission using the existing track driving pipeline.

The inherited MissionCore matches bbox bottom footprints to the selected lane
mask and immediately requests the opposite lane at obstacle_near_y, without an
obstacle stop or speed cap.
No LaserScan subscription or LiDAR correction is used by this executable.
"""
from skku_track_drive_pkg.controller_liveness import run_controller
from .mission_controller_node import MissionControllerNode

# 사용자 조정값은 mission_bbox_only.launch.py 상단 및 Mission Tuner GUI에서 관리합니다.
# 1009의 live bev_src_* / ROI 튜닝은 부모 TrackController가 그대로 관리합니다.
# 자주 조정하는 bbox 파라미터:
# obstacle_near_y       : 640x480 영상 기준 가까운 장애물의 bbox 하단 y
# 현재 차선 mask와 bbox 하단이 겹치고 obstacle_near_y 이상이면 즉시 반대 차선 요청.
# 과거 면적/연속 확인/반대 차선 검사/유지시간/쿨다운/회피 PWM 설정은 사용하지 않습니다.
# 전환 후 자동 회피는 5초간 막습니다. GUI의 L은 즉시 반전하며 5초를 다시 시작합니다.


class BboxMissionControllerNode(MissionControllerNode):
    """Named opt-in mode for the existing bbox-only obstacle logic."""

    def initialize_mode(self):
        super().initialize_mode()
        self.declare_parameter('initial_target_lane', 2)
        lane = self.get_parameter('initial_target_lane').value
        if lane not in (1, 2):
            raise ValueError('initial_target_lane must be 1 or 2')
        self.mission.target_lane = lane
        self.get_logger().info(
            'BBox mission: current lane mask + bbox bottom -> immediate opposite lane; no obstacle stop/speed cap'
        )


def main(args=None):
    run_controller(BboxMissionControllerNode, args)
