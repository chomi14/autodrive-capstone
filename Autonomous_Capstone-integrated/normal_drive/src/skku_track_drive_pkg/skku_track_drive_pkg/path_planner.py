import numpy as np
from scipy.interpolate import CubicSpline
from .messages import PathPlanningResult


class PathPlanner:
    def __init__(self, car_center_point=(320, 179)):
        self.car_center_point = car_center_point

    def plan(self, lane_info) -> PathPlanningResult:
        target_points = lane_info.target_points
        if len(target_points) < 3:
            return PathPlanningResult()

        x_points, y_points = zip(*[(tp.target_x, tp.target_y) for tp in target_points])

        # 예전에는 여기서 차량 중심점(car_center_point)을 경로에 강제로 넣었는데,
        # 그러면 경로가 차를 관통해 cross-track error가 항상 0이 되어 Stanley 횡방향
        # 제어가 무력화됐다. 경로는 '차선 중심선'만으로 만들고, 차량과의 횡방향 이탈은
        # MotionPlanner가 CTE로 계산하게 둔다.
        y_points_list = list(y_points)
        x_points_list = list(x_points)

        sorted_points = sorted(zip(y_points_list, x_points_list), key=lambda point: point[0])
        y_points, x_points = zip(*sorted_points)

        try:
            cs = CubicSpline(y_points, x_points, bc_type="natural")
            y_new = np.linspace(min(y_points), max(y_points), 100)
            x_new = cs(y_new)
            return PathPlanningResult(x_points=list(map(float, x_new)), y_points=list(map(float, y_new)))
        except Exception as e:
            print(f"[PathPlanner] fail: {e}")
            return PathPlanningResult()
