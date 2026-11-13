import math
from .messages import MotionCommand


class MotionPlanner:
    def __init__(
        self,
        stanley_gain=0.0,
        stanley_softening=1e-3,
        max_steering=100.0,
        max_steering_angle_deg=20.0,
        lookahead_steps=5,
        heading_step=1,
        car_center_x=320.0,
        car_center_y=179.0,
        default_left_speed=120,
        default_right_speed=120,
        stop_on_red=True,
        red_light_stop_y_max=300,
        steering_sign=1,
        heading_gain=0.7,
        traffic_light_class_name="traffic_light",
        red_light_confirm_frames=1,
    ):
        self.stanley_gain = stanley_gain
        self.stanley_softening = stanley_softening
        self.max_steering = max_steering
        self.max_steering_angle_deg = max_steering_angle_deg
        self.lookahead_steps = lookahead_steps
        self.heading_step = heading_step
        self.car_center_x = car_center_x
        self.car_center_y = car_center_y
        self.default_left_speed = default_left_speed
        self.default_right_speed = default_right_speed
        self.stop_on_red = stop_on_red
        self.red_light_stop_y_max = red_light_stop_y_max
        self.steering_sign = steering_sign
        self.heading_gain = heading_gain
        self.traffic_light_class_name = traffic_light_class_name
        # 정지선을 넘은(=bbox 아랫변이 기준선보다 위로 올라온) 상태가 이 프레임 수만큼
        # 연속돼야 실제로 정지한다(1=기존처럼 즉시 정지). 오탐/한 프레임 흔들림으로
        # 갑자기 급정지하는 것을 막기 위한 확인 카운터.
        self.red_light_confirm_frames = max(1, int(red_light_confirm_frames))
        self._red_confirm_count = 0
        self.path_data = None
        # 디버그/표시용 최근 값 (heading 오차, cross-track 오차, 목표 조향)
        self.last_heading_deg = 0.0
        self.last_cte = 0.0
        self.last_target_steer = 0
        self.last_reference_point = None
        # 빨간불 정지 확인 진행 상태(avoid.py가 퍼센트 바 표시에 사용)
        self.last_red_light_close = False   # 이번 프레임에 "정지선을 넘은 빨간불"이 보였는지
        self.last_red_confirm_count = 0     # 연속 확인 프레임 수(0~red_light_confirm_frames)

    def plan(self, detections, path_result, traffic_light="None", lidar_obstacle=False) -> MotionCommand:
        if lidar_obstacle:
            return MotionCommand(0, 0, 0)

        # 정지선(red_light_stop_y_max)보다 신호등 bbox 아랫변이 위로 올라왔는지만
        # 이번 프레임 기준으로 먼저 판단(red_close). 실제 정지는 이 상태가
        # red_light_confirm_frames번 연속돼야 확정된다(_red_confirm_count).
        red_close = False
        if self.stop_on_red and traffic_light == "Red":
            for detection in detections.detections:
                if detection.class_name == self.traffic_light_class_name:
                    y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2)
                    if y_max < self.red_light_stop_y_max:
                        red_close = True
                        break

        self._red_confirm_count = (
            min(self.red_light_confirm_frames, self._red_confirm_count + 1) if red_close else 0
        )
        self.last_red_light_close = red_close
        self.last_red_confirm_count = self._red_confirm_count

        if self._red_confirm_count >= self.red_light_confirm_frames:
            return MotionCommand(0, 0, 0)

        if len(path_result.x_points) >= 2:
            raw_path = list(zip(path_result.x_points, path_result.y_points))
            self.path_data = sorted(raw_path, key=lambda p: p[1], reverse=True)
            steering = self.compute_stanley_steering()
        else:
            # A momentary YOLO/lane miss must not snap the steering back to
            # center at full speed. Keep the most recent valid steering command
            # until a new valid path produces an updated command.
            steering = self.last_target_steer
            self.last_reference_point = None

        return MotionCommand(steering=steering, left_speed=self.default_left_speed, right_speed=self.default_right_speed)

    def compute_stanley_steering(self):
        if not self.path_data or len(self.path_data) < 2:
            self.last_reference_point = None
            return 0

        vehicle_pos = (self.car_center_x, self.car_center_y)
        nearest_idx = self.find_nearest_index(vehicle_pos, self.path_data)
        if nearest_idx is None:
            self.last_reference_point = None
            return 0

        lookahead = int(max(1, self.lookahead_steps))
        ref_idx = min(len(self.path_data) - 2, max(nearest_idx, lookahead))
        self.last_reference_point = self.path_data[ref_idx]
        ref_heading = self.path_heading(ref_idx)
        vehicle_heading = -math.pi / 2

        heading_error = self.normalize_angle(ref_heading - vehicle_heading)
        cross_track_error = self.signed_cross_track_error(vehicle_pos, ref_idx)

        v = 1.0
        stanley_term = -math.atan2(self.stanley_gain * cross_track_error, v + self.stanley_softening)
        # heading_gain: heading 오차 항의 가중치(실시간 튜닝 대상). 예전 고정값 0.7.
        steering = (heading_error + stanley_term) * self.heading_gain

        steering_deg = math.degrees(steering)

        steering_deg_clamped = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, steering_deg))
        steering_ratio = steering_deg_clamped / self.max_steering_angle_deg
        # steering_sign으로 아두이노 좌/우 규약(+=왼쪽, -=오른쪽)에 맞춰 부호를 반전한다.
        steering_cmd = steering_ratio * self.max_steering * self.steering_sign
        steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

        # 표시용 값 저장 (출력은 드라이버에서 현재 조향과 함께 한 줄로 처리)
        self.last_heading_deg = math.degrees(heading_error)
        self.last_cte = cross_track_error
        self.last_target_steer = int(round(steering_cmd))
        return int(round(steering_cmd))

    @staticmethod
    def find_nearest_index(position, path_points):
        px, py = position
        min_dist = float("inf")
        min_idx = None
        for i, (x, y) in enumerate(path_points):
            dist = (x - px) ** 2 + (y - py) ** 2
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        return min_idx

    def path_heading(self, idx):
        step = max(1, int(self.heading_step))
        if idx >= len(self.path_data) - step:
            idx = len(self.path_data) - step - 1
        x1, y1 = self.path_data[idx]
        x2, y2 = self.path_data[idx + step]
        return math.atan2(y2 - y1, x2 - x1)

    def signed_cross_track_error(self, position, idx):
        px, py = position
        if idx >= len(self.path_data) - 1:
            idx -= 1
        x1, y1 = self.path_data[idx]
        x2, y2 = self.path_data[idx + 1]
        dx, dy = x2 - x1, y2 - y1
        cross = dx * (py - y1) - dy * (px - x1)
        length = math.hypot(dx, dy) + 1e-6
        return cross / length

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
