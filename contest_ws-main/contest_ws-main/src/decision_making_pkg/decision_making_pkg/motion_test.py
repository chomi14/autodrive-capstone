#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces_pkg.msg import LaneInfo, MotionCommand
import re

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_mission')

        # Subscriptions
        self.create_subscription(LaneInfo, '/cam0/lane_info', self.lane_info_callback, 10)
        self.create_subscription(String,   'traffic_light_result', self.traffic_callback,   10)
        self.create_subscription(String,   'obstacle_result',      self.obstacle_callback, 10)

        # Publisher
        self.motion_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        # 장애물 기준값 및 연속 감지/해제 횟수
        self.obstacle_threshold_2to1   = 1000    # 2→1 전환용
        self.obstacle_threshold_1to2   = 10000   # 1→2 전환용
        self.obstacle_required_count       = 3
        self.obstacle_clear_required_count = 3
        self.obstacle_detect_counter       = 0
        self.obstacle_clear_counter        = 0
        self.wait_for_obstacle_clear       = False

        self.coming = 100000

        # 2→1 전환 후 스티어링 안정화 대기
        self.wait_for_steer_stable     = False
        self.steer_stable_counter      = 0
        self.steer_stable_required     = 5

        # 신호등 기준값
        self.traffic_area_threshold = 10000

        # 빨간불 연속 감지/해제 카운터
        self.red_required_count       = 3
        self.red_detect_counter       = 0
        self.red_clear_required_count = 3
        self.red_clear_counter        = 0

        # 상태 변수
        self.is_changing_lane   = False
        self.wait_for_red_clear = False
        self.current_lane       = None
        self.target_lane        = 2

        # lane 1 설정
        self.lane1_angle_weight      = 0.7
        self.lane1_position_weight   = 0.05
        self.lane1_normal_speed      = 100
        self.lane1_lane_change_speed = 100

        # lane 2 설정
        self.lane2_angle_weight      = 0.7
        self.lane2_position_weight   = 0.05
        self.lane2_normal_speed      = 255
        self.lane2_lane_change_speed = 100

    def get_lane_config(self):
        if self.current_lane == 1:
            return (
                self.lane1_angle_weight,
                self.lane1_position_weight,
                self.lane1_normal_speed,
                self.lane1_lane_change_speed
            )
        else:
            return (
                self.lane2_angle_weight,
                self.lane2_position_weight,
                self.lane2_normal_speed,
                self.lane2_lane_change_speed
            )

    def obstacle_callback(self, msg: String):
        if self.wait_for_steer_stable:
            return

        match = re.match(r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)', msg.data)
        if not match:
            return

        detected = match.group(1).lower() == 'true'
        area = float(match.group(2))
        self.get_logger().info(f"🚧 Obstacle: detected={detected}, area={area}")

        if self.current_lane == 2:
            threshold = self.obstacle_threshold_2to1
        elif self.current_lane == 1:
            threshold = self.obstacle_threshold_1to2
        else:
            threshold = self.obstacle_threshold_2to1

        if detected and area > threshold:
            self.obstacle_clear_counter = 0
            self.obstacle_detect_counter += 1

            if not self.wait_for_obstacle_clear and self.obstacle_detect_counter >= self.obstacle_required_count:
                self.target_lane = 1 if self.target_lane == 2 else 2
                self.is_changing_lane = True
                self.wait_for_obstacle_clear = True

                if self.current_lane == 2 and self.target_lane == 1:
                    self.wait_for_steer_stable = True
                    self.steer_stable_counter = 0
                    self.obstacle_area_history = []  # 면적 크기 변화를 저장할 리스트 초기화

        else:
            self.obstacle_detect_counter = 0
            if self.wait_for_obstacle_clear:
                self.obstacle_clear_counter += 1
                if self.obstacle_clear_counter >= self.obstacle_clear_required_count:
                    self.wait_for_obstacle_clear = False
                    self.obstacle_clear_counter = 0
                    self.get_logger().info("🟢 Obstacle cleared: allow next lane switch")

        # 스티어링 안정화 이후 1차선에서 장애물 크기 변화 관찰
        if self.current_lane == 1 and not self.wait_for_steer_stable:
            if detected:
                self.obstacle_area_history.append(area)

                # 최근 5개의 장애물 크기 변화만 관찰
                if len(self.obstacle_area_history) > 5:
                    self.obstacle_area_history.pop(0)

                if len(self.obstacle_area_history) == 5:
                    # 장애물 크기가 대체적으로 작아지고 있을 경우
                    if all(x > y for x, y in zip(self.obstacle_area_history, self.obstacle_area_history[1:])):
                        self.get_logger().info("📉 Obstacle shrinking, waiting to proceed...")
                        # 장애물이 일정 크기 이하로 작아지면 다시 출발
                        if area < self.obstacle_threshold_1to2 / 2:  # 임의의 기준값, 상황에 맞게 조정 필요
                            self.get_logger().info("✅ Obstacle small enough, proceeding!")
                            cmd = MotionCommand()
                            cmd.left_speed = self.lane1_normal_speed
                            cmd.right_speed = self.lane1_normal_speed
                            cmd.steering = 0
                            self.motion_pub.publish(cmd)
                    # 장애물 크기가 대체적으로 커지고 있을 경우
                    elif all(x < y for x, y in zip(self.obstacle_area_history, self.obstacle_area_history[1:])):
                        self.get_logger().info("📈 Obstacle growing, need additional logic here...")
                        # 이곳에 커지고 있는 장애물을 위한 추가 로직을 구현하면 됩니다.

    def traffic_callback(self, msg: String):
        match = re.match(
            r'Detected:\s*(\w+),\s*Area:\s*([\d.]+)(?:,\s*Color:\s*(\w+))?',
            msg.data
        )
        if not match:
            return

        detected = match.group(1).lower() == 'true'
        area     = float(match.group(2))
        color    = (match.group(3) or '').lower()

        if detected and area > self.traffic_area_threshold and color == 'red':
            self.red_clear_counter = 0
            self.red_detect_counter += 1
            if not self.wait_for_red_clear and self.red_detect_counter >= self.red_required_count:
                cmd = MotionCommand()
                cmd.left_speed  = 0
                cmd.right_speed = 0
                cmd.steering    = 0
                self.motion_pub.publish(cmd)
                self.wait_for_red_clear = True
        else:
            self.red_detect_counter = 0
            if self.wait_for_red_clear:
                self.red_clear_counter += 1
                if self.red_clear_counter >= self.red_clear_required_count:
                    self.wait_for_red_clear = False
                    self.red_clear_counter = 0

    def lane_info_callback(self, msg: LaneInfo):
        # 빨간불 정지 중이면 무시
        if self.wait_for_red_clear:
            self.get_logger().warning("⛔ Red light stop")
            return

        self.current_lane    = msg.lane_num
        steering_angle       = msg.steering_angle
        vehicle_position_x   = msg.vehicle_position_x

        # 차선 변경 완료 감지
        if (self.is_changing_lane and
            self.current_lane == self.target_lane and
            abs(vehicle_position_x) <= 100):
            self.is_changing_lane = False
            self.get_logger().info(f"✅ Lane change complete: now on lane {self.current_lane}")

        # 가중치와 속도 가져오기
        angle_w, pos_w, normal_spd, change_spd = self.get_lane_config()

        # steering 계산
        if self.is_changing_lane:
            steering = -10 if self.target_lane == 1 else 10
            left_spd = right_spd = change_spd
            self.get_logger().info("🔁 Changing lane…")
        else:
            mapped   = (steering_angle / 50.0) * 10.0 * angle_w
            adjust   = -vehicle_position_x * pos_w
            steering = max(-10, min(mapped + adjust, 10))
            left_spd  = right_spd = normal_spd
            if self.current_lane == 1 and steering < -8:
                left_spd = 200

        # 2→1 전환 후 steer 안정화 체크
        if self.wait_for_steer_stable:
            if abs(steering) <= 3:
                self.steer_stable_counter += 1
                if self.steer_stable_counter >= self.steer_stable_required:
                    # 안정화 완료 → 장애물 감지 재허용
                    self.wait_for_obstacle_clear = False
                    self.wait_for_steer_stable  = False
                    self.obstacle_detect_counter = 0
                    self.obstacle_clear_counter  = 0
                    self.get_logger().info("✅ Steering stabilized: allow next obstacle detection")
            else:
                # 기준치 벗어나면 카운터 리셋
                self.steer_stable_counter = 0

        # MotionCommand 발행
        cmd = MotionCommand()
        cmd.steering    = int(steering)
        cmd.left_speed  = left_spd
        cmd.right_speed = right_spd
        self.motion_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MotionNode interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
