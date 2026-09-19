#!/usr/bin/env python3
"""
MotionPlanningNode
──────────────────
* 입력
  - DetectionArray       : 장애물·신호등 정보
  - PathPlanningResult   : (필요 시) 경로 정보
  - String               : 신호등 색상
  - Bool                 : LiDAR 장애물 플래그
  - LaneInfo             : YOLOv8 차선 인식 결과  ← 새로 추가

* 출력
  - MotionCommand : steering (‑7 ~ +7), wheel speeds (0 ~ 255)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import (
    DetectionArray, PathPlanningResult, MotionCommand, LaneInfo
)

# ---------- 기본 토픽 이름 ----------
SUB_DETECTION_TOPIC_NAME    = "detections"
SUB_PATH_TOPIC_NAME         = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
SUB_LANEINFO_TOPIC_NAME      = "yolov8_lane_info"          # ← PID 제어를 위해서 추가
PUB_TOPIC_NAME               = "topic_control_signal"

# ---------- 주기 / 파라미터 ----------
TIMER_SEC = 0.1                          # 10 Hz
STEER_CMD_LIMIT = 7                      # ‑7 ~ +7
WHEEL_SPEED_DEFAULT = 255

# ---------- PID 계수 ----------
KP =  0.05
KI = 0.0
KD = 0.005


class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__("motion_planner_node")

        # ───── 파라미터 선언 ────────────────────────────────
        self.declare_parameter("sub_detection_topic", SUB_DETECTION_TOPIC_NAME)
        self.declare_parameter("sub_path_topic", SUB_PATH_TOPIC_NAME)
        self.declare_parameter("sub_traffic_light_topic", SUB_TRAFFIC_LIGHT_TOPIC_NAME)
        self.declare_parameter("sub_lidar_obstacle_topic", SUB_LIDAR_OBSTACLE_TOPIC_NAME)
        self.declare_parameter("sub_laneinfo_topic", SUB_LANEINFO_TOPIC_NAME)
        self.declare_parameter("pub_topic", PUB_TOPIC_NAME)
        self.declare_parameter("timer", TIMER_SEC)
        self.wheel_speed_default = self.declare_parameter("wheel_speed_default", WHEEL_SPEED_DEFAULT).value
        self.steer_cmd_limit = self.declare_parameter("steer_cmd_limit", STEER_CMD_LIMIT).value
        self.kp = self.declare_parameter("kp", KP).value
        self.ki = self.declare_parameter("ki", KI).value
        self.kd = self.declare_parameter("kd", KD).value
        self.offset_correction = self.declare_parameter("offset_correction", 10).value
        self.image_center_x_fallback = self.declare_parameter("image_center_x_fallback", 320).value
        self.red_light_stop_y_max = self.declare_parameter("red_light_stop_y_max", 160).value
        self.traffic_light_class = self.declare_parameter("traffic_light_class", "traffic_light").value

        # ───── QoS 설정 ────────────────────────────────────
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # ───── 내부 변수 ───────────────────────────────────
        self.detection_msg    = None
        self.path_msg         = None
        self.traffic_light_msg = None
        self.lidar_msg        = None
        self.laneinfo_msg     = None

        # PID 상태
        self.prev_error = 0.0
        self.int_error  = 0.0

        # ───── 서브스크라이버 ──────────────────────────────
        self.create_subscription(
            DetectionArray,
            self.get_parameter("sub_detection_topic").value,
            self.detection_cb,
            self.qos_profile
        )
        self.create_subscription(
            PathPlanningResult,
            self.get_parameter("sub_path_topic").value,
            self.path_cb,
            self.qos_profile
        )
        self.create_subscription(
            String,
            self.get_parameter("sub_traffic_light_topic").value,
            self.traffic_light_cb,
            self.qos_profile
        )
        self.create_subscription(
            Bool,
            self.get_parameter("sub_lidar_obstacle_topic").value,
            self.lidar_cb,
            self.qos_profile
        )
        self.create_subscription(                             # ← LaneInfo 구독
            LaneInfo,
            self.get_parameter("sub_laneinfo_topic").value,
            self.laneinfo_cb,
            self.qos_profile
        )

        # ───── 퍼블리셔 ────────────────────────────────────
        self.publisher = self.create_publisher(
            MotionCommand,
            self.get_parameter("pub_topic").value,
            self.qos_profile
        )

        # ───── 주기 실행 타이머 ────────────────────────────
        self.timer_period = self.get_parameter("timer").value
        self.create_timer(self.timer_period, self.timer_cb)

    # ───────────── 콜백들 ─────────────────────────────────
    def detection_cb(self, msg: DetectionArray):
        self.detection_msg = msg

    def path_cb(self, msg: PathPlanningResult):
        self.path_msg = msg                                # 필요 시 활용

    def traffic_light_cb(self, msg: String):
        self.traffic_light_msg = msg

    def lidar_cb(self, msg: Bool):
        self.lidar_msg = msg

    def laneinfo_cb(self, msg: LaneInfo):
        self.laneinfo_msg = msg

    # ───────────── PID 기반 조향 계산 ─────────────────────
    def steering_from_laneinfo(self) -> int:
        """
        LaneInfo.target_points[-1] 기준 x‑축 오차로 PID 계산
        반환값: ‑7 ~ +7 (int)
        """
        if self.laneinfo_msg is None or not self.laneinfo_msg.target_points:
            return 0

        tgt = self.laneinfo_msg.target_points[-1]          # 가장 먼 target 선택
        # 개선안: 중간 지점 사용 (앞으로 더 먼 시점을 보게)
        # mid_idx = len(self.laneinfo_msg.target_points) //3
        # tgt = self.laneinfo_msg.target_points[mid_idx]


        img_center_x = self.laneinfo_msg.image_width / 2 if hasattr(self.laneinfo_msg, "image_width") else self.image_center_x_fallback

        
        
        # error = float(tgt.target_x) - img_center_x

        ##  임시추가  ( 왼쪽에 닿으면 + 오른쪽선에 닿으면 -)
        error = float(tgt.target_x) + self.offset_correction - img_center_x


        # PID
        self.int_error += error * self.timer_period
        deriv = (error - self.prev_error) / self.timer_period
        self.prev_error = error

        steer = self.kp * error + self.ki * self.int_error + self.kd * deriv
        steer = max(min(int(round(steer)),  self.steer_cmd_limit), -self.steer_cmd_limit)
        return steer

    # # ───────────── 주기적 제어 루프 ───────────────────────
    # def timer_cb(self):
    #     steering_cmd = 0
    #     left_speed   = WHEEL_SPEED_DEFAULT
    #     right_speed  = WHEEL_SPEED_DEFAULT

    #     # LiDAR 장애물 우선 정지
    #     if self.lidar_msg and self.lidar_msg.data:
    #         steering_cmd = 0
    #         left_speed   = 0
    #         right_speed  = 0

    #     # # 빨간 신호등 정지
    #     # elif self.traffic_light_msg and self.traffic_light_msg.data == "Red":
    #     #     steering_cmd = 0
    #     #     left_speed   = 0
    #     #     right_speed  = 0


    #     elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red':
    #         # 빨간색 신호등을 감지한 경우
    #         for detection in self.detection_data.detections:
    #             if detection.class_name=='traffic_light':
    #                 x_min = int(detection.bbox.center.position.x - detection.bbox.size.x / 2) # bbox의 좌측상단 꼭짓점 x좌표
    #                 x_max = int(detection.bbox.center.position.x + detection.bbox.size.x / 2) # bbox의 우측하단 꼭짓점 x좌표
    #                 y_min = int(detection.bbox.center.position.y - detection.bbox.size.y / 2) # bbox의 좌측상단 꼭짓점 y좌표
    #                 y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2) # bbox의 우측하단 꼭짓점 y좌표

    #                 if y_max < 150:
    #                     # 신호등 위치에 따른 정지명령 결정
    #                     self.steering_command = 0 
    #                     self.left_speed_command = 0 
    #                     self.right_speed_command = 0
    #     # 차선 기반 PID 조향
    #     else:
    #         steering_cmd = self.steering_from_laneinfo()

    #     # ── 메시지 퍼블리시 ───────────────────────────────
    #     self.get_logger().info(
    #         f"steer {steering_cmd:+d} | L {left_speed} R {right_speed}"
    #     )

    #     cmd = MotionCommand()
    #     cmd.steering    = steering_cmd
    #     cmd.left_speed  = left_speed
    #     cmd.right_speed = right_speed
    #     self.publisher.publish(cmd)


    def timer_cb(self):
        steering_cmd = 0
        left_speed   = self.wheel_speed_default
        right_speed  = self.wheel_speed_default

        # LiDAR 장애물 우선 정지
        if self.lidar_msg and self.lidar_msg.data:
            steering_cmd = 0
            left_speed   = 0
            right_speed  = 0

        # 빨간 신호등: 감지된 bbox 위치를 보고 정지 여부 결정
        elif self.traffic_light_msg and self.traffic_light_msg.data == "Red":
            stop_flag = False
            if self.detection_msg:
                for detection in self.detection_msg.detections:
                    if detection.class_name == self.traffic_light_class:
                        # bbox 하단 y좌표 계산
                        y_max = detection.bbox.center.position.y + detection.bbox.size.y / 2
                        # y_max가 150픽셀 미만일 때(상단에 있을 때)만 정지
                        if y_max < self.red_light_stop_y_max:
                            stop_flag = True
                        break

            if stop_flag:
                steering_cmd = 0
                left_speed   = 0
                right_speed  = 0
            else:
                # 시야 상의 빨간불이 너무 멀거나 인식되지 않으면 평소대로 차선 PID
                steering_cmd = self.steering_from_laneinfo()

        # 그 외(초록불·노란불·신호 미검출)는 차선 기반 PID
        else:
            steering_cmd = self.steering_from_laneinfo()

        # ── 메시지 퍼블리시 ───────────────────────────────
        self.get_logger().info(
            f"steer {steering_cmd:+d} | L {left_speed} R {right_speed}"
        )

        cmd = MotionCommand()
        cmd.steering    = steering_cmd
        cmd.left_speed  = left_speed
        cmd.right_speed = right_speed
        self.publisher.publish(cmd)


# ───────────── 메인 엔트리 ───────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
