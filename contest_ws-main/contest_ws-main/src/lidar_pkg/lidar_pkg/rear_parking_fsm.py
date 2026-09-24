import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from interfaces_pkg.msg import MotionCommand
from tf_transformations import euler_from_quaternion
import numpy as np

class RearParkingFSM(Node):
    def __init__(self):
        super().__init__('rear_parking_fsm')

        self.lidar_sub = self.create_subscription(LaserScan, 'lidar_processed', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(MotionCommand, 'motion_command', 10)

        self.state = 'WAIT_FOR_CAR1'
        self.car270_seen_count = 0
        self.pause_start_time = None
        self.forward_straight_start_time = None
        self.start_yaw = None
        self.current_yaw = None
        self.start_time = None
        self.first_detect_time = None

        self.get_logger().info('Rear Parking FSM with Yaw Tracking started.')

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def lidar_callback(self, msg):
        ranges = msg.ranges

        def detected(angle, threshold=1.5):
            if not ranges:
                return False
            idx = angle % len(ranges)
            return 0.05 < ranges[idx] < threshold

        def not_detected(angle, threshold=1.5):
            if not ranges:
                return True
            idx = angle % len(ranges)
            return not (0.05 < ranges[idx] < threshold)

        now = self.get_clock().now().nanoseconds

        if self.state == 'WAIT_FOR_CAR1':
            # FSM 시작 시간 초기화
            if self.start_time is None:
                self.start_time = now

            # 전진 명령
            motion = MotionCommand()
            motion.steering = 0
            motion.left_speed = 150
            motion.right_speed = 150
            self.cmd_pub.publish(motion)

            # [1] 시작 후 2초 이내는 감지 무시
            if now - self.start_time < 2e9:
                return

            # [2] 감지된 경우
            if detected(270):
                if self.first_detect_time is None:
                    self.first_detect_time = now
                    self.car270_seen_count = 1
                    self.get_logger().info('270° 첫 감지 기록')
                else:
                    if now - self.first_detect_time <= 2e9:
                        self.car270_seen_count += 1
                        self.get_logger().info(f'270° 재감지 → count = {self.car270_seen_count}')
                        if self.car270_seen_count == 2:
                            self.state = 'REVERSE_RIGHT'
                            self.start_yaw = self.current_yaw
                            self.get_logger().info('270° 두 번째 감지 → REVERSE_RIGHT 진입, yaw 저장')
                    else:
                        self.car270_seen_count = 1
                        self.first_detect_time = now
                        self.get_logger().info('270° 재감지 지연 → count 초기화')

        elif self.state == 'REVERSE_RIGHT':
            if self.start_yaw is not None and self.current_yaw is not None:
                yaw_diff = abs(self.normalize_angle(self.current_yaw - self.start_yaw))
                if yaw_diff >= np.radians(85):
                    self.state = 'REVERSE_STRAIGHT'
                    self.get_logger().info(f'Yaw 변화 {np.degrees(yaw_diff):.1f}° → REVERSE_STRAIGHT 진입')
                    return

            motion = MotionCommand()
            motion.steering = 30
            motion.left_speed = -130
            motion.right_speed = -130
            self.cmd_pub.publish(motion)

        elif self.state == 'REVERSE_STRAIGHT':
            if not_detected(270) and not_detected(90):
                self.state = 'PAUSE_CENTER'
                self.pause_start_time = now
                self.get_logger().info('차량이 양쪽에서 사라짐 → 3초 정지 시작')
            else:
                motion = MotionCommand()
                motion.steering = 0
                motion.left_speed = -130
                motion.right_speed = -130
                self.cmd_pub.publish(motion)

        elif self.state == 'PAUSE_CENTER':
            if now - self.pause_start_time >= 3e9:
                self.state = 'FORWARD_STRAIGHT'
                self.forward_straight_start_time = now
                self.get_logger().info('정지 완료 → FORWARD_STRAIGHT')

        elif self.state == 'FORWARD_STRAIGHT':
            motion = MotionCommand()
            motion.steering = 0
            motion.left_speed = 150
            motion.right_speed = 150
            self.cmd_pub.publish(motion)

            if now - self.forward_straight_start_time >= 1.5e9:
                if not_detected(90) and not_detected(270):
                    self.state = 'FORWARD_RIGHT'
                    self.get_logger().info('Still clear → FORWARD_RIGHT')

        elif self.state == 'FORWARD_RIGHT':
            if detected(270):
                self.state = 'FORWARD_OUT'
                self.get_logger().info('Car1 detected again → FORWARD_OUT')
            else:
                motion = MotionCommand()
                motion.steering = 30
                motion.left_speed = 150
                motion.right_speed = 150
                self.cmd_pub.publish(motion)

        elif self.state == 'FORWARD_OUT':
            motion = MotionCommand()
            motion.steering = 0
            motion.left_speed = 150
            motion.right_speed = 150
            self.cmd_pub.publish(motion)
            self.get_logger().info('Straight OUT → Parking complete.')

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle

    def destroy(self):
        motion = MotionCommand()
        motion.steering = 0
        motion.left_speed = 0
        motion.right_speed = 0
        self.cmd_pub.publish(motion)

def main(args=None):
    rclpy.init(args=args)
    node = RearParkingFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
