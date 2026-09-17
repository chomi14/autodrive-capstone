import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from interfaces_pkg.msg import MotionCommand


PUB_TOPIC_NAME = "topic_control_signal"
TIMER = 0.05

START_DELAY = 0.0
START_POSITION = 1
OBSTACLE_LAYOUT = 1
MIRROR_DIRECTION = 0

PARK_SPEED = 90
OUT_SPEED = 100
STEER_MAX = 6
STOP_TIME = 4.0


class ParkingMissionNode(Node):
    def __init__(self):
        super().__init__("parking_mission_node")

        self.pub_topic = self.declare_parameter("pub_topic", PUB_TOPIC_NAME).value
        self.timer_period = self.declare_parameter("timer", TIMER).value
        self.start_delay = self.declare_parameter("start_delay", START_DELAY).value
        self.start_position = self.declare_parameter(
            "start_position", START_POSITION).value
        self.obstacle_layout = self.declare_parameter(
            "obstacle_layout", OBSTACLE_LAYOUT).value
        self.mirror_direction = self.declare_parameter(
            "mirror_direction", MIRROR_DIRECTION).value

        self.park_speed = self.declare_parameter("park_speed", PARK_SPEED).value
        self.out_speed = self.declare_parameter("out_speed", OUT_SPEED).value
        self.steer_max = self.declare_parameter("steer_max", STEER_MAX).value
        self.stop_time = self.declare_parameter("stop_time", STOP_TIME).value

        self.reverse_turn_time = self.declare_parameter(
            "reverse_turn_time", 1.35).value
        self.reverse_counter_time = self.declare_parameter(
            "reverse_counter_time", 1.15).value
        self.reverse_straight_time = self.declare_parameter(
            "reverse_straight_time", 0.75).value
        self.forward_turn_time = self.declare_parameter(
            "forward_turn_time", 1.20).value
        self.forward_straight_time = self.declare_parameter(
            "forward_straight_time", 1.35).value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.publisher = self.create_publisher(
            MotionCommand, self.pub_topic, self.qos_profile)

        self.sequence = self.build_sequence()
        self.sequence_start_time = self.get_clock().now()
        self.last_step_index = None

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            "parking sequence ready: "
            f"start_position={self.start_position}, "
            f"obstacle_layout={self.obstacle_layout}, "
            f"mirror_direction={self.get_direction()}"
        )

    def build_sequence(self):
        direction = self.get_direction()
        steer = int(self.steer_max) * direction
        park_speed = -abs(int(self.park_speed))
        out_speed = abs(int(self.out_speed))

        if int(self.obstacle_layout) == 2:
            reverse_turn_time = self.reverse_turn_time + 0.15
            reverse_counter_time = self.reverse_counter_time + 0.10
        else:
            reverse_turn_time = self.reverse_turn_time
            reverse_counter_time = self.reverse_counter_time

        return [
            ("START_DELAY", self.start_delay, 0, 0, 0),
            ("REVERSE_TURN_IN", reverse_turn_time,
             steer, park_speed, park_speed),
            ("REVERSE_COUNTER_STEER", reverse_counter_time,
             -steer, park_speed, park_speed),
            ("REVERSE_STRAIGHTEN", self.reverse_straight_time,
             0, park_speed, park_speed),
            ("PARKED_STOP", self.stop_time, 0, 0, 0),
            ("FORWARD_TURN_OUT", self.forward_turn_time,
             -steer, out_speed, out_speed),
            ("FORWARD_STRAIGHT_OUT", self.forward_straight_time,
             0, out_speed, out_speed),
            ("DONE_STOP", 9999.0, 0, 0, 0),
        ]

    def get_direction(self):
        if int(self.mirror_direction) in (-1, 1):
            return int(self.mirror_direction)

        start_position = int(self.start_position)
        if start_position in (2, 4):
            return -1
        return 1

    def timer_callback(self):
        elapsed = (
            self.get_clock().now() - self.sequence_start_time
        ).nanoseconds / 1e9

        accumulated = 0.0
        for index, step in enumerate(self.sequence):
            name, duration, steering, left_speed, right_speed = step
            accumulated += float(duration)
            if elapsed <= accumulated:
                self.publish_motion(steering, left_speed, right_speed)
                if index != self.last_step_index:
                    self.last_step_index = index
                    self.get_logger().info(
                        f"parking_state: {name}, "
                        f"steering: {steering}, "
                        f"left_speed: {left_speed}, "
                        f"right_speed: {right_speed}"
                    )
                return

        self.publish_motion(0, 0, 0)

    def publish_motion(self, steering, left_speed, right_speed):
        msg = MotionCommand()
        msg.steering = int(steering)
        msg.left_speed = int(left_speed)
        msg.right_speed = int(right_speed)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        node.publish_motion(0, 0, 0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
