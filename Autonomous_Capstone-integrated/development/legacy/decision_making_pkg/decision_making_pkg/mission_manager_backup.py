import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import Bool, Int32, String
from interfaces_pkg.msg import DetectionArray


SUB_DETECTION_TOPIC_NAME = "detections"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"

PUB_TARGET_LANE_TOPIC_NAME = "mission_target_lane"
PUB_FORCE_STOP_TOPIC_NAME = "mission_force_stop"
PUB_SPEED_LIMIT_TOPIC_NAME = "mission_speed_limit"
PUB_STATE_TOPIC_NAME = "mission_state"

TIMER = 0.1

DEFAULT_LANE = 2
LANE1 = 1
LANE2 = 2

IMAGE_CENTER_X = 320.0
OBSTACLE_CLASS_NAME = "obstacle"
OBSTACLE_NEAR_Y = 120.0
OBSTACLE_LOST_TIMEOUT = 1.5
LANE_CHANGE_HOLD_TIME = 2.0
OBSTACLE_LEFT_IS_LANE2 = True
TRAFFIC_LIGHT_TIMEOUT = 1.0
WAIT_FOR_GREEN_ON_START = False

NORMAL_SPEED = 250
AVOID_SPEED = 120
STOP_SPEED = 0


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__("mission_manager_node")

        self.sub_detection_topic = self.declare_parameter(
            "sub_detection_topic", SUB_DETECTION_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter(
            "sub_traffic_light_topic", SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter(
            "sub_lidar_obstacle_topic", SUB_LIDAR_OBSTACLE_TOPIC_NAME).value

        self.pub_target_lane_topic = self.declare_parameter(
            "pub_target_lane_topic", PUB_TARGET_LANE_TOPIC_NAME).value
        self.pub_force_stop_topic = self.declare_parameter(
            "pub_force_stop_topic", PUB_FORCE_STOP_TOPIC_NAME).value
        self.pub_speed_limit_topic = self.declare_parameter(
            "pub_speed_limit_topic", PUB_SPEED_LIMIT_TOPIC_NAME).value
        self.pub_state_topic = self.declare_parameter(
            "pub_state_topic", PUB_STATE_TOPIC_NAME).value

        self.default_lane = self.declare_parameter(
            "default_lane", DEFAULT_LANE).value
        self.image_center_x = self.declare_parameter(
            "image_center_x", IMAGE_CENTER_X).value
        self.obstacle_near_y = self.declare_parameter(
            "obstacle_near_y", OBSTACLE_NEAR_Y).value
        self.obstacle_lost_timeout = self.declare_parameter(
            "obstacle_lost_timeout", OBSTACLE_LOST_TIMEOUT).value
        self.lane_change_hold_time = self.declare_parameter(
            "lane_change_hold_time", LANE_CHANGE_HOLD_TIME).value
        self.obstacle_left_is_lane2 = self.declare_parameter(
            "obstacle_left_is_lane2", OBSTACLE_LEFT_IS_LANE2).value
        self.traffic_light_timeout = self.declare_parameter(
            "traffic_light_timeout", TRAFFIC_LIGHT_TIMEOUT).value
        self.wait_for_green_on_start = self.declare_parameter(
            "wait_for_green_on_start", WAIT_FOR_GREEN_ON_START).value
        self.normal_speed = self.declare_parameter(
            "normal_speed", NORMAL_SPEED).value
        self.avoid_speed = self.declare_parameter(
            "avoid_speed", AVOID_SPEED).value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.detection_data = None
        self.traffic_light = "None"
        self.lidar_obstacle = False
        self.target_lane = int(self.default_lane)
        self.last_obstacle_time = None
        self.last_lane_change_time = None
        self.last_traffic_light_time = None
        self.started = not bool(self.wait_for_green_on_start)

        self.create_subscription(
            DetectionArray, self.sub_detection_topic,
            self.detection_callback, self.qos_profile)
        self.create_subscription(
            String, self.sub_traffic_light_topic,
            self.traffic_light_callback, self.qos_profile)
        self.create_subscription(
            Bool, self.sub_lidar_obstacle_topic,
            self.lidar_callback, self.qos_profile)

        self.target_lane_pub = self.create_publisher(
            Int32, self.pub_target_lane_topic, self.qos_profile)
        self.force_stop_pub = self.create_publisher(
            Bool, self.pub_force_stop_topic, self.qos_profile)
        self.speed_limit_pub = self.create_publisher(
            Int32, self.pub_speed_limit_topic, self.qos_profile)
        self.state_pub = self.create_publisher(
            String, self.pub_state_topic, self.qos_profile)

        self.timer = self.create_timer(TIMER, self.timer_callback)

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def traffic_light_callback(self, msg: String):
        self.traffic_light = msg.data
        self.last_traffic_light_time = self.get_clock().now()

    def lidar_callback(self, msg: Bool):
        self.lidar_obstacle = msg.data

    def timer_callback(self):
        now = self.get_clock().now()

        force_stop = False
        speed_limit = int(self.normal_speed)
        state = "NORMAL_DRIVE"

        traffic_light = self.get_fresh_traffic_light(now)

        if not self.started:
            if traffic_light == "Green":
                self.started = True
                state = "START_GREEN_LIGHT"
            else:
                force_stop = True
                speed_limit = STOP_SPEED
                state = "WAIT_GREEN_LIGHT"

        if traffic_light == "Red":
            force_stop = True
            speed_limit = STOP_SPEED
            state = "STOP_RED_LIGHT"
        elif self.started and traffic_light == "Yellow":
            speed_limit = min(speed_limit, int(self.avoid_speed))
            state = "SLOW_YELLOW_LIGHT"

        if self.lidar_obstacle:
            force_stop = True
            speed_limit = STOP_SPEED
            state = "EMERGENCY_LIDAR_STOP"

        obstacle_lane = self.detect_near_obstacle_lane()
        if obstacle_lane is not None:
            self.last_obstacle_time = now
            if obstacle_lane == self.target_lane:
                self.target_lane = self.opposite_lane(self.target_lane)
                self.last_lane_change_time = now
            if not force_stop:
                speed_limit = min(speed_limit, int(self.avoid_speed))
                state = f"AVOID_LANE{obstacle_lane}"
        elif self.should_return_to_default_lane(now):
            self.target_lane = int(self.default_lane)

        if self.last_lane_change_time is not None:
            elapsed = (now - self.last_lane_change_time).nanoseconds / 1e9
            if elapsed < self.lane_change_hold_time and not force_stop:
                state = f"LANE_CHANGE_TO_{self.target_lane}"
                speed_limit = min(speed_limit, int(self.avoid_speed))

        self.publish_mission(target_lane=self.target_lane,
                             force_stop=force_stop,
                             speed_limit=speed_limit,
                             state=state)

    def get_fresh_traffic_light(self, now):
        if self.last_traffic_light_time is None:
            return "None"

        elapsed = (now - self.last_traffic_light_time).nanoseconds / 1e9
        if elapsed > self.traffic_light_timeout:
            return "None"

        return self.traffic_light

    def detect_near_obstacle_lane(self):
        if self.detection_data is None:
            return None

        nearest_lane = None
        nearest_y = -1.0

        for detection in self.detection_data.detections:
            if detection.class_name != OBSTACLE_CLASS_NAME:
                continue

            bbox = detection.bbox
            center_x = bbox.center.position.x
            bottom_y = bbox.center.position.y + bbox.size.y / 2.0

            if bottom_y < self.obstacle_near_y:
                continue

            if bottom_y > nearest_y:
                nearest_y = bottom_y
                nearest_lane = self.estimate_lane_from_x(center_x)

        return nearest_lane

    def estimate_lane_from_x(self, center_x):
        is_left_side = center_x < self.image_center_x
        if bool(self.obstacle_left_is_lane2):
            return LANE2 if is_left_side else LANE1
        return LANE1 if is_left_side else LANE2

    def opposite_lane(self, lane):
        if lane == LANE1:
            return LANE2
        return LANE1

    def should_return_to_default_lane(self, now):
        if self.target_lane == int(self.default_lane):
            return False
        if self.last_obstacle_time is None:
            return False

        elapsed = (now - self.last_obstacle_time).nanoseconds / 1e9
        return elapsed > self.obstacle_lost_timeout

    def publish_mission(self, target_lane, force_stop, speed_limit, state):
        target_lane_msg = Int32()
        target_lane_msg.data = int(target_lane)
        self.target_lane_pub.publish(target_lane_msg)

        force_stop_msg = Bool()
        force_stop_msg.data = bool(force_stop)
        self.force_stop_pub.publish(force_stop_msg)

        speed_limit_msg = Int32()
        speed_limit_msg.data = int(speed_limit)
        self.speed_limit_pub.publish(speed_limit_msg)

        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)

        self.get_logger().info(
            f"state: {state}, target_lane: {target_lane}, "
            f"force_stop: {force_stop}, speed_limit: {speed_limit}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
