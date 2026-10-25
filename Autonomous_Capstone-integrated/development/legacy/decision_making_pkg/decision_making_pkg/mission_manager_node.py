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
LANE_CLASS_NAMES = {
    LANE1: "lane1",
    LANE2: "lane2",
}
OBSTACLE_NEAR_Y = 300
OBSTACLE_LANE2_MIN_X = 180.0
OBSTACLE_LANE1_MIN_X = 240.0
OBSTACLE_LANE_MATCH_MARGIN = 25.0
OBSTACLE_MAX_COUNT = 2
LANE1_SECOND_OBSTACLE_MIN_Y = 330.0
LANE2_FIRST_OBSTACLE_TOP_Y = 360.0
LANE2_FIRST_OBSTACLE_MIN_WIDTH = 100.0

AVOID_HOLD_TIME = 2.0
PASS_MIN_TIME = 0.8
PASS_MAX_TIME = 2.0
SECOND_OBSTACLE_ARM_TIME = 0.0
TRAFFIC_LIGHT_TIMEOUT = 1.0
WAIT_FOR_GREEN_ON_START = False

NORMAL_SPEED = 150
AVOID_SPEED = 90
STOP_SPEED = 0

OBSTACLE_NORMAL = "OBSTACLE_NORMAL"
OBSTACLE_AVOIDING = "OBSTACLE_AVOIDING"
OBSTACLE_PASSING = "OBSTACLE_PASSING"
OBSTACLE_DONE = "OBSTACLE_DONE"


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
        self.obstacle_lane2_min_x = self.declare_parameter(
            "obstacle_lane2_min_x", OBSTACLE_LANE2_MIN_X).value
        self.obstacle_lane_match_margin = self.declare_parameter(
            "obstacle_lane_match_margin", OBSTACLE_LANE_MATCH_MARGIN).value
        self.obstacle_max_count = self.declare_parameter(
            "obstacle_max_count", OBSTACLE_MAX_COUNT).value
        self.lane1_second_obstacle_min_y = self.declare_parameter(
            "lane1_second_obstacle_min_y", LANE1_SECOND_OBSTACLE_MIN_Y).value
        self.lane2_first_obstacle_top_y = self.declare_parameter(
            "lane2_first_obstacle_top_y", LANE2_FIRST_OBSTACLE_TOP_Y).value
        self.lane2_first_obstacle_min_width = self.declare_parameter(
            "lane2_first_obstacle_min_width", LANE2_FIRST_OBSTACLE_MIN_WIDTH).value
        self.avoid_hold_time = self.declare_parameter(
            "avoid_hold_time", AVOID_HOLD_TIME).value
        self.pass_min_time = self.declare_parameter(
            "pass_min_time", PASS_MIN_TIME).value
        self.pass_max_time = self.declare_parameter(
            "pass_max_time", PASS_MAX_TIME).value
        self.second_obstacle_arm_time = self.declare_parameter(
            "second_obstacle_arm_time", SECOND_OBSTACLE_ARM_TIME).value
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
        self.last_traffic_light_time = None
        self.started = not bool(self.wait_for_green_on_start)

        self.obstacle_state = OBSTACLE_NORMAL
        self.obstacle_count = 0
        self.obstacle_state_start_time = self.get_clock().now()
        self.current_obstacle_lane = None

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

        if not force_stop:
            obstacle_state, obstacle_speed = self.update_obstacle_state(now)
            state = obstacle_state
            speed_limit = min(speed_limit, obstacle_speed)

        self.publish_mission(target_lane=self.target_lane,
                             force_stop=force_stop,
                             speed_limit=speed_limit,
                             state=state)

    def update_obstacle_state(self, now):
        obstacle_lane = self.detect_near_obstacle_lane()
        elapsed = self.elapsed_state_time(now)

        if self.obstacle_state == OBSTACLE_DONE:
            return self.make_search_state("OBSTACLE_DONE_DRIVE"), int(self.normal_speed)

        if self.obstacle_state == OBSTACLE_NORMAL:
            if self.is_waiting_to_arm_second_obstacle(elapsed):
                return self.make_search_state("ARMING_NEXT_OBSTACLE"), int(self.normal_speed)
            if self.obstacle_count == 0 and self.has_first_lane2_obstacle():
                self.start_avoidance(now, LANE2)
                return self.make_state("AVOIDING_TO"), int(self.avoid_speed)
            if self.should_start_avoidance(obstacle_lane):
                self.start_avoidance(now, obstacle_lane)
                return self.make_state("AVOIDING_TO"), int(self.avoid_speed)
            return self.make_search_state("SEARCH_OBSTACLE"), int(self.normal_speed)

        if self.obstacle_state == OBSTACLE_AVOIDING:
            if elapsed >= float(self.avoid_hold_time):
                self.transition_obstacle_state(now, OBSTACLE_PASSING)
                return self.make_state("PASSING_ON"), int(self.avoid_speed)
            return self.make_state("AVOIDING_TO"), int(self.avoid_speed)

        if self.obstacle_state == OBSTACLE_PASSING:
            if self.obstacle_count == 1 and self.should_start_avoidance(obstacle_lane):
                self.start_avoidance(now, obstacle_lane)
                return self.make_state("AVOIDING_TO"), int(self.avoid_speed)

            if self.obstacle_count == 1 and self.has_near_obstacle_in_lane(LANE2):
                self.target_lane = LANE1
                return self.make_state("PASSING_ON"), int(self.avoid_speed)

            pass_time_done = elapsed >= float(self.pass_min_time)
            obstacle_clear = obstacle_lane is None or obstacle_lane != self.current_obstacle_lane
            pass_timeout = elapsed >= float(self.pass_max_time)
            if pass_time_done and (obstacle_clear or pass_timeout):
                if self.obstacle_count >= int(self.obstacle_max_count):
                    self.transition_obstacle_state(now, OBSTACLE_DONE)
                    return self.make_search_state("OBSTACLE_DONE_DRIVE"), int(self.normal_speed)
                self.transition_obstacle_state(now, OBSTACLE_NORMAL)
                return self.make_search_state("SEARCH_NEXT_OBSTACLE"), int(self.normal_speed)
            return self.make_state("PASSING_ON"), int(self.avoid_speed)

        self.transition_obstacle_state(now, OBSTACLE_NORMAL)
        self.target_lane = int(self.default_lane)
        return self.make_search_state("SEARCH_OBSTACLE"), int(self.normal_speed)

    def should_start_avoidance(self, obstacle_lane):
        if obstacle_lane is None:
            return False
        if self.obstacle_count >= int(self.obstacle_max_count):
            return False
        if self.is_first_lane2_obstacle(obstacle_lane):
            return self.has_first_lane2_obstacle()
        if self.is_second_lane1_obstacle(obstacle_lane):
            return self.is_lane1_ready_for_second_avoidance()
        if obstacle_lane != self.target_lane:
            return False
        return True

    def is_first_lane2_obstacle(self, obstacle_lane):
        return self.obstacle_count == 0 and obstacle_lane == LANE2

    def is_second_lane1_obstacle(self, obstacle_lane):
        return self.obstacle_count == 1 and obstacle_lane == LANE1

    def has_first_lane2_obstacle(self):
        return self.has_obstacle_in_lane_top_y_at_or_below_and_wide(
            LANE2,
            float(self.lane2_first_obstacle_top_y),
            float(self.lane2_first_obstacle_min_width))

    def is_lane1_ready_for_second_avoidance(self):
        return self.has_obstacle_in_lane_below_y(
            LANE1, float(self.lane1_second_obstacle_min_y))

    def has_obstacle_in_lane_below_y(self, lane, min_bottom_y):
        if self.detection_data is None:
            return False

        for detection in self.detection_data.detections:
            if detection.class_name != OBSTACLE_CLASS_NAME:
                continue

            bbox = detection.bbox
            bottom_y = bbox.center.position.y + bbox.size.y / 2.0
            if bottom_y < min_bottom_y:
                continue

            if self.estimate_obstacle_lane(detection) == lane:
                return True
        return False

    def has_obstacle_in_lane_top_y_at_or_below_and_wide(
            self, lane, max_top_y, min_width):
        if self.detection_data is None:
            return False

        for detection in self.detection_data.detections:
            if detection.class_name != OBSTACLE_CLASS_NAME:
                continue

            bbox = detection.bbox
            top_y = bbox.center.position.y - bbox.size.y / 2.0
            if top_y > max_top_y or bbox.size.x <= min_width:
                continue

            if self.estimate_obstacle_lane(detection) == lane:
                return True
        return False

    def is_waiting_to_arm_second_obstacle(self, elapsed):
        if self.obstacle_count != 1:
            return False
        return elapsed < float(self.second_obstacle_arm_time)

    def start_avoidance(self, now, obstacle_lane):
        self.current_obstacle_lane = obstacle_lane
        self.obstacle_count += 1
        self.target_lane = self.opposite_lane(obstacle_lane)
        self.transition_obstacle_state(now, OBSTACLE_AVOIDING)

    def transition_obstacle_state(self, now, next_state):
        if self.obstacle_state == next_state:
            return
        self.obstacle_state = next_state
        self.obstacle_state_start_time = now

    def elapsed_state_time(self, now):
        return (now - self.obstacle_state_start_time).nanoseconds / 1e9

    def make_state(self, prefix):
        return (
            f"{prefix}_LANE{self.target_lane}_"
            f"OBS{self.obstacle_count}"
        )

    def make_search_state(self, prefix):
        return (
            f"{prefix}_LANE{self.target_lane}_"
            f"OBS{self.obstacle_count}"
        )

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
        target_lane_obstacle_y = -1.0
        target_lane_obstacle = None

        for detection in self.detection_data.detections:
            if detection.class_name != OBSTACLE_CLASS_NAME:
                continue

            bbox = detection.bbox
            center_x = bbox.center.position.x
            bottom_y = bbox.center.position.y + bbox.size.y / 2.0

            if bottom_y < self.obstacle_near_y:
                continue

            obstacle_lane = self.estimate_obstacle_lane(detection)
            if obstacle_lane == self.target_lane and bottom_y > target_lane_obstacle_y:
                target_lane_obstacle_y = bottom_y
                target_lane_obstacle = obstacle_lane

            if bottom_y > nearest_y:
                nearest_y = bottom_y
                nearest_lane = obstacle_lane

        if target_lane_obstacle is not None:
            return target_lane_obstacle
        return nearest_lane

    def has_near_obstacle_in_lane(self, lane):
        if self.detection_data is None:
            return False

        for detection in self.detection_data.detections:
            if detection.class_name != OBSTACLE_CLASS_NAME:
                continue

            bbox = detection.bbox
            bottom_y = bbox.center.position.y + bbox.size.y / 2.0
            if bottom_y < self.obstacle_near_y:
                continue

            if self.estimate_obstacle_lane(detection) == lane:
                return True
        return False

    def estimate_obstacle_lane(self, obstacle_detection):
        bbox = obstacle_detection.bbox
        point_x = bbox.center.position.x
        point_y = bbox.center.position.y + bbox.size.y / 2.0

        lane_from_mask = self.estimate_lane_from_masks(point_x, point_y)
        if lane_from_mask is not None:
            return lane_from_mask

        return self.estimate_lane_from_x(point_x)

    def estimate_lane_from_masks(self, point_x, point_y):
        if self.detection_data is None:
            return None

        closest_lane = None
        closest_distance = None
        margin = float(self.obstacle_lane_match_margin)

        for detection in self.detection_data.detections:
            lane = self.lane_from_class_name(detection.class_name)
            if lane is None or len(detection.mask.data) < 3:
                continue

            points = [(point.x, point.y) for point in detection.mask.data]
            if self.point_in_polygon(point_x, point_y, points):
                return lane

            lane_range = self.polygon_x_range_at_y(points, point_y, margin)
            if lane_range is None:
                continue

            min_x, max_x = lane_range
            if min_x - margin <= point_x <= max_x + margin:
                lane_center_x = (min_x + max_x) / 2.0
                distance = abs(point_x - lane_center_x)
                if closest_distance is None or distance < closest_distance:
                    closest_distance = distance
                    closest_lane = lane

        return closest_lane

    def lane_from_class_name(self, class_name):
        for lane, lane_class_name in LANE_CLASS_NAMES.items():
            if class_name == lane_class_name:
                return lane
        return None

    def polygon_x_range_at_y(self, points, y, margin):
        intersections = []
        point_count = len(points)

        for i in range(point_count):
            x1, y1 = points[i]
            x2, y2 = points[(i + 1) % point_count]
            if abs(y1 - y2) < 1e-6:
                continue
            if y < min(y1, y2) - margin or y > max(y1, y2) + margin:
                continue

            clamped_y = max(min(y, max(y1, y2)), min(y1, y2))
            ratio = (clamped_y - y1) / (y2 - y1)
            intersections.append(x1 + ratio * (x2 - x1))

        if len(intersections) < 2:
            return None

        return min(intersections), max(intersections)

    def point_in_polygon(self, x, y, points):
        inside = False
        j = len(points) - 1

        for i in range(len(points)):
            xi, yi = points[i]
            xj, yj = points[j]
            crosses_y = (yi > y) != (yj > y)
            if crosses_y:
                x_intersection = (xj - xi) * (y - yi) / (yj - yi) + xi
                if x < x_intersection:
                    inside = not inside
            j = i

        return inside

    def estimate_lane_from_x(self, center_x):
        if center_x >= self.obstacle_lane2_min_x:
            return LANE2
        return LANE1

    def opposite_lane(self, lane):
        if lane == LANE1:
            return LANE2
        return LANE1

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
            f"obstacle_state: {self.obstacle_state}, "
            f"obstacle_count: {self.obstacle_count}, "
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
