import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool, Int32
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
SUB_FORCE_STOP_TOPIC_NAME = "mission_force_stop"
SUB_SPEED_LIMIT_TOPIC_NAME = "mission_speed_limit"
PUB_TOPIC_NAME = "topic_control_signal"

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

# 추가 CONSTANTS 선언
MAX_STEP = 7
THETA_MAX_DEG = 75.0
ALPHA = 0.3
MAX_STEP_DELTA = 1
MAX_SPEED = 250
MIN_SPEED = 250
DEFAULT_SPEED_LIMIT = 250
STEERING_BIAS = -2


class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        # self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        # self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.sub_force_stop_topic = self.declare_parameter('sub_force_stop_topic', SUB_FORCE_STOP_TOPIC_NAME).value
        self.sub_speed_limit_topic = self.declare_parameter('sub_speed_limit_topic', SUB_SPEED_LIMIT_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.steering_bias = self.declare_parameter('steering_bias', STEERING_BIAS).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        # self.traffic_light_data = None
        # self.lidar_data = None
        self.force_stop = False
        self.speed_limit = DEFAULT_SPEED_LIMIT

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0

        # 추가 변수 설정 (11조)
        self.target_slope_f = 0
        self.prev_step = 0
        self.cnt_dead = 0

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        # self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        # self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)
        self.force_stop_sub = self.create_subscription(Bool, self.sub_force_stop_topic, self.force_stop_callback, self.qos_profile)
        self.speed_limit_sub = self.create_subscription(Int32, self.sub_speed_limit_topic, self.speed_limit_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))

    def force_stop_callback(self, msg: Bool):
        self.force_stop = msg.data

    def speed_limit_callback(self, msg: Int32):
        self.speed_limit = msg.data
                
    # def traffic_light_callback(self, msg: String):
    #     self.traffic_light_data = msg

    # def lidar_callback(self, msg: Bool):
        # self.lidar_data = msg
        
    def timer_callback(self):
        # 1. 경로 데이터가 아예 없는 경우 (안전을 위해 정지)
        if self.path_data is None:
            self.steering_command = 0
            self.left_speed_command = 0
            self.right_speed_command = 0
            self.get_logger().warn("---------Path data none!!!---------")
    
        # 2. 경로 데이터가 부족한 경우 (데드 레코닝 또는 정지)
        elif len(self.path_data) < 10:
            self.cnt_dead += 1
            if self.cnt_dead > 30:
                self.get_logger().info("Dead reckoning mode")
                self.steering_command = 0
                self.left_speed_command = 100
                self.right_speed_command = 100
    
    # 3. 정상 경로 추종 (11조 알고리즘 작동)
        else:
            self.cnt_dead = 0
            target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
        
            # [11조 제어 로직 적용]
            # Dead Zone 설정
            if abs(target_slope) < 1.0:
                    target_slope = 0.0

                # LPF (Low Pass Filter)
            self.target_slope_f = (1 - ALPHA) * self.target_slope_f + ALPHA * target_slope
            
            # 단위 변환 및 스텝 제한
            step_f = (self.target_slope_f / THETA_MAX_DEG) * MAX_STEP
            step = int(round(step_f))
            step = max(-MAX_STEP, min(MAX_STEP, step))
            
            # 변화율 제한 (Slew Rate Limiter)
            step = max(self.prev_step - MAX_STEP_DELTA, min(self.prev_step + MAX_STEP_DELTA, step))
            self.prev_step = step
            step = max(-MAX_STEP, min(MAX_STEP, step + int(self.steering_bias)))
            self.steering_command = step

                # 속도 결정 (기본 주행)
            tmp_speed = MAX_SPEED - abs(self.target_slope_f) / THETA_MAX_DEG * (MAX_SPEED - MIN_SPEED)
            self.left_speed_command = int(max(MIN_SPEED, tmp_speed))
            self.right_speed_command = int(max(MIN_SPEED, tmp_speed))

        if self.force_stop:
            self.steering_command = 0
            self.left_speed_command = 0
            self.right_speed_command = 0
        else:
            self.left_speed_command = min(self.left_speed_command, self.speed_limit)
            self.right_speed_command = min(self.right_speed_command, self.speed_limit)





 


        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"left_speed: {self.left_speed_command}, " 
                               f"right_speed: {self.right_speed_command}")

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
