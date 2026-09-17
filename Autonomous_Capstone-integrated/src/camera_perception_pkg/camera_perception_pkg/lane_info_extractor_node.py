import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from interfaces_pkg.msg import TargetPoint, LaneInfo, DetectionArray, BoundingBox2D, Detection
from .lib import camera_perception_func_lib as CPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "detections"
SUB_TARGET_LANE_TOPIC_NAME = "mission_target_lane"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_lane_info"
ROI_IMAGE_TOPIC_NAME = "roi_image"  # 추가: ROI 이미지 퍼블리시 토픽

# 화면에 이미지를 처리하는 과정을 띄울것인지 여부: True, 또는 False 중 택1하여 입력
SHOW_IMAGE = True

CENTER_X = 270
DEFAULT_TARGET_LANE = 2
#----------------------------------------------


class Yolov8InfoExtractor(Node):
    def __init__(self):
        super().__init__('lane_info_extractor_node')

        self.sub_topic = self.declare_parameter('sub_detection_topic', SUB_TOPIC_NAME).value
        self.sub_target_lane_topic = self.declare_parameter('sub_target_lane_topic', SUB_TARGET_LANE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.show_image = self.declare_parameter('show_image', SHOW_IMAGE).value
        self.target_lane = self.declare_parameter('target_lane', DEFAULT_TARGET_LANE).value

        self.cv_bridge = CvBridge()

        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.subscriber = self.create_subscription(DetectionArray, self.sub_topic, self.yolov8_detections_callback, self.qos_profile)
        self.target_lane_subscriber = self.create_subscription(Int32, self.sub_target_lane_topic, self.target_lane_callback, self.qos_profile)
        self.publisher = self.create_publisher(LaneInfo, self.pub_topic, self.qos_profile)

        # ROI 이미지 퍼블리셔 추가
        self.roi_image_publisher = self.create_publisher(Image, ROI_IMAGE_TOPIC_NAME, self.qos_profile)

    def target_lane_callback(self, msg: Int32):
        if msg.data in (1, 2):
            self.target_lane = msg.data

    def yolov8_detections_callback(self, detection_msg: DetectionArray):
        if len(detection_msg.detections) == 0:
            return
        
        lane_class_name = f'lane{self.target_lane}'
        lane_edge_image = CPFL.draw_edges(detection_msg, cls_name=lane_class_name, color=255)
 

        (h, w) = (lane_edge_image.shape[0], lane_edge_image.shape[1]) #(480, 640)
        dst_mat = [[round(w * 0.3), round(h * 0.0)], [round(w * 0.7), round(h * 0.0)], [round(w * 0.7), h], [round(w * 0.3), h]]
        src_mat = [[238, 316],[402, 313], [501, 476], [155, 476]]
               # 1. 이미지가 None이 아니고, 크기가 0보다 큰지 확인
        if lane_edge_image is not None and lane_edge_image.size > 0:
            try:
                lane_bird_image = CPFL.bird_convert(lane_edge_image, srcmat=src_mat, dstmat=dst_mat)
                
                # (이후 lane_bird_image를 사용하는 코드들도 이 if문 안으로 넣거나,
                #  else 문에서 예외 처리를 해주어야 합니다.)
                
            except cv2.error as e:
                self.get_logger().warn(f"OpenCV Error ignored: {e}")
                return
        else:
            # 이미지가 비었을 경우 로그 출력 (디버깅용)
            self.get_logger().warn(f"Skipping frame: '{lane_class_name}' edge image is empty or invalid.")
            return
        # lane_bird_image = CPFL.bird_convert(lane_edge_image, srcmat=src_mat, dstmat=dst_mat)
        roi_image = CPFL.roi_rectangle_below(lane_bird_image, cutting_idx=300)

        if self.show_image:
            cv2.imshow('lane_edge_image', lane_edge_image)
            cv2.imshow('lane_bird_img', lane_bird_image)
            cv2.imshow('roi_img', roi_image)
            cv2.waitKey(1)

        # roi_image를 uint8 형식으로 변환
        roi_image = cv2.convertScaleAbs(roi_image)  # 64FC1 -> uint8로 변환

        # roi_image를 ROS Image 메시지로 변환
        try:
            roi_image_msg = self.cv_bridge.cv2_to_imgmsg(roi_image, encoding="mono8")
            # ROI 이미지를 퍼블리시
            self.roi_image_publisher.publish(roi_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert and publish ROI image: {e}")
        
        grad = CPFL.dominant_gradient(roi_image, theta_limit=70)
                
        target_points = []
        for target_point_y in range(5, 155, 50):  # 예시로 5에서 155까지 50씩 증가
            target_point_x = CPFL.get_lane_center(roi_image, detection_height=target_point_y, 
                                                detection_thickness=10, road_gradient=grad, lane_width=300)
            
            target_point = TargetPoint()
            target_point.target_x = round(target_point_x)
            target_point.target_y = round(target_point_y)
            target_points.append(target_point)

        lane = LaneInfo()
        lane.slope = grad
        lane.target_points = target_points

        self.publisher.publish(lane)


def main(args=None):
    rclpy.init(args=args)
    node = Yolov8InfoExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
  
if __name__ == '__main__':
    main()
