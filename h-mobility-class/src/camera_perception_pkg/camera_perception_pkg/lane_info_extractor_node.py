import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from interfaces_pkg.msg import TargetPoint, LaneInfo, DetectionArray, BoundingBox2D, Detection
from .lib import camera_perception_func_lib as CPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "detections"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_lane_info"
ROI_IMAGE_TOPIC_NAME = "roi_image"  # 추가: ROI 이미지 퍼블리시 토픽

# 화면에 이미지를 처리하는 과정을 띄울것인지 여부: True, 또는 False 중 택1하여 입력
SHOW_IMAGE = True
#----------------------------------------------


class Yolov8InfoExtractor(Node):
    def __init__(self):
        super().__init__('lane_info_extractor_node')

        self.sub_topic = self.declare_parameter('sub_detection_topic', SUB_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.show_image = self.declare_parameter('show_image', SHOW_IMAGE).value
        self.show_bev_image = self.declare_parameter('show_bev_image', False).value
        self.roi_image_topic = self.declare_parameter('roi_image_topic', ROI_IMAGE_TOPIC_NAME).value
        self.lane_class = self.declare_parameter('lane_class', 'lane2').value
        self.bev_dst_left_ratio = self.declare_parameter('bev_dst_left_ratio', 0.3).value
        self.bev_dst_right_ratio = self.declare_parameter('bev_dst_right_ratio', 0.7).value
        self.bev_src_points = self.declare_parameter('bev_src_points', [238, 316, 402, 313, 501, 476, 155, 476]).value
        self.roi_cutting_idx = self.declare_parameter('roi_cutting_idx', 300).value
        self.theta_limit = self.declare_parameter('theta_limit', 70).value
        self.target_y_start = self.declare_parameter('target_y_start', 5).value
        self.target_y_stop = self.declare_parameter('target_y_stop', 155).value
        self.target_y_step = self.declare_parameter('target_y_step', 50).value
        self.detection_thickness = self.declare_parameter('detection_thickness', 10).value
        self.lane_width = self.declare_parameter('lane_width', 300).value

        self.cv_bridge = CvBridge()

        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.subscriber = self.create_subscription(DetectionArray, self.sub_topic, self.yolov8_detections_callback, self.qos_profile)
        self.publisher = self.create_publisher(LaneInfo, self.pub_topic, self.qos_profile)

        # ROI 이미지 퍼블리셔 추가
        self.roi_image_publisher = self.create_publisher(Image, self.roi_image_topic, self.qos_profile)

    def yolov8_detections_callback(self, detection_msg: DetectionArray):
        if len(detection_msg.detections) == 0:
            return
        
        lane2_edge_image = CPFL.draw_edges(detection_msg, cls_name=self.lane_class, color=255)   # 도로 좌우 경계를 흰색으로 표현

        (h, w) = (lane2_edge_image.shape[0], lane2_edge_image.shape[1]) #(480, 640)
        dst_mat = [[round(w * self.bev_dst_left_ratio), round(h * 0.0)], [round(w * self.bev_dst_right_ratio), round(h * 0.0)], [round(w * self.bev_dst_right_ratio), h], [round(w * self.bev_dst_left_ratio), h]]
        src_mat = [self.bev_src_points[i:i + 2] for i in range(0, 8, 2)]
        
        lane2_bird_image = CPFL.bird_convert(lane2_edge_image, srcmat=src_mat, dstmat=dst_mat)
        roi_image = CPFL.roi_rectangle_below(lane2_bird_image, cutting_idx=self.roi_cutting_idx)

        if self.show_image:
            cv2.imshow('lane2_edge_image', lane2_edge_image)
            cv2.imshow('roi_img', roi_image)

        if self.show_bev_image:
            cv2.imshow('lane2_bird_img', lane2_bird_image)

        if self.show_image or self.show_bev_image:
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
        
        grad = CPFL.dominant_gradient(roi_image, theta_limit=self.theta_limit)
                
        target_points = []
        for target_point_y in range(self.target_y_start, self.target_y_stop, self.target_y_step):  # 예시로 5에서 155까지 50씩 증가
            target_point_x = CPFL.get_lane_center(roi_image, detection_height=target_point_y, 
                                                detection_thickness=self.detection_thickness, road_gradient=grad, lane_width=self.lane_width)
            
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
