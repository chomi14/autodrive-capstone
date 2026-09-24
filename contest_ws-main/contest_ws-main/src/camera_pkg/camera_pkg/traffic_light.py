import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')

        self.bridge = CvBridge()
        self.last_status = None
        self.declare_parameter('publish_mask', False)
        self.publish_mask = bool(self.get_parameter('publish_mask').value)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        result_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 원본 영상 & DetectionArray 동기화 (기존 퍼블리셔 유지)
        self.image_sub     = Subscriber(self, Image,          '/cam1/image_raw',   qos_profile=sensor_qos)
        self.detection_sub = Subscriber(self, DetectionArray, '/cam1/detections', qos_profile=sensor_qos)
        self.ts            = ApproximateTimeSynchronizer(
            [self.image_sub, self.detection_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        # 결과 문자열 퍼블리셔
        self.pub      = self.create_publisher(String, 'traffic_light_result', result_qos)
        # → 마스킹 영상 퍼블리셔 추가
        self.mask_pub = self.create_publisher(Image,  '/cam1/traffic_light_mask', sensor_qos)

        # 색상 검출용 HSV 범위
        self.hsv_ranges = {
            'red1':   (np.array([0,   100, 95]), np.array([10,  255,255])),
            'red2':   (np.array([160, 100, 95]), np.array([179, 255,255])),
            'yellow': (np.array([20,  100, 95]), np.array([30,  255,255])),
            'green':  (np.array([40,  100, 95]), np.array([90,  255,255]))
        }

    def sync_callback(self, img_msg, det_msg):
        # 1) 원본 프레임 변환
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # 2) 색 검출 & 문자열 퍼블리시
        detected, area, color = self.detect_traffic_light_color(cv_img, det_msg)
        status_msg = f"Detected: {detected}, Area: {area}, Color: {color}"
        out = String()
        out.data = status_msg
        self.pub.publish(out)
        self.last_status = status_msg

        if not self.publish_mask:
            return

        # 3) 마스킹 영상 생성
        overlay = cv_img.copy()
        for det in det_msg.detections:
            if det.class_name != 'traffic_light':
                continue

            # bbox 픽셀 좌표
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)
            w  = int(det.bbox.size.x)
            h  = int(det.bbox.size.y)
            x1 = max(cx - w//2, 0)
            y1 = max(cy - h//2, 0)
            x2 = min(cx + w//2, cv_img.shape[1])
            y2 = min(cy + h//2, cv_img.shape[0])

            # 컬러 매핑
            if   color == 'red':    bgr = (0, 0, 255)
            elif color == 'yellow': bgr = (0, 255, 255)
            elif color == 'green':  bgr = (0, 255, 0)
            else:                   bgr = (255, 255, 255)  # unknown

            # 반투명 박스
            cv2.rectangle(overlay, (x1, y1), (x2, y2), bgr, -1)

        # 4) 합성
        alpha = 0.5
        cv2.addWeighted(overlay, alpha, cv_img, 1 - alpha, 0, cv_img)

        # 5) 퍼블리시
        mask_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        mask_msg.header = img_msg.header
        self.mask_pub.publish(mask_msg)

    def detect_traffic_light_color(self, image, detections: DetectionArray):
        for det in detections.detections:
            if det.class_name == 'traffic_light':
                # ROI 설정
                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)
                w  = int(det.bbox.size.x)
                h  = int(det.bbox.size.y)

                x1 = max(cx - w//2, 0)
                y1 = max(cy - h//2, 0)
                x2 = min(cx + w//2, image.shape[1])
                y2 = min(cy + h//2, image.shape[0])

                roi = image[y1:y2, x1:x2]
                if roi.size == 0:
                    continue

                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                # 마스크 생성
                red_mask    = cv2.inRange(hsv, *self.hsv_ranges['red1']) + cv2.inRange(hsv, *self.hsv_ranges['red2'])
                yellow_mask = cv2.inRange(hsv, *self.hsv_ranges['yellow'])
                green_mask  = cv2.inRange(hsv, *self.hsv_ranges['green'])

                # 면적 계산
                areas = {
                    'red':    int(np.sum(red_mask    > 0)),
                    'yellow': int(np.sum(yellow_mask > 0)),
                    'green':  int(np.sum(green_mask  > 0))
                }
                # 최대 면적 색상 선택
                max_color = max(areas, key=areas.get)
                if areas[max_color] == 0:
                    max_color = 'unknown'

                return True, w*h, max_color

        return False, 0, 'none'


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
