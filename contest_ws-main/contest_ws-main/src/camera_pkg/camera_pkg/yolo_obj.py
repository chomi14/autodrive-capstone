import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from interfaces_pkg.msg import Detection, DetectionArray, Mask, Point2D

import os
import torch


class YoloSegNode(LifecycleNode):
    def __init__(self):
        super().__init__('yolo_seg')

        # 현재 스크립트 파일의 디렉토리를 기준으로 상대경로 설정
        default_model_path = os.path.join(os.path.dirname(__file__), 'model', 'yolo11n-seg.pt')
        self.declare_parameter("model_path", default_model_path)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.3)

        self.bridge = CvBridge()
        self.model = None
        self.device = None

        self.get_logger().info("🟢 YoloSegNode initialized")

    def on_configure(self, state):
        self.get_logger().info("⚙️  Configuring YoloSegNode")

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found: {model_path}")
            return TransitionCallbackReturn.FAILURE

        try:
            self.model = YOLO(model_path)
            self.model.fuse()
            self.get_logger().info(f"✅ Model loaded from {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            return TransitionCallbackReturn.FAILURE

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self._pub = self.create_lifecycle_publisher(DetectionArray, "detections", qos)
        self.image_qos_profile = qos

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("🚀 Activating YoloSegNode")

        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_cb,
            self.image_qos_profile
        )

        return TransitionCallbackReturn.SUCCESS

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        try:
            results = self.model.predict(
                source=frame,
                device=self.device,
                conf=self.threshold,
                stream=False,
                verbose=False
            )[0].cpu()

            detections = DetectionArray()
            detections.header = msg.header

            if results.boxes:
                for i, box in enumerate(results.boxes):
                    det = Detection()
                    det.class_id = int(box.cls)
                    det.class_name = self.model.names[det.class_id]
                    det.score = float(box.conf)

                    # box.xywh: [center_x, center_y, width, height]
                    b = box.xywh[0]
                    det.bbox.center.position.x = float(b[0])
                    det.bbox.center.position.y = float(b[1])
                    det.bbox.size.x = float(b[2])
                    det.bbox.size.y = float(b[3])

                    if results.masks:
                        mask_xy = results.masks.xy[i]
                        mask_msg = Mask()
                        mask_msg.data = [Point2D(x=float(p[0]), y=float(p[1])) for p in mask_xy]
                        mask_msg.height = frame.shape[0]
                        mask_msg.width = frame.shape[1]
                        det.mask = mask_msg

                    detections.detections.append(det)

            self._pub.publish(detections)

        except Exception as e:
            self.get_logger().error(f"Segmentation failed: {e}")

    def on_cleanup(self, state):
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegNode()
    if node.trigger_configure() == TransitionCallbackReturn.SUCCESS:
        if node.trigger_activate() == TransitionCallbackReturn.SUCCESS:
            rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


