#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisherNode(Node):
    """Ubuntu/V4L2 camera -> sensor_msgs/Image publisher."""

    def __init__(self):
        super().__init__('camera_publisher_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic', 'image_raw')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('show', False)

        self.device = self.get_parameter('device').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.show = bool(self.get_parameter('show').value)

        # /dev/videoN or /dev/v4l/by-id/... can both be passed as a string.
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().warn(f'V4L2 open failed for {self.device}; retrying with default backend')
            self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open camera: {self.device}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Image, self.topic, qos)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / max(self.fps, 1.0), self.on_timer)
        self.frame_count = 0
        self.get_logger().info(
            f'Camera opened: device={self.device}, requested={self.width}x{self.height}@{self.fps:.1f}, topic=/{self.topic}'
        )

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Camera frame read failed')
            return

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)
        self.frame_count += 1

        if self.show:
            cv2.imshow('ROS2 Camera', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'[camera_publisher_node] ERROR: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
