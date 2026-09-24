import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import os
import sys
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

# ===== 사용자 설정 =====
DATA_SOURCE = 'camera'  # camera, image, video 중 하나
CAM_NUM = 0
CAMERA_DEVICE = ''
IMAGE_DIRECTORY_PATH = '/home/sg/contest_ws/src/camera_pkg/camera_pkg/lib/lane2'
VIDEO_FILE_PATH = '/absolute/path/to/your/video.mp4'
SHOW_IMAGE = True
TIMER = 0.03
PUB_TOPIC_NAME = 'image_raw'
# =====================

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image')  # node 이름

        # Declare parameters
        self.declare_parameter('data_source', DATA_SOURCE)
        self.declare_parameter('cam_num', CAM_NUM)
        # A stable /dev/v4l/by-id path can be supplied here.  Keep cam_num for
        # compatibility with the original contest launch files.
        self.declare_parameter('camera_device', CAMERA_DEVICE)
        self.declare_parameter('img_dir', IMAGE_DIRECTORY_PATH)
        self.declare_parameter('video_path', VIDEO_FILE_PATH)
        self.declare_parameter('show_image', SHOW_IMAGE)
        self.declare_parameter('timer_period', TIMER)
        self.declare_parameter('pub_topic', PUB_TOPIC_NAME)
        self.declare_parameter('window_name', 'Published Image')  # 창 이름을 파라미터로 설정

        # Get parameters
        self.data_source = self.get_parameter('data_source').get_parameter_value().string_value
        self.cam_num = self.get_parameter('cam_num').get_parameter_value().integer_value
        self.camera_device = self.get_parameter('camera_device').value
        self.img_dir = self.get_parameter('img_dir').get_parameter_value().string_value
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value  # 파라미터로 받은 창 이름

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, self.pub_topic,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1
            )
        )

        # 데이터 소스에 따라 처리
        if self.data_source == 'camera':
            source = self.camera_device if self.camera_device else self.cam_num
            self.cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.get_logger().error(f'카메라 {source}를 열 수 없습니다.')
                rclpy.shutdown()
                sys.exit(1)
            # Do not let the V4L2 backend accumulate stale frames while YOLO is
            # busy.  Some drivers ignore this property, which is harmless.
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        elif self.data_source == 'video':
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                self.get_logger().error(f'비디오 파일을 열 수 없습니다: {self.video_path}')
                rclpy.shutdown()
                sys.exit(1)

        elif self.data_source == 'image':
            if not os.path.isdir(self.img_dir):
                self.get_logger().error(f'이미지 디렉토리를 찾을 수 없습니다: {self.img_dir}')
                rclpy.shutdown()
                sys.exit(1)
            self.img_list = sorted(os.listdir(self.img_dir))
            self.img_idx = 0
        else:
            self.get_logger().error(f'지원하지 않는 data_source입니다: {self.data_source}')
            rclpy.shutdown()
            sys.exit(1)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.data_source in ['camera', 'video']:
            ret, frame = self.cap.read()
            if not ret:
                if self.data_source == 'video':
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    return
                self.get_logger().warn('프레임을 읽을 수 없습니다.')
                return

            frame = cv2.resize(frame, (640, 480))
            self.publish_image(frame)

        elif self.data_source == 'image':
            if self.img_idx < len(self.img_list):
                img_path = os.path.join(self.img_dir, self.img_list[self.img_idx])
                img = cv2.imread(img_path)
                if img is None:
                    self.get_logger().warn(f'이미지 파일이 아닙니다: {self.img_list[self.img_idx]}')
                else:
                    img = cv2.resize(img, (640, 480))
                    self.publish_image(img)
                self.img_idx += 1
            else:
                self.img_idx = 0  # 다시 처음부터

    def publish_image(self, frame):
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'image_frame'
        self.publisher.publish(img_msg)

        if self.show_image:
            # window_name을 파라미터로 받아서 사용
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
