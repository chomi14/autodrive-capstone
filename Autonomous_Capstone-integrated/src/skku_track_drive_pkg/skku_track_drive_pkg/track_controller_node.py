#!/usr/bin/env python3
import time
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from interfaces_pkg.msg import MotionCommand as RosMotionCommand

from .yolo_perception import YoloDetector
from .lane_processing import LaneInfoExtractor
from .path_planner import PathPlanner
from .motion_planner import MotionPlanner


class TrackControllerNode(Node):
    """ROS2 wrapper for the track-driving pipeline from 2026_skku_autodrive-final.

    Input:  /image_raw (sensor_msgs/Image)
    Output: /topic_control_signal (interfaces_pkg/MotionCommand)
    Safety: starts disabled by default. Enable with the SetBool service.
    """

    def __init__(self):
        super().__init__('track_controller_node')

        default_model = str(Path(get_package_share_directory('skku_track_drive_pkg')) / 'models' / 'best.pt')

        # ROS I/O
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('cmd_topic', 'topic_control_signal')
        self.declare_parameter('debug_topic', 'track_debug_image')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('start_enabled', False)

        # YOLO
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence', 0.5)

        # Vehicle/control. Current Arduino firmware uses -7..+7 and negative=left.
        self.declare_parameter('max_steering', 7.0)
        self.declare_parameter('speed', 100)
        self.declare_parameter('steering_sign', 1.0)
        self.declare_parameter('max_steering_angle_deg', 50.0)
        self.declare_parameter('stanley_gain', 0.02)
        self.declare_parameter('stanley_softening', 0.001)
        self.declare_parameter('lookahead_steps', 10)
        self.declare_parameter('heading_step', 3)
        self.declare_parameter('heading_gain', 0.6)
        self.declare_parameter('car_center_x', 320.0)
        self.declare_parameter('car_center_y', 179.0)

        # Lane/BEV values copied from the summer track-driving config.
        self.declare_parameter('bev_top_shift', -8)
        self.declare_parameter('roi_cut', 300)
        self.declare_parameter('look_shift', 50)
        self.declare_parameter('slope_ema_alpha', 0.3)
        self.declare_parameter('virtual_lane_width', 300)
        self.declare_parameter('bev_pad', 250)

        image_topic = self.get_parameter('image_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.enabled = bool(self.get_parameter('start_enabled').value)

        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        conf = float(self.get_parameter('confidence').value)

        self.speed = int(self.get_parameter('speed').value)
        self.bridge = CvBridge()
        self.last_t = None
        self.frame_counter = 0

        self.get_logger().info(f'Loading summer track model: {model_path}')
        self.yolo = YoloDetector(model_path, device=device, conf=conf)
        self.lane = LaneInfoExtractor(
            show_image=False,
            bev_top_shift=int(self.get_parameter('bev_top_shift').value),
            roi_cut=int(self.get_parameter('roi_cut').value),
            look_shift=int(self.get_parameter('look_shift').value),
            slope_ema_alpha=float(self.get_parameter('slope_ema_alpha').value),
            virtual_lane_width=int(self.get_parameter('virtual_lane_width').value),
            bev_pad=int(self.get_parameter('bev_pad').value),
        )
        self.path_planner = PathPlanner(
            car_center_point=(
                int(self.get_parameter('car_center_x').value),
                int(self.get_parameter('car_center_y').value),
            )
        )
        self.motion = MotionPlanner(
            stanley_gain=float(self.get_parameter('stanley_gain').value),
            stanley_softening=float(self.get_parameter('stanley_softening').value),
            max_steering=float(self.get_parameter('max_steering').value),
            max_steering_angle_deg=float(self.get_parameter('max_steering_angle_deg').value),
            lookahead_steps=int(self.get_parameter('lookahead_steps').value),
            heading_step=int(self.get_parameter('heading_step').value),
            car_center_x=float(self.get_parameter('car_center_x').value),
            car_center_y=float(self.get_parameter('car_center_y').value),
            default_left_speed=self.speed,
            default_right_speed=self.speed,
            stop_on_red=False,
            steering_sign=int(round(float(self.get_parameter('steering_sign').value))),
            heading_gain=float(self.get_parameter('heading_gain').value),
        )

        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, image_qos)
        self.cmd_pub = self.create_publisher(RosMotionCommand, cmd_topic, cmd_qos)
        self.debug_pub = self.create_publisher(Image, debug_topic, image_qos)
        self.enable_srv = self.create_service(SetBool, '~/set_enabled', self.on_set_enabled)

        self.get_logger().info(
            f'Track controller ready. enabled={self.enabled}. '
            f'Call /track_controller_node/set_enabled to start/stop.'
        )

    def on_set_enabled(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self.publish_stop()
        response.success = True
        response.message = 'ENABLED' if self.enabled else 'DISABLED / STOPPED'
        self.get_logger().warn(f'Track driving {response.message}')
        return response

    def publish_stop(self):
        msg = RosMotionCommand()
        msg.steering = 0
        msg.left_speed = 0
        msg.right_speed = 0
        self.cmd_pub.publish(msg)

    @staticmethod
    def _draw_detection(frame, detections):
        for det in detections.detections:
            x = int(det.bbox.center.position.x)
            y = int(det.bbox.center.position.y)
            w = int(det.bbox.size.x)
            h = int(det.bbox.size.y)
            x1, y1 = max(0, x - w // 2), max(0, y - h // 2)
            x2, y2 = min(frame.shape[1] - 1, x + w // 2), min(frame.shape[0] - 1, y + h // 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.putText(frame, f'{det.class_name}:{det.score:.2f}', (x1, max(15, y1 - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    def _make_debug(self, frame, detections, lane_info, path_result, cmd, fps):
        out = frame.copy()
        self._draw_detection(out, detections)

        # Target/path coordinates are BEV coordinates; the original drive.py overlays them
        # approximately onto the source image after adding roi_cut. We preserve that debug convention.
        roi_cut = int(self.lane.roi_cut)
        for tp in lane_info.target_points:
            x = int(tp.target_x)
            y = int(tp.target_y) + roi_cut
            if 0 <= x < out.shape[1] and 0 <= y < out.shape[0]:
                cv2.circle(out, (x, y), 4, (0, 0, 255), -1)

        pts = []
        for x, y in zip(path_result.x_points, path_result.y_points):
            px, py = int(x), int(y) + roi_cut
            if 0 <= px < out.shape[1] and 0 <= py < out.shape[0]:
                pts.append((px, py))
        for i in range(len(pts) - 1):
            cv2.line(out, pts[i], pts[i + 1], (255, 0, 0), 1)

        state = 'DRIVING' if self.enabled else 'DISABLED'
        cv2.putText(out, f'{state} fps={fps:.1f} steer={cmd.steering:+d} speed={cmd.left_speed}',
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                    (0, 255, 0) if self.enabled else (0, 0, 255), 2)
        cv2.putText(out, f'head={self.motion.last_heading_deg:+.1f}deg cte={self.motion.last_cte:+.1f}px',
                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 1)
        return out

    def on_image(self, image_msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            detections = self.yolo.detect(frame)
            lane_info = self.lane.process(detections, frame)
            path_result = self.path_planner.plan(lane_info)
            internal_cmd = self.motion.plan(detections, path_result, 'None', False)

            ros_cmd = RosMotionCommand()
            ros_cmd.steering = int(max(-7, min(7, internal_cmd.steering)))
            if self.enabled:
                ros_cmd.left_speed = int(max(-255, min(255, internal_cmd.left_speed)))
                ros_cmd.right_speed = int(max(-255, min(255, internal_cmd.right_speed)))
            else:
                ros_cmd.left_speed = 0
                ros_cmd.right_speed = 0
            self.cmd_pub.publish(ros_cmd)

            now = time.monotonic()
            fps = 0.0 if self.last_t is None else 1.0 / max(1e-6, now - self.last_t)
            self.last_t = now
            self.frame_counter += 1

            if self.frame_counter % 10 == 0:
                self.get_logger().info(
                    f'enabled={self.enabled} fps={fps:.1f} steer={ros_cmd.steering:+d} '
                    f'L={ros_cmd.left_speed} R={ros_cmd.right_speed} '
                    f'head={self.motion.last_heading_deg:+.1f} cte={self.motion.last_cte:+.1f}'
                )

            if self.publish_debug:
                dbg = self._make_debug(frame, detections, lane_info, path_result, ros_cmd, fps)
                dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
                dbg_msg.header = image_msg.header
                self.debug_pub.publish(dbg_msg)

        except Exception as exc:
            self.get_logger().error(f'Track pipeline error: {type(exc).__name__}: {exc}')
            self.publish_stop()

    def destroy_node(self):
        try:
            self.publish_stop()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
