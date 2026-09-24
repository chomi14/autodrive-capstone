#!/usr/bin/env python3
"""Publish a video file as a live-camera-compatible ROS image stream."""

import math
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import Image


class VideoReplayNode(Node):
    """Replay video frames on the front-camera image topic."""

    def __init__(self):
        super().__init__('video_replay_node')

        self.declare_parameter('video_path', '')
        self.declare_parameter('loop', False)
        self.declare_parameter('playback_fps', 0.0)
        self.declare_parameter('topic', '/camera/front/image_raw')
        self.declare_parameter('frame_id', 'front_camera_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('wait_for_subscriber', False)
        self.declare_parameter('max_frames', 0)
        self.declare_parameter('reliability', 'reliable')

        raw_path = str(self.get_parameter('video_path').value)
        self.video_path = Path(raw_path).expanduser()
        self.loop = bool(self.get_parameter('loop').value)
        requested_fps = float(self.get_parameter('playback_fps').value)
        self.topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.wait_for_subscriber = bool(self.get_parameter('wait_for_subscriber').value)
        self.max_frames = max(0, int(self.get_parameter('max_frames').value))
        reliability_name = str(self.get_parameter('reliability').value).lower()

        if not raw_path:
            raise RuntimeError('video_path is required')
        if not self.video_path.is_file():
            raise RuntimeError(f'Video file does not exist: {self.video_path}')
        if requested_fps < 0.0:
            raise RuntimeError('playback_fps must be 0 (source FPS) or greater than 0')
        if self.width <= 0 or self.height <= 0:
            raise RuntimeError('width and height must be positive')
        if reliability_name not in ('best_effort', 'reliable'):
            raise RuntimeError('reliability must be best_effort or reliable')

        self.cap = cv2.VideoCapture(str(self.video_path))
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open video file: {self.video_path}')

        source_fps = float(self.cap.get(cv2.CAP_PROP_FPS))
        if not math.isfinite(source_fps) or source_fps <= 0.0:
            source_fps = 30.0
            self.get_logger().warn('Video has no valid FPS metadata; using 30.0 FPS')
        self.playback_fps = requested_fps if requested_fps > 0.0 else source_fps

        qos = QoSProfile(
            depth=1,
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if reliability_name == 'best_effort'
                else ReliabilityPolicy.RELIABLE
            ),
            history=HistoryPolicy.KEEP_LAST,
        )
        self.publisher = self.create_publisher(Image, self.topic, qos)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / self.playback_fps, self.on_timer)
        self.frame_count = 0
        self._waiting_logged = False
        self._started = False
        self._finished = False
        self._first_publish_time = None
        self._last_publish_time = None
        self._stop_deadline = None
        self.done = Future()

        source_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        source_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        source_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.get_logger().info(
            f'Video opened: {self.video_path} '
            f'(source={source_width}x{source_height}@{source_fps:.3f}, frames={source_frames}); '
            f'publishing={self.width}x{self.height}@{self.playback_fps:.3f} '
            f'topic={self.topic}, encoding=bgr8, reliability={reliability_name}, '
            f'loop={self.loop}'
        )

    def _finish(self, reason):
        if self._finished:
            return
        self._finished = True
        self.timer.cancel()
        self.get_logger().info(reason)
        self.done.set_result(True)

    def on_timer(self):
        if self._stop_deadline is not None:
            if time.monotonic() >= self._stop_deadline:
                elapsed = max(0.0, self._last_publish_time - self._first_publish_time)
                actual_fps = (
                    (self.frame_count - 1) / elapsed
                    if self.frame_count > 1 and elapsed > 0.0 else 0.0
                )
                self._finish(
                    f'REPLAY_SUMMARY published_frames={self.frame_count} '
                    f'elapsed_sec={elapsed:.6f} actual_publish_fps={actual_fps:.3f}'
                )
            return

        if (
            self.wait_for_subscriber
            and not self._started
            and self.publisher.get_subscription_count() == 0
        ):
            if not self._waiting_logged:
                self.get_logger().info('Waiting for an image subscriber before starting replay')
                self._waiting_logged = True
            return

        if not self._started:
            self._started = True
        if self._waiting_logged:
            self.get_logger().info('Image subscriber connected; starting replay')
            self._waiting_logged = False

        ok, frame = self.cap.read()
        if (not ok or frame is None) and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
            if ok and frame is not None:
                self.get_logger().info('Looping video from the first frame')

        if not ok or frame is None:
            self._finish(
                f'Video replay complete after {self.frame_count} published frames'
                if not self.loop
                else 'Video rewind failed; stopping replay safely'
            )
            return

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)
        self.frame_count += 1
        publish_time = time.monotonic()
        if self._first_publish_time is None:
            self._first_publish_time = publish_time
        self._last_publish_time = publish_time
        if self.max_frames > 0 and self.frame_count >= self.max_frames:
            # Leave a short drain interval so the depth-1 subscriber can finish
            # its last callback before this process triggers launch shutdown.
            self._stop_deadline = publish_time + 1.0

    def destroy_node(self):
        try:
            if getattr(self, 'cap', None) is not None:
                self.cap.release()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VideoReplayNode()
        rclpy.spin_until_future_complete(node, node.done)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        print(f'[video_replay_node] ERROR: {exc}', flush=True)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
