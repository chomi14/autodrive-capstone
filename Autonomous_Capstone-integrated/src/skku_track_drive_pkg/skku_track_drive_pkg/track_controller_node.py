#!/usr/bin/env python3
import json
import statistics
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
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

        # Parameter callbacks and image callbacks are serialized by the default
        # executor today.  The lock also keeps a future multi-threaded executor
        # from applying only half of a tuning update to one frame.
        self._parameter_lock = threading.RLock()

        default_model = str(
            Path(get_package_share_directory('skku_track_drive_pkg'))
            / 'models'
            / 'best.pt'
        )

        # ROS I/O
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('cmd_topic', 'topic_control_signal')
        self.declare_parameter('debug_topic', 'track_debug_image')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('publish_bev_debug', True)
        self.declare_parameter('debug_log', False)
        self.declare_parameter('debug_log_interval', 10)
        self.declare_parameter('profile', False)
        self.declare_parameter('profile_warmup_frames', 10)
        self.declare_parameter('profile_report_interval', 50)
        self.declare_parameter('profile_input_fps', 0.0)
        self.declare_parameter('image_reliability', 'best_effort')
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('loaded_tuning_config', '')

        # YOLO
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence', 0.5)

        # Vehicle/control. Current Arduino firmware uses -7..+7 and negative=left.
        self.declare_parameter('max_steering', 7.0)
        self.declare_parameter('speed', 255)
        self.declare_parameter('allow_speed_tuning', True)
        self.declare_parameter('steering_sign', 1.0)
        self.declare_parameter('max_steering_angle', 50.0)
        self.declare_parameter('stanley_gain', 0.02)
        self.declare_parameter('stanley_softening', 0.001)
        self.declare_parameter('lookahead_index', 10)
        self.declare_parameter('heading_step', 3)
        self.declare_parameter('heading_gain', 0.6)
        self.declare_parameter('car_center_x', 320.0)
        self.declare_parameter('car_center_y', 179.0)

        # Lane/BEV values copied from the summer track-driving config.
        self.declare_parameter('bev_top_shift', -8)
        self.declare_parameter('roi_cut', 300)
        self.declare_parameter('look_shift', 50)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('virtual_lane_width', 300)
        self.declare_parameter('bev_pad', 250)
        self.declare_parameter('center_ema_alpha', 0.35)
        self.declare_parameter('max_center_jump_px', 80.0)
        self.declare_parameter('max_missed_frames', 12)
        self.declare_parameter('min_component_area', 250)

        image_topic = self.get_parameter('image_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.publish_bev_debug = bool(self.get_parameter('publish_bev_debug').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)
        self.debug_log_interval = max(1, int(self.get_parameter('debug_log_interval').value))
        self.profile = bool(self.get_parameter('profile').value)
        self.profile_warmup_frames = max(
            0, int(self.get_parameter('profile_warmup_frames').value)
        )
        self.profile_report_interval = max(
            1, int(self.get_parameter('profile_report_interval').value)
        )
        self.profile_input_fps = max(
            0.0, float(self.get_parameter('profile_input_fps').value)
        )
        image_reliability = str(self.get_parameter('image_reliability').value).lower()
        if image_reliability not in ('best_effort', 'reliable'):
            raise RuntimeError('image_reliability must be best_effort or reliable')
        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.loaded_tuning_config = str(self.get_parameter('loaded_tuning_config').value)

        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        conf = float(self.get_parameter('confidence').value)

        self.speed = int(self.get_parameter('speed').value)
        self.allow_speed_tuning = bool(
            self.get_parameter('allow_speed_tuning').value
        )
        self.bridge = CvBridge()
        self._warned_generic_bgr_encoding = False
        self.last_t = None
        self.frame_counter = 0
        self.profile_warmup_total_ms = []
        self.profile_timings = {
            'image_conversion': [],
            'yolo': [],
            'lane_extractor': [],
            'path_planner': [],
            'motion_planner': [],
            'visualization': [],
            'total': [],
        }
        self.profile_first_stamp_ns = None
        self.profile_last_stamp_ns = None
        self.profile_first_wall = None
        self.profile_last_wall = None

        self.get_logger().info(f'Loading summer track model: {model_path}')
        self.yolo = YoloDetector(model_path, device=device, conf=conf)
        if self.profile:
            self._log_profile_device(device)
        self.lane = LaneInfoExtractor(
            show_image=False,
            bev_top_shift=int(self.get_parameter('bev_top_shift').value),
            roi_cut=int(self.get_parameter('roi_cut').value),
            look_shift=int(self.get_parameter('look_shift').value),
            slope_ema_alpha=float(self.get_parameter('ema_alpha').value),
            virtual_lane_width=int(self.get_parameter('virtual_lane_width').value),
            bev_pad=int(self.get_parameter('bev_pad').value),
            capture_debug=self.publish_debug,
            center_ema_alpha=float(self.get_parameter('center_ema_alpha').value),
            max_center_jump_px=float(self.get_parameter('max_center_jump_px').value),
            max_missed_frames=int(self.get_parameter('max_missed_frames').value),
            min_component_area=int(self.get_parameter('min_component_area').value),
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
            max_steering_angle_deg=float(self.get_parameter('max_steering_angle').value),
            lookahead_steps=int(self.get_parameter('lookahead_index').value),
            heading_step=int(self.get_parameter('heading_step').value),
            car_center_x=float(self.get_parameter('car_center_x').value),
            car_center_y=float(self.get_parameter('car_center_y').value),
            default_left_speed=self.speed,
            default_right_speed=self.speed,
            stop_on_red=False,
            steering_sign=int(round(float(self.get_parameter('steering_sign').value))),
            heading_gain=float(self.get_parameter('heading_gain').value),
        )
        self._validate_initial_tuning()
        self._parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_tuning_parameters
        )

        image_qos = QoSProfile(
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if image_reliability == 'reliable'
                else ReliabilityPolicy.BEST_EFFORT
            ),
            history=HistoryPolicy.KEEP_LAST,
        )
        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(Image, image_topic, self.on_image, image_qos)
        self.cmd_pub = self.create_publisher(RosMotionCommand, cmd_topic, cmd_qos)
        # Debug images are large and optional.  Never let a slow GUI apply
        # reliable-DDS backpressure to the control callback.
        self.debug_pub = self.create_publisher(Image, debug_topic, debug_qos)
        self.enable_srv = self.create_service(SetBool, '~/set_enabled', self.on_set_enabled)

        self.get_logger().info(
            f'Track controller ready. enabled={self.enabled}. '
            f'Call /track_controller_node/set_enabled to start/stop.'
        )
        if self.loaded_tuning_config:
            self.get_logger().info(f'Loaded tuning config: {self.loaded_tuning_config}')

    @staticmethod
    def _tuning_limits():
        """Runtime-safe numeric limits; the GUI may intentionally be narrower."""
        return {
            # Arduino motor commands use signed 8-bit PWM (-255..255).
            'speed': (int, 0, 255),
            'stanley_gain': (float, 0.0, 0.2),
            'heading_gain': (float, 0.0, 2.0),
            'lookahead_index': (int, 1, 99),
            'confidence': (float, 0.0, 1.0),
            'bev_top_shift': (int, -100, 100),
            'look_shift': (int, -150, 150),
            'ema_alpha': (float, 0.0, 1.0),
            'virtual_lane_width': (int, 1, 1000),
            'max_steering_angle': (float, 1.0, 90.0),
            # Advanced parameters are dynamic but intentionally absent from the GUI.
            'stanley_softening': (float, 0.0, 10.0),
            'heading_step': (int, 1, 99),
            'roi_cut': (int, 0, 470),
            'bev_pad': (int, 0, 1000),
            'car_center_x': (float, -2000.0, 2000.0),
            'car_center_y': (float, -2000.0, 2000.0),
            'center_ema_alpha': (float, 0.0, 1.0),
            'max_center_jump_px': (float, 1.0, 640.0),
            'max_missed_frames': (int, 0, 120),
            'min_component_area': (int, 1, 100000),
        }

    @classmethod
    def _validate_tuning_value(cls, name, value):
        expected_type, minimum, maximum = cls._tuning_limits()[name]
        if expected_type is int:
            valid_type = isinstance(value, int) and not isinstance(value, bool)
        else:
            valid_type = isinstance(value, float)
        if not valid_type:
            return f'{name} must be {expected_type.__name__}'
        if not minimum <= value <= maximum:
            return f'{name} must be in [{minimum}, {maximum}]'
        return ''

    def _validate_initial_tuning(self):
        errors = []
        for name in self._tuning_limits():
            value = self.get_parameter(name).value
            reason = self._validate_tuning_value(name, value)
            if reason:
                errors.append(reason)
        if errors:
            raise RuntimeError('Invalid tuning parameters: ' + '; '.join(errors))

    def _apply_tuning_values(self, updates):
        """Apply validated values to the objects used by the image callback."""
        if 'speed' in updates:
            self.speed = updates['speed']
            self.motion.default_left_speed = updates['speed']
            self.motion.default_right_speed = updates['speed']
        if 'confidence' in updates:
            self.yolo.conf = updates['confidence']

        motion_attributes = {
            'stanley_gain': 'stanley_gain',
            'heading_gain': 'heading_gain',
            'lookahead_index': 'lookahead_steps',
            'max_steering_angle': 'max_steering_angle_deg',
            'stanley_softening': 'stanley_softening',
            'heading_step': 'heading_step',
            'car_center_x': 'car_center_x',
            'car_center_y': 'car_center_y',
        }
        for parameter_name, attribute_name in motion_attributes.items():
            if parameter_name in updates:
                setattr(self.motion, attribute_name, updates[parameter_name])

        lane_attributes = {
            'bev_top_shift': 'bev_top_shift',
            'look_shift': 'look_shift',
            'ema_alpha': 'slope_ema_alpha',
            'virtual_lane_width': 'virtual_lane_width',
            'roi_cut': 'roi_cut',
            'bev_pad': 'bev_pad',
            'center_ema_alpha': 'center_ema_alpha',
            'max_center_jump_px': 'max_center_jump_px',
            'max_missed_frames': 'max_missed_frames',
            'min_component_area': 'min_component_area',
        }
        for parameter_name, attribute_name in lane_attributes.items():
            if parameter_name in updates:
                setattr(self.lane, attribute_name, updates[parameter_name])

        if 'car_center_x' in updates or 'car_center_y' in updates:
            self.path_planner.car_center_point = (
                self.motion.car_center_x,
                self.motion.car_center_y,
            )

    def on_tuning_parameters(self, parameters):
        if not self.allow_speed_tuning and any(
            parameter.name == 'speed' for parameter in parameters
        ):
            return SetParametersResult(
                successful=False,
                reason='speed is fixed for this launch',
            )
        display_parameter_names = {'publish_debug', 'publish_bev_debug'}
        restart_only = [
            parameter.name
            for parameter in parameters
            if parameter.name not in self._tuning_limits()
            and parameter.name not in display_parameter_names
        ]
        if restart_only:
            return SetParametersResult(
                successful=False,
                reason=(
                    'Runtime update is not supported for restart-only parameters: '
                    + ', '.join(restart_only)
                ),
            )
        tuning_updates = {
            parameter.name: parameter.value
            for parameter in parameters
            if parameter.name in self._tuning_limits()
        }
        display_updates = {
            parameter.name: parameter.value
            for parameter in parameters
            if parameter.name in display_parameter_names
        }
        for name, value in tuning_updates.items():
            reason = self._validate_tuning_value(name, value)
            if reason:
                return SetParametersResult(successful=False, reason=reason)
        for name, value in display_updates.items():
            if not isinstance(value, bool):
                return SetParametersResult(
                    successful=False,
                    reason=f'{name} must be bool',
                )

        if tuning_updates or display_updates:
            with self._parameter_lock:
                self._apply_tuning_values(tuning_updates)
                if 'publish_debug' in display_updates:
                    self.publish_debug = display_updates['publish_debug']
                    self.lane.capture_debug = self.publish_debug
                if 'publish_bev_debug' in display_updates:
                    self.publish_bev_debug = display_updates['publish_bev_debug']
            applied = dict(tuning_updates)
            applied.update(display_updates)
            rendered = ', '.join(f'{name}={value}' for name, value in applied.items())
            self.get_logger().info(f'TUNING_APPLIED {rendered}')
        return SetParametersResult(successful=True)

    def _log_profile_device(self, requested_device):
        try:
            import torch

            model_device = str(next(self.yolo.yolo.model.parameters()).device)
            payload = {
                'requested_device': requested_device,
                'model_device': model_device,
                'torch_version': torch.__version__,
                'torch_cuda_version': torch.version.cuda,
                'cuda_available': torch.cuda.is_available(),
                'cuda_device_index': (
                    torch.cuda.current_device() if torch.cuda.is_available() else None
                ),
                'cuda_device_name': (
                    torch.cuda.get_device_name(torch.cuda.current_device())
                    if torch.cuda.is_available() else None
                ),
                'cuda_allocated_mib': (
                    round(torch.cuda.memory_allocated() / 1024 ** 2, 3)
                    if torch.cuda.is_available() else 0.0
                ),
                'cuda_reserved_mib': (
                    round(torch.cuda.memory_reserved() / 1024 ** 2, 3)
                    if torch.cuda.is_available() else 0.0
                ),
            }
            self.get_logger().info(f'PROFILE_DEVICE {json.dumps(payload, sort_keys=True)}')
        except Exception as exc:
            self.get_logger().warn(f'PROFILE_DEVICE unavailable: {exc}')

    @staticmethod
    def _percentile(values, percentile):
        if not values:
            return 0.0
        ordered = sorted(values)
        position = (len(ordered) - 1) * percentile / 100.0
        lower = int(position)
        upper = min(lower + 1, len(ordered) - 1)
        fraction = position - lower
        return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction

    @classmethod
    def _timing_stats(cls, values):
        if not values:
            return {'mean_ms': 0.0, 'median_ms': 0.0, 'p95_ms': 0.0, 'max_ms': 0.0}
        return {
            'mean_ms': round(statistics.fmean(values), 3),
            'median_ms': round(statistics.median(values), 3),
            'p95_ms': round(cls._percentile(values, 95.0), 3),
            'max_ms': round(max(values), 3),
        }

    @staticmethod
    def _stamp_ns(image_msg):
        return image_msg.header.stamp.sec * 1_000_000_000 + image_msg.header.stamp.nanosec

    def _record_profile(self, image_msg, durations_ms):
        frame_number = self.frame_counter
        if frame_number <= self.profile_warmup_frames:
            self.profile_warmup_total_ms.append(durations_ms['total'])
            if frame_number == self.profile_warmup_frames:
                stats = self._timing_stats(self.profile_warmup_total_ms)
                self.get_logger().info(
                    f'PROFILE_WARMUP frames={frame_number} '
                    f'total_mean_ms={stats["mean_ms"]} total_max_ms={stats["max_ms"]}'
                )
            return

        stamp_ns = self._stamp_ns(image_msg)
        now = time.perf_counter()
        if self.profile_first_stamp_ns is None:
            self.profile_first_stamp_ns = stamp_ns
            self.profile_first_wall = now
        self.profile_last_stamp_ns = stamp_ns
        self.profile_last_wall = now
        for stage, value in durations_ms.items():
            self.profile_timings[stage].append(value)

        samples = len(self.profile_timings['total'])
        if samples % self.profile_report_interval == 0:
            self._report_profile()

    def _report_profile(self):
        samples = len(self.profile_timings['total'])
        if samples == 0:
            return

        wall_elapsed = max(0.0, self.profile_last_wall - self.profile_first_wall)
        stamp_elapsed = max(
            0.0, (self.profile_last_stamp_ns - self.profile_first_stamp_ns) / 1e9
        )
        published_estimate = samples
        if self.profile_input_fps > 0.0:
            published_estimate = max(
                samples, int(round(stamp_elapsed * self.profile_input_fps)) + 1
            )
        dropped_estimate = max(0, published_estimate - samples)
        total_stats = self._timing_stats(self.profile_timings['total'])
        actual_processed_hz = (
            (samples - 1) / wall_elapsed if samples > 1 and wall_elapsed > 0.0 else 0.0
        )
        effective_fps = (
            1000.0 / total_stats['mean_ms'] if total_stats['mean_ms'] > 0.0 else 0.0
        )
        payload = {
            'samples': samples,
            'warmup_frames_excluded': self.profile_warmup_frames,
            'input_fps': self.profile_input_fps,
            'published_frames_estimate': published_estimate,
            'processed_frames': samples,
            'dropped_frames_estimate': dropped_estimate,
            'drop_ratio': round(dropped_estimate / published_estimate, 6),
            'actual_processed_hz': round(actual_processed_hz, 3),
            'steering_publish_hz': round(actual_processed_hz, 3),
            'effective_processing_fps': round(effective_fps, 3),
            'stages': {
                stage: self._timing_stats(values)
                for stage, values in self.profile_timings.items()
            },
        }
        try:
            import torch

            if torch.cuda.is_available():
                payload['cuda_allocated_mib'] = round(
                    torch.cuda.memory_allocated() / 1024 ** 2, 3
                )
                payload['cuda_reserved_mib'] = round(
                    torch.cuda.memory_reserved() / 1024 ** 2, 3
                )
        except Exception:
            pass
        self.get_logger().info(f'PROFILE_SUMMARY {json.dumps(payload, sort_keys=True)}')

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

    def publish_perception_fallback(self):
        """Keep driving through a transient perception/pipeline failure.

        A broken lane mask must not inject a zero-speed command between valid
        frames.  Hold the most recent valid steering command and keep the
        configured, equal motor command.  Explicit disable/arm safety paths
        still use ``publish_stop`` and remain unchanged.
        """
        if not self.enabled:
            self.publish_stop()
            return
        msg = RosMotionCommand()
        msg.steering = int(max(-7, min(7, self.motion.last_target_steer)))
        msg.left_speed = int(max(-255, min(255, self.speed)))
        msg.right_speed = int(max(-255, min(255, self.speed)))
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

    @staticmethod
    def _lane_polygons(detections, width, height):
        polygons = []
        for detection in detections.detections:
            if detection.class_name != 'lane2' or len(detection.mask.data) < 3:
                continue
            points = np.array(
                [[point.x, point.y] for point in detection.mask.data],
                dtype=np.float32,
            )
            points[:, 0] = np.clip(points[:, 0], 0, width - 1)
            points[:, 1] = np.clip(points[:, 1], 0, height - 1)
            polygons.append(np.rint(points).astype(np.int32))
        return polygons

    @staticmethod
    def _transform_bev_points(points, inverse_transform):
        if not points or inverse_transform is None:
            return []
        source = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
        transformed = cv2.perspectiveTransform(source, inverse_transform)
        return [
            tuple(int(value) for value in np.rint(point))
            for point in transformed[:, 0, :]
        ]

    @staticmethod
    def _draw_path(image, points, color, thickness=2):
        visible = [
            point for point in points
            if 0 <= point[0] < image.shape[1] and 0 <= point[1] < image.shape[0]
        ]
        if len(visible) >= 2:
            cv2.polylines(
                image,
                [np.asarray(visible, dtype=np.int32)],
                False,
                color,
                thickness,
                cv2.LINE_AA,
            )

    def _make_bev_debug(self, frame_shape, path_result):
        height, width = frame_shape[:2]
        debug = self.lane.last_debug
        if debug is None:
            panel = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(
                panel,
                'Lane BEV: no lane intermediate',
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )
            return panel

        mask = debug['bev_mask']
        panel = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
        panel[mask > 0] = (70, 70, 70)
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(panel, contours, -1, (255, 255, 0), 1, cv2.LINE_AA)

        roi_cut = int(debug['roi_cut'])
        cv2.line(panel, (0, roi_cut), (panel.shape[1] - 1, roi_cut), (90, 90, 90), 1)
        for target_y, lane_center in debug['samples']:
            y = int(target_y) + roi_cut
            for edge_x in lane_center.edges:
                cv2.circle(panel, (int(edge_x), y), 5, (0, 255, 0), -1)
            if lane_center.virtual_x is not None:
                cv2.circle(
                    panel,
                    (int(lane_center.virtual_x), y),
                    5,
                    (0, 165, 255),
                    -1,
                )
            cv2.circle(panel, (int(lane_center.center), y), 4, (0, 0, 255), -1)

        bev_path = [
            (int(round(x)), int(round(y)) + roi_cut)
            for x, y in zip(path_result.x_points, path_result.y_points)
        ]
        self._draw_path(panel, bev_path, (255, 80, 0), 2)

        vehicle = (
            int(round(self.motion.car_center_x)),
            int(round(self.motion.car_center_y)) + roi_cut,
        )
        cv2.drawMarker(
            panel,
            vehicle,
            (255, 0, 255),
            cv2.MARKER_CROSS,
            18,
            2,
        )
        if self.motion.last_reference_point is not None:
            reference = (
                int(round(self.motion.last_reference_point[0])),
                int(round(self.motion.last_reference_point[1])) + roi_cut,
            )
            cv2.circle(panel, reference, 7, (0, 255, 255), 2)

        cv2.putText(
            panel,
            'Lane BEV  cyan=mask green=lane orange=virtual red=center',
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            panel,
            'blue=spline magenta=vehicle yellow=reference',
            (8, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return panel

    def _make_debug(
        self,
        frame,
        detections,
        lane_info,
        path_result,
        cmd,
        fps,
        lane_detected,
        path_generated,
    ):
        out = frame.copy()
        polygons = self._lane_polygons(detections, out.shape[1], out.shape[0])
        if polygons:
            mask_overlay = out.copy()
            cv2.fillPoly(mask_overlay, polygons, (255, 255, 0))
            out = cv2.addWeighted(mask_overlay, 0.28, out, 0.72, 0.0)
            cv2.polylines(out, polygons, True, (255, 255, 0), 1, cv2.LINE_AA)
        self._draw_detection(out, detections)

        lane_debug = self.lane.last_debug
        if lane_debug is not None:
            roi_cut = int(lane_debug['roi_cut'])
            inverse = lane_debug['inverse_transform']
            target_bev = [
                (point.target_x, point.target_y + roi_cut)
                for point in lane_info.target_points
            ]
            for point in self._transform_bev_points(target_bev, inverse):
                if 0 <= point[0] < out.shape[1] and 0 <= point[1] < out.shape[0]:
                    cv2.circle(out, point, 5, (0, 0, 255), -1)

            path_bev = [
                (x, y + roi_cut)
                for x, y in zip(path_result.x_points, path_result.y_points)
            ]
            raw_path = self._transform_bev_points(path_bev, inverse)
            self._draw_path(out, raw_path, (255, 80, 0), 2)

            vehicle_bev = [(
                self.motion.car_center_x,
                self.motion.car_center_y + roi_cut,
            )]
            vehicle_raw = self._transform_bev_points(vehicle_bev, inverse)
            if vehicle_raw:
                cv2.drawMarker(
                    out,
                    vehicle_raw[0],
                    (255, 0, 255),
                    cv2.MARKER_CROSS,
                    18,
                    2,
                )

            if self.motion.last_reference_point is not None:
                reference_bev = [(
                    self.motion.last_reference_point[0],
                    self.motion.last_reference_point[1] + roi_cut,
                )]
                reference_raw = self._transform_bev_points(reference_bev, inverse)
                if reference_raw:
                    cv2.circle(out, reference_raw[0], 7, (0, 255, 255), 2)

        state = 'DRIVING' if self.enabled else 'DISABLED'
        shade = out.copy()
        cv2.rectangle(shade, (0, 0), (out.shape[1] - 1, 78), (0, 0, 0), -1)
        out = cv2.addWeighted(shade, 0.62, out, 0.38, 0.0)
        cv2.putText(
            out,
            f'{state} fps={fps:.1f} steer={cmd.steering:+d} '
            f'speed={cmd.left_speed}/{cmd.right_speed}',
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 0) if self.enabled else (0, 0, 255),
            2,
        )
        cv2.putText(out,
                    f'lane={str(lane_detected).lower()} '
                    f'path={str(path_generated).lower()} '
                    f'src={lane_info.source} conf={lane_info.confidence:.2f} '
                    f'head={self.motion.last_heading_deg:+.1f}deg '
                    f'cte={self.motion.last_cte:+.1f}px',
                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 1)
        cv2.putText(
            out,
            'cyan=YOLO lane red=target blue=path magenta=vehicle yellow=reference',
            (10, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )

        if self.publish_bev_debug:
            bev_panel = self._make_bev_debug(frame.shape, path_result)
            return np.hstack((out, bev_panel))
        return out

    def _image_to_bgr(self, image_msg: Image):
        """Accept canonical BGR8 and legacy OpenCV 8UC3 image messages.

        Some publishers created with cv_bridge's passthrough default label a
        three-channel OpenCV frame as ``8UC3``. cv_bridge refuses to convert
        that generic encoding directly to ``bgr8``, even though those legacy
        publishers supply BGR data. Handle it explicitly so one such frame does
        not abort the complete perception/control callback.
        """
        encoding = str(image_msg.encoding).lower()
        if encoding == '8uc3':
            if not self._warned_generic_bgr_encoding:
                self.get_logger().warn(
                    'Received legacy 8UC3 image encoding; treating it as BGR8'
                )
                self._warned_generic_bgr_encoding = True
            frame = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding='passthrough'
            )
        else:
            frame = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding='bgr8'
            )
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f'Expected a 3-channel image, got shape={frame.shape}')
        return np.ascontiguousarray(frame)

    def on_image(self, image_msg: Image):
        try:
            callback_start = time.perf_counter() if self.profile else None
            frame = self._image_to_bgr(image_msg)
            conversion_end = time.perf_counter() if self.profile else None
            with self._parameter_lock:
                detections = self.yolo.detect(frame)
                yolo_end = time.perf_counter() if self.profile else None
                lane_info = self.lane.process(detections, frame)
                lane_end = time.perf_counter() if self.profile else None
                path_result = self.path_planner.plan(lane_info)
                path_end = time.perf_counter() if self.profile else None
                internal_cmd = self.motion.plan(detections, path_result, 'None', False)
                motion_end = time.perf_counter() if self.profile else None
                tuning_snapshot = (
                    self.motion.stanley_gain,
                    self.motion.heading_gain,
                    self.motion.lookahead_steps,
                )

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
            lane_detected = bool(lane_info.valid)
            path_generated = len(path_result.x_points) >= 2

            if self.debug_log and self.frame_counter % self.debug_log_interval == 0:
                self.get_logger().info(
                    f'frame={self.frame_counter} processed=true '
                    f'lane_detected={str(lane_detected).lower()} '
                    f'path_generated={str(path_generated).lower()} '
                    f'fps={fps:.1f} steer={ros_cmd.steering:+d} '
                    f'L={ros_cmd.left_speed} R={ros_cmd.right_speed} '
                    f'head={self.motion.last_heading_deg:+.1f} cte={self.motion.last_cte:+.1f} '
                    f'k={tuning_snapshot[0]:.4f} heading_gain={tuning_snapshot[1]:.3f} '
                    f'lookahead={tuning_snapshot[2]}'
                )

            visualization_start = time.perf_counter() if self.profile else None
            if self.publish_debug:
                dbg = self._make_debug(
                    frame,
                    detections,
                    lane_info,
                    path_result,
                    ros_cmd,
                    fps,
                    lane_detected,
                    path_generated,
                )
                dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
                dbg_msg.header = image_msg.header
                self.debug_pub.publish(dbg_msg)

            callback_end = time.perf_counter() if self.profile else None
            if self.profile:
                self._record_profile(image_msg, {
                    'image_conversion': (conversion_end - callback_start) * 1000.0,
                    'yolo': (yolo_end - conversion_end) * 1000.0,
                    'lane_extractor': (lane_end - yolo_end) * 1000.0,
                    'path_planner': (path_end - lane_end) * 1000.0,
                    'motion_planner': (motion_end - path_end) * 1000.0,
                    'visualization': (
                        callback_end - visualization_start
                    ) * 1000.0,
                    'total': (callback_end - callback_start) * 1000.0,
                })

        except Exception as exc:
            self.get_logger().error(
                f'Track pipeline error: {type(exc).__name__}: {exc}; '
                'holding last steering and configured speed'
            )
            self.publish_perception_fallback()

    def destroy_node(self):
        try:
            if self.profile:
                self._report_profile()
            # SIGINT may already have invalidated the ROS context (for example
            # when the replay launch shuts down at EOF), so only publish while
            # the publisher can still be used.
            if rclpy.ok():
                self.publish_stop()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
