#!/usr/bin/env python3
"""OpenCV tuning panel backed by the ROS 2 parameter services."""

import os
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_parameter_events,
)
from sensor_msgs.msg import Image

from interfaces_pkg.msg import MotionCommand


TUNER_WINDOW_NAME = 'Track Tuner'
DEBUG_WINDOW_NAME = 'Track Debug View'

# label, maximum trackbar position, position -> value, value -> position
TRACKBARS = {
    'speed': ('Speed', 255, lambda p: int(p), lambda v: int(v)),
    'stanley_gain': (
        'Stanley Gain x1000', 75,
        lambda p: round((p + 5) / 1000.0, 3),
        lambda v: round(float(v) * 1000.0) - 5,
    ),
    'heading_gain': (
        'Heading Gain x100', 150,
        lambda p: round(p / 100.0, 2),
        lambda v: round(float(v) * 100.0),
    ),
    'lookahead_index': (
        'Lookahead Index', 27,
        lambda p: int(p + 3),
        lambda v: int(v) - 3,
    ),
    'confidence': (
        'Confidence x100', 60,
        lambda p: round((p + 20) / 100.0, 2),
        lambda v: round(float(v) * 100.0) - 20,
    ),
    'bev_top_shift': (
        'BEV Top Shift', 80,
        lambda p: int(p - 40),
        lambda v: int(v) + 40,
    ),
    'look_shift': ('Look Shift', 120, lambda p: int(p), lambda v: int(v)),
    'ema_alpha': (
        'EMA Alpha x100', 100,
        lambda p: round(p / 100.0, 2),
        lambda v: round(float(v) * 100.0),
    ),
    'virtual_lane_width': (
        'Virtual Lane Width', 200,
        lambda p: int(p + 200),
        lambda v: int(v) - 200,
    ),
    'max_steering_angle': (
        'Max Steering Angle', 40,
        lambda p: float(p + 30),
        lambda v: round(float(v)) - 30,
    ),
}
DISPLAY_PARAMETERS = ('publish_debug', 'publish_bev_debug')


class TrackTunerNode(Node):
    """Change TrackController parameters; never publishes actuator commands."""

    def __init__(self):
        super().__init__('track_tuner_node')
        self.declare_parameter('target_node', '/track_controller_node')
        self.declare_parameter('cmd_topic', '/topic_control_signal')
        self.declare_parameter('debug_topic', '/track_debug_image')
        self.declare_parameter(
            'save_path', str(Path('~/.config/autodrive/track_tuning.yaml').expanduser())
        )
        self.declare_parameter('loaded_config_path', '')
        self.declare_parameter('allow_speed_tuning', True)
        self.declare_parameter('fixed_speed', 250)

        self.target_node = str(self.get_parameter('target_node').value).rstrip('/')
        self.save_path = Path(str(self.get_parameter('save_path').value)).expanduser()
        self.loaded_config_path = str(self.get_parameter('loaded_config_path').value)
        self.allow_speed_tuning = bool(
            self.get_parameter('allow_speed_tuning').value
        )
        self.fixed_speed = int(self.get_parameter('fixed_speed').value)
        self.trackbars = dict(TRACKBARS)
        if not self.allow_speed_tuning:
            self.trackbars.pop('speed')
        self.managed_parameters = tuple(self.trackbars) + DISPLAY_PARAMETERS

        self.get_client = self.create_client(
            GetParameters, f'{self.target_node}/get_parameters'
        )
        self.set_client = self.create_client(
            SetParameters, f'{self.target_node}/set_parameters'
        )
        self.parameter_event_sub = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.on_parameter_event,
            qos_profile_parameter_events,
        )
        monitor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.cmd_sub = self.create_subscription(
            MotionCommand,
            str(self.get_parameter('cmd_topic').value),
            self.on_command,
            monitor_qos,
        )
        self.bridge = CvBridge()
        self.debug_sub = self.create_subscription(
            Image,
            str(self.get_parameter('debug_topic').value),
            self.on_debug_image,
            monitor_qos,
        )

        self.current_values = {}
        self.desired_values = {}
        self._get_future = None
        self._set_future = None
        self._set_values = None
        self._initializing_ui = False
        self._ui_ready = False
        self._window_closed = False
        self._debug_window_open = False
        self._latest_debug_frame = None
        self._last_command_time = None
        self.processing_fps = 0.0
        self.current_steering = 0
        self.current_left_speed = 0
        self.current_right_speed = 0
        self.timer = self.create_timer(1.0 / 30.0, self.on_timer)

        shown_config = self.loaded_config_path or '(controller defaults)'
        self.get_logger().info(f'Loaded tuning config: {shown_config}')
        self.get_logger().info(f'Tuning save path: {self.save_path}')
        self.get_logger().info('Waiting for TrackController parameter services')

    def on_debug_image(self, message):
        try:
            self._latest_debug_frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to convert debug image: {exc}')

    def on_command(self, message):
        now = time.monotonic()
        if self._last_command_time is not None:
            instantaneous = 1.0 / max(1e-6, now - self._last_command_time)
            if self.processing_fps == 0.0:
                self.processing_fps = instantaneous
            else:
                self.processing_fps = 0.1 * instantaneous + 0.9 * self.processing_fps
        self._last_command_time = now
        self.current_steering = int(message.steering)
        self.current_left_speed = int(message.left_speed)
        self.current_right_speed = int(message.right_speed)

    def on_parameter_event(self, event):
        if event.node.rstrip('/') != self.target_node:
            return
        changed = {}
        for parameter_message in event.changed_parameters:
            if parameter_message.name in self.managed_parameters:
                changed[parameter_message.name] = Parameter.from_parameter_msg(
                    parameter_message
                ).value
        if not changed:
            return
        self.current_values.update(changed)
        self.desired_values.update(changed)
        self._set_trackbar_positions()

    def _request_initial_values(self):
        request = GetParameters.Request()
        request.names = list(self.managed_parameters)
        self._get_future = self.get_client.call_async(request)
        self._get_future.add_done_callback(self._on_initial_values)

    def _on_initial_values(self, future):
        try:
            response = future.result()
            values = {
                name: parameter_value_to_python(value)
                for name, value in zip(self.managed_parameters, response.values)
            }
        except Exception as exc:
            self.get_logger().error(f'Failed to read tuning parameters: {exc}')
            self._get_future = None
            return

        self.current_values = values
        self.desired_values = dict(values)
        self._create_ui()
        rendered = ', '.join(f'{name}={value}' for name, value in values.items())
        self.get_logger().info(f'Initial tuning parameters: {rendered}')

    def _position_for(self, name, value):
        maximum = self.trackbars[name][1]
        position = self.trackbars[name][3](value)
        return max(0, min(maximum, int(position)))

    def _create_ui(self):
        try:
            cv2.namedWindow(TUNER_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(TUNER_WINDOW_NAME, 900, 720)
            self._initializing_ui = True
            for name, (label, maximum, _decode, _encode) in self.trackbars.items():
                position = self._position_for(name, self.current_values[name])
                cv2.createTrackbar(
                    label,
                    TUNER_WINDOW_NAME,
                    position,
                    maximum,
                    lambda pos, parameter_name=name: self._on_trackbar(
                        parameter_name, pos
                    ),
                )
            self._initializing_ui = False
            self._ui_ready = True
        except cv2.error as exc:
            raise RuntimeError(
                'OpenCV GUI could not open. Check DISPLAY/Wayland/X11 configuration.'
            ) from exc

    def _on_trackbar(self, name, position):
        if self._initializing_ui:
            return
        self.desired_values[name] = self.trackbars[name][2](position)

    def _set_trackbar_positions(self):
        if not self._ui_ready:
            return
        self._initializing_ui = True
        for name in self.trackbars:
            value = self.desired_values[name]
            cv2.setTrackbarPos(
                self.trackbars[name][0],
                TUNER_WINDOW_NAME,
                self._position_for(name, value),
            )
        self._initializing_ui = False

    def _send_pending_parameters(self):
        if self._set_future is not None:
            return
        updates = {
            name: value
            for name, value in self.desired_values.items()
            if self.current_values.get(name) != value
        }
        if not updates:
            return

        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=value).to_parameter_msg()
            for name, value in updates.items()
        ]
        self._set_values = updates
        self._set_future = self.set_client.call_async(request)
        self._set_future.add_done_callback(self._on_parameters_set)

    def _on_parameters_set(self, future):
        updates = self._set_values or {}
        self._set_future = None
        self._set_values = None
        try:
            response = future.result()
            failures = [result.reason for result in response.results if not result.successful]
        except Exception as exc:
            failures = [str(exc)]

        if failures:
            self.get_logger().error('Parameter update rejected: ' + '; '.join(failures))
            self.desired_values = dict(self.current_values)
            self._set_trackbar_positions()
            return

        self.current_values.update(updates)

    @staticmethod
    def write_yaml_atomic(path, values):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {'track_controller_node': {'ros__parameters': dict(values)}}
        temporary_name = None
        try:
            with tempfile.NamedTemporaryFile(
                mode='w',
                encoding='utf-8',
                dir=path.parent,
                prefix=f'.{path.name}.',
                suffix='.tmp',
                delete=False,
            ) as stream:
                temporary_name = stream.name
                yaml.safe_dump(data, stream, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, path)
            directory_fd = os.open(path.parent, os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except Exception:
            if temporary_name:
                try:
                    Path(temporary_name).unlink()
                except FileNotFoundError:
                    pass
            raise

    def save(self):
        values = {name: self.desired_values[name] for name in self.trackbars}
        self.write_yaml_atomic(self.save_path, values)
        self.get_logger().info(f'Saved tuning parameters to:\n{self.save_path}')
        for name, value in values.items():
            self.get_logger().info(f'  {name}: {value}')

    def _draw_status(self):
        canvas = np.zeros((300, 900, 3), dtype=np.uint8)
        lines = [
            'P: save   D: debug view   B: BEV panel   Q/ESC: close GUI only',
            (
                f'Debug: {"ON" if self.desired_values.get("publish_debug") else "OFF"}   '
                f'BEV: {"ON" if self.desired_values.get("publish_bev_debug") else "OFF"}'
            ),
            (
                f'configured speed: {self.desired_values.get("speed", "-")}'
                if self.allow_speed_tuning
                else f'fixed speed: {self.fixed_speed} (longitudinal tuning disabled)'
            ),
            (
                f'command steering: {self.current_steering:+d}   '
                f'L/R speed: {self.current_left_speed}/{self.current_right_speed}'
            ),
            f'control update rate: {self.processing_fps:.1f} Hz',
            f'loaded: {self.loaded_config_path or "controller defaults"}',
            f'save to: {self.save_path}',
            'Closing this GUI does not stop TrackController or vehicle control.',
        ]
        for index, line in enumerate(lines):
            cv2.putText(
                canvas,
                line,
                (12, 27 + index * 33),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
        cv2.imshow(TUNER_WINDOW_NAME, canvas)

    def _draw_debug(self):
        enabled = bool(self.desired_values.get('publish_debug', False))
        if not enabled:
            if self._debug_window_open:
                cv2.destroyWindow(DEBUG_WINDOW_NAME)
                self._debug_window_open = False
            return
        if self._latest_debug_frame is None:
            return
        if not self._debug_window_open:
            cv2.namedWindow(DEBUG_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                DEBUG_WINDOW_NAME,
                self._latest_debug_frame.shape[1],
                self._latest_debug_frame.shape[0],
            )
            self._debug_window_open = True
        cv2.imshow(DEBUG_WINDOW_NAME, self._latest_debug_frame)

    def _close_gui_windows(self, log_continues=True):
        if self._debug_window_open:
            cv2.destroyWindow(DEBUG_WINDOW_NAME)
            self._debug_window_open = False
        if self._ui_ready and not self._window_closed:
            cv2.destroyWindow(TUNER_WINDOW_NAME)
        self._window_closed = True
        if log_continues:
            self.get_logger().info(
                'GUI windows closed; TrackController and vehicle control '
                'continue running.'
            )

    def on_timer(self):
        if self._get_future is None:
            if self.get_client.service_is_ready() and self.set_client.service_is_ready():
                self._request_initial_values()
            return
        if not self._ui_ready or self._window_closed:
            return

        self._send_pending_parameters()
        self._draw_status()
        self._draw_debug()
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('p'), ord('P')):
            try:
                self.save()
            except Exception as exc:
                self.get_logger().error(f'Failed to save tuning parameters: {exc}')
        elif key in (ord('d'), ord('D')):
            enabled = not bool(self.desired_values.get('publish_debug', False))
            self.desired_values['publish_debug'] = enabled
            self.get_logger().info(
                f'Debug visualization {"enabled" if enabled else "disabled"}'
            )
            if not enabled and self._debug_window_open:
                cv2.destroyWindow(DEBUG_WINDOW_NAME)
                self._debug_window_open = False
        elif key in (ord('b'), ord('B')):
            enabled = not bool(self.desired_values.get('publish_bev_debug', True))
            self.desired_values['publish_bev_debug'] = enabled
            self.get_logger().info(
                f'BEV panel {"enabled" if enabled else "disabled"}'
            )
        elif key in (ord('q'), ord('Q'), 27):
            self._close_gui_windows()

    def destroy_node(self):
        try:
            if self._ui_ready and not self._window_closed:
                self._close_gui_windows(log_continues=False)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackTunerNode()
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
