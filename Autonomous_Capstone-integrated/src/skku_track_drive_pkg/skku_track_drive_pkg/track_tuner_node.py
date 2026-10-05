#!/usr/bin/env python3
"""OpenCV tuning panel backed by the ROS 2 parameter services."""

import os
import json
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_parameter_events,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

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

    tuner_window = TUNER_WINDOW_NAME
    debug_window = DEBUG_WINDOW_NAME
    control_windows = ()
    parking = False

    def __init__(self, node_name='track_tuner_node', extra_trackbars=None, parameter_root='track_controller_node', window_prefix='Track', parking=False, only_extra_controls=False):
        super().__init__(node_name)
        self.parameter_root = parameter_root
        self.tuner_window = window_prefix + ' Tuner'
        self.debug_window = window_prefix + ' Debug View'
        self.control_windows = []
        self.parameter_windows = {}
        self.mode_status = {}
        self.parking = parking
        self.mode_controls_separate = window_prefix != 'Track'
        self.declare_parameter('target_node', '/track_controller_node')
        self.declare_parameter('cmd_topic', '/topic_control_signal')
        self.declare_parameter('debug_topic', '/track_debug_image')
        self.declare_parameter(
            'save_path', str(Path('~/.config/autodrive/track_tuning.yaml').expanduser())
        )
        self.declare_parameter('loaded_config_path', '')
        self.declare_parameter('allow_speed_tuning', True)
        self.declare_parameter('fixed_speed', 250)
        self.declare_parameter('status_topic', '')

        self.target_node = str(self.get_parameter('target_node').value).rstrip('/')
        self.save_path = Path(str(self.get_parameter('save_path').value)).expanduser()
        self.loaded_config_path = str(self.get_parameter('loaded_config_path').value)
        self.allow_speed_tuning = bool(
            self.get_parameter('allow_speed_tuning').value
        )
        self.fixed_speed = int(self.get_parameter('fixed_speed').value)
        # All HighGUI windows in this process share waitKey. Forward W/S from
        # either tuner or debug view to the single calibration/arm gate. Never
        # publish actuator commands here. A token tags W at key-receipt time,
        # so queued keys from before a stop cannot become a fresh start event.
        key_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.drive_key_pub = self.create_publisher(String, 'vehicle/drive_key', key_qos)
        self.ui_heartbeat_pub = self.create_publisher(String, 'vehicle/tuner_heartbeat', key_qos)
        self._arm_challenge = ''
        self._last_ui_heartbeat = 0.0
        self.create_subscription(String, 'vehicle/arm_challenge', self.on_arm_challenge, state_qos)
        self.command_status = {}
        self.create_subscription(String, 'vehicle/command_status', self.on_serial_status, key_qos)
        self.trackbars = {} if parking or only_extra_controls else dict(TRACKBARS)
        self.trackbars.update(extra_trackbars or {})
        if not self.allow_speed_tuning:
            self.trackbars.pop('speed', None)
        persisted = {}
        if self.loaded_config_path and Path(self.loaded_config_path).is_file():
            with Path(self.loaded_config_path).open() as stream:
                persisted = (yaml.safe_load(stream) or {}).get(self.parameter_root, {}).get('ros__parameters', {})
        self.managed_parameters = tuple(dict.fromkeys([*self.trackbars, *persisted, *DISPLAY_PARAMETERS]))
        status_topic = str(self.get_parameter('status_topic').value)
        if status_topic:
            self.create_subscription(String, status_topic, self.on_mode_status, 1)

        self.get_client = self.create_client(
            GetParameters, f'{self.target_node}/get_parameters'
        )
        self.set_client = self.create_client(
            SetParametersAtomically, f'{self.target_node}/set_parameters_atomically'
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

    def on_mode_status(self, message):
        try:
            self.mode_status = json.loads(message.data)
        except (ValueError, TypeError):
            self.mode_status = {'status': message.data}

    def on_serial_status(self, message):
        try:
            self.command_status = json.loads(message.data)
        except (ValueError, TypeError):
            self.command_status = {}

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
            cv2.namedWindow(self.tuner_window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.tuner_window, 900, 720)
            self._initializing_ui = True
            for index, (name, (label, maximum, _decode, _encode)) in enumerate(self.trackbars.items()):
                window = self.tuner_window
                if extra := (index // 12 + int(self.mode_controls_separate)):
                    window = f'{self.tuner_window} Controls {extra}'
                    if window not in self.control_windows:
                        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
                        cv2.resizeWindow(window, 620, 650)
                        self.control_windows.append(window)
                self.parameter_windows[name] = window
                position = self._position_for(name, self.current_values[name])
                cv2.createTrackbar(
                    label,
                    window,
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
                self.parameter_windows[name],
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

        request = SetParametersAtomically.Request()
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
            failures = [] if response.result.successful else [response.result.reason]
        except Exception as exc:
            failures = [str(exc)]

        if failures:
            self.get_logger().error('Parameter update rejected: ' + '; '.join(failures))
            self.desired_values = dict(self.current_values)
            self._set_trackbar_positions()
            return

        self.current_values.update(updates)

    @staticmethod
    def write_yaml_atomic(path, values, root='track_controller_node'):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {root: {'ros__parameters': dict(values)}}
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
        if self._set_future is not None or any(self.current_values.get(k) != self.desired_values.get(k) for k in self.trackbars):
            self.get_logger().warn('Wait for parameter acceptance before P/save')
            return
        values = {name: self.current_values[name] for name in self.managed_parameters if name not in DISPLAY_PARAMETERS}
        # Preserve advanced values loaded from this mode, never another mode.
        if self.loaded_config_path and Path(self.loaded_config_path).is_file():
            with Path(self.loaded_config_path).open() as stream:
                previous = yaml.safe_load(stream) or {}
            saved = previous.get(self.parameter_root, {}).get('ros__parameters', {})
            values = {**saved, **values}
        if self.save_path.is_file():
            with self.save_path.open() as stream:
                document = yaml.safe_load(stream) or {}
            if self.parameter_root not in document:
                raise ValueError('Refusing to overwrite another mode configuration')
        self.write_yaml_atomic(self.save_path, values, root=self.parameter_root)
        self.get_logger().info(f'Saved tuning parameters to:\n{self.save_path}')
        for name, value in values.items():
            self.get_logger().info(f'  {name}: {value}')

    def _draw_status(self):
        lines = [
            'W: start   S: stop   P: save   D: debug   B: BEV   R: parking reset   Q/ESC: close',
            (
                f'Debug: {"ON" if self.desired_values.get("publish_debug") else "OFF"}   '
                f'BEV: {"ON" if self.desired_values.get("publish_bev_debug") else "OFF"}'
            ),
            (
                f'calibration PWM: {self.desired_values["pwm"]:+d}, steering: {self.desired_values.get("steering_step", 0):+d}'
                if 'pwm' in self.desired_values else
                f'parking PWM forward/reverse: {self.desired_values.get("forward_pwm")}/{self.desired_values.get("reverse_pwm")}'
                if self.parking else
                f'configured speed: {self.desired_values.get("speed", "-")}'
                if self.allow_speed_tuning else
                f'fixed speed: {self.fixed_speed} (longitudinal tuning disabled)'
            ),
            (
                f'controller request steering: {self.current_steering:+d}   '
                f'L/R speed: {self.current_left_speed}/{self.current_right_speed}'
            ),
            f'control update rate: {self.processing_fps:.1f} Hz',
            f'loaded: {self.loaded_config_path or "controller defaults"}',
            f'save to: {self.save_path}',
            f'Focus {self.tuner_window}, its control/debug windows or vehicle_start_gate for W/S.',
        ]
        if 'traffic_boxes' in self.mode_status:
            status = self.mode_status
            lines.extend([
                f"MISSION {status.get('state')} reason={status.get('reason')} stop={status.get('stop')}",
                f"path lane={status.get('path_lane')} target(next)={status.get('target_lane')} valid={status.get('path_valid')}",
                f"path blocked={status.get('path_blocked')} confirm={status.get('obstacle_confirm_count')}/{status.get('obstacle_confirm_required')}",
                f"alternative lane={status.get('alternate_lane')} visible={status.get('alternate_visible')} blocked={status.get('alternate_blocked')} valid={status.get('alternate_valid')}",
                f"alternative mask area={status.get('alternate_mask_area_px')}px (mask evidence, not a planned path)",
                f"LIGHT {status.get('traffic_color')} red_latched={status.get('red_latched')} selected_ratio={status.get('selected_color_ratio')} max_candidate_ratio={status.get('color_ratio')}",
                f"red={status.get('red_count')}/{status.get('red_confirm_required')} green={status.get('green_count')}/{status.get('green_confirm_required')}",
                f"system={status.get('system_state', 'NORMAL')} image_age_s={status.get('image_age_s', '-')}",
            ])
        else:
            lines.extend(f'{k}: {v}' for k, v in self.mode_status.items())
        for index, light in enumerate(self.mode_status.get('traffic_boxes', [])):
            ratios = light.get('ratios', {})
            lines.append(f"light#{index} box={light['box']} eligible={light['eligible']} used={light.get('used_for_color')} "
                         f"color={light.get('color', 'rejected')} R/G/Y=" +
                         '/'.join(f"{ratios.get(k, 0):.3f}" for k in ('Red', 'Green', 'Yellow')))
        applied = getattr(self, 'command_status', {})
        if applied:
            lines.append(f"SERIAL command PWM L/R={applied.get('left_pwm')}/{applied.get('right_pwm')} "
                         f"steer={applied.get('steering')} armed={applied.get('armed')}")
            lines.append(f"PERCEPTION age={applied.get('result_age_s')} "
                         f"stop={applied.get('stop_reason')}")
            lines.append('Serial values are software commands; physical motor PWM is unmeasured.')
        canvas = np.zeros((max(300, 45 + len(lines) * 33), 1100, 3), dtype=np.uint8)
        for index, line in enumerate(lines):
            # Preserve the filename and latest values when a long path exceeds
            # the window width. Every mode status field gets its own row.
            if cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, .62, 1)[0][0] > 1070:
                tail = line
                while cv2.getTextSize('... ' + tail, cv2.FONT_HERSHEY_SIMPLEX, .62, 1)[0][0] > 1070:
                    tail = tail[1:]
                line = '... ' + tail
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
        cv2.imshow(self.tuner_window, canvas)

    def _draw_debug(self):
        enabled = bool(self.desired_values.get('publish_debug', False))
        if not enabled:
            if self._debug_window_open:
                cv2.destroyWindow(self.debug_window)
                self._debug_window_open = False
            return
        if self._latest_debug_frame is None:
            return
        if not self._debug_window_open:
            cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                self.debug_window,
                self._latest_debug_frame.shape[1],
                self._latest_debug_frame.shape[0],
            )
            self._debug_window_open = True
        cv2.imshow(self.debug_window, self._latest_debug_frame)

    def _close_gui_windows(self, log_continues=True):
        self.forward_drive_key('s')
        if self._debug_window_open:
            cv2.destroyWindow(self.debug_window)
            self._debug_window_open = False
        if self._ui_ready and not self._window_closed:
            cv2.destroyWindow(self.tuner_window)
        for window in self.control_windows:
            cv2.destroyWindow(window)
        self.control_windows = []
        self._window_closed = True
        if log_continues:
            self.get_logger().info(
                'GUI windows closed; disarm requested. Camera/controller remain running; '
                'reopen the tuner to restore its heartbeat before starting.'
            )

    def on_arm_challenge(self, msg):
        self._arm_challenge = msg.data

    def forward_drive_key(self, key):
        self.get_logger().info(f'KEY received in tuner/debug: {key!r}; forwarding to arm gate')
        # Stops do not need a token; they are always permitted. W must carry the
        # challenge known when the physical key was received, never a later one.
        payload = f'w:{self._arm_challenge}' if key == 'w' else key
        self.drive_key_pub.publish(String(data=payload))

    def on_timer(self):
        # Keep this independent of controller parameter-service availability.
        # Stop heartbeat when the user closes the panel, and let the bridge
        # detect abrupt process death (including SIGKILL) without ROS cleanup.
        now = time.monotonic()
        if not self._window_closed and self._arm_challenge and now - self._last_ui_heartbeat >= 0.1:
            self.ui_heartbeat_pub.publish(String(data=self._arm_challenge))
            self._last_ui_heartbeat = now
        if self._get_future is None:
            if self.get_client.service_is_ready() and self.set_client.service_is_ready():
                self._request_initial_values()
            return
        if not self._ui_ready or self._window_closed:
            return

        if any(cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1 for window in [self.tuner_window, *self.control_windows]):
            self._close_gui_windows()
            return

        self._send_pending_parameters()
        self._draw_status()
        self._draw_debug()
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('w'), ord('W'), ord('s'), ord('S'), ord('x'), ord('X'), 32):
            self.forward_drive_key(chr(key).lower())
        elif key in (ord('r'), ord('R')) and self.parking:
            from std_srvs.srv import Trigger
            if not hasattr(self, 'reset_client'):
                self.reset_client = self.create_client(Trigger, f'{self.target_node}/reset')
            if self.reset_client.service_is_ready():
                self.reset_client.call_async(Trigger.Request())
        elif key in (ord('p'), ord('P')):
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
                cv2.destroyWindow(self.debug_window)
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
            # ROS might already be shut down. If publish is unavailable the
            # missing heartbeat still removes authorization at the bridge.
            if rclpy.ok(context=self.context):
                self.forward_drive_key('s')
            if self._ui_ready and not self._window_closed:
                if self._debug_window_open:
                    cv2.destroyWindow(self.debug_window)
                cv2.destroyWindow(self.tuner_window)
                for window in self.control_windows:
                    cv2.destroyWindow(window)
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
