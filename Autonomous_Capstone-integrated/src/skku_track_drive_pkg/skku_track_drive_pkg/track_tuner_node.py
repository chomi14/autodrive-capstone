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
from .bev_geometry import BEV_SOURCE_DEFAULTS
from .modes.bbox_mission.tuning_panel import MISSION_PRIMARY_CONTROLS, MISSION_WINDOW_NAME, MissionTuningPanel
from .stop_history import StopHistory


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
        'Lookahead Index (+3)', 96,
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
    'roi_cut': ('ROI Cut (px)', 470, lambda p: int(p), lambda v: int(v)),
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
TRACKBARS.update({
    name: (
        f'P{index // 2 + 1} {name.split("_")[2].upper()} {name[-1].upper()}'
        + (' (base)' if name.endswith('_y') and index < 4 else ''),
        639 if name.endswith('_x') else 479,
        lambda p: int(p), lambda v: int(v),
    )
    for index, name in enumerate(BEV_SOURCE_DEFAULTS)
})
DISPLAY_PARAMETERS = ('publish_debug', 'publish_bev_debug')


class TrackTunerNode(Node):
    """Change TrackController parameters; never publishes actuator commands."""

    tuner_window = TUNER_WINDOW_NAME
    debug_window = DEBUG_WINDOW_NAME
    control_windows = ()
    mission_panel = None
    parking = False

    def __init__(self, node_name='track_tuner_node', extra_trackbars=None, parameter_root='track_controller_node', window_prefix='Track', parking=False, only_extra_controls=False):
        super().__init__(node_name)
        self.parameter_root = parameter_root
        self.tuner_window = (
            TUNER_WINDOW_NAME if parameter_root == 'mission_controller_node' else window_prefix + ' Tuner'
        )
        self.debug_window = window_prefix + ' Debug View'
        self.control_windows = []
        self.parameter_windows = {}
        self._mission_detail_open = False
        self.mode_status = {}
        self.stop_history = StopHistory()
        self.parking = parking
        self.mode_controls_separate = self.tuner_window != TUNER_WINDOW_NAME
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
        self.capture_key_pub = self.create_publisher(String, 'capture/key', key_qos)
        self.capture_status = {}
        self.capture_status_time = float('-inf')
        self.create_subscription(String, 'capture/status', self.on_capture_status, state_qos)
        self._arm_challenge = ''
        self._last_ui_heartbeat = 0.0
        self.create_subscription(String, 'vehicle/arm_challenge', self.on_arm_challenge, state_qos)
        self.command_status = {}
        self.create_subscription(String, 'vehicle/command_status', self.on_serial_status, key_qos)
        self.trackbars = {} if parking or only_extra_controls else dict(TRACKBARS)
        self.trackbars.update(extra_trackbars or {})
        if self.parameter_root == 'mission_controller_node':
            self.trackbars = {
                **{name: self.trackbars[name] for name in MISSION_PRIMARY_CONTROLS if name in self.trackbars},
                **self.trackbars,
            }
        if not self.allow_speed_tuning:
            self.trackbars.pop('speed', None)
        persisted = {}
        if self.loaded_config_path and Path(self.loaded_config_path).is_file():
            with Path(self.loaded_config_path).open() as stream:
                persisted = (yaml.safe_load(stream) or {}).get(self.parameter_root, {}).get('ros__parameters', {})
        # Old mission tuning files use traffic_min_bottom_y. Querying this
        # removed parameter truncates GetParameters and leaves the tuner
        # without required values such as speed.
        if self.parameter_root == 'mission_controller_node' and 'traffic_min_bottom_y' in persisted:
            from .mode_parameters import MISSION
            persisted = dict(persisted)
            persisted.pop('traffic_min_bottom_y')
            persisted.setdefault('traffic_max_bottom_y', MISSION['traffic_max_bottom_y'][0])
        # The legacy car Y is relative to the ROI at first load. Save its
        # reference cut alongside it so changing/saving ROI cannot move the
        # full-BEV vehicle reference on the next launch.
        vehicle_reference = ('car_center_y', 'car_center_reference_roi_cut') if 'roi_cut' in self.trackbars else ()
        self.managed_parameters = tuple(dict.fromkeys([
            *self.trackbars, *persisted, *vehicle_reference, *DISPLAY_PARAMETERS,
        ]))
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
        self._last_debug_image_at = float('-inf')
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
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
        except (ValueError, TypeError):
            self.mode_status = {'status': message.data}
            return
        self.mode_status = status
        reason = (status.get('reason') or status.get('state') or 'stop requested') if status.get('stop') is True else None
        self._record_stop('MISSION', reason)

    def on_serial_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
        except (ValueError, TypeError):
            return
        self.command_status = status
        reason = None
        if status.get('armed') is False:
            reason = status.get('stop_reason') or 'disarmed (reason not reported)'
        elif (status.get('armed') is True and status.get('left_pwm') == 0
              and status.get('right_pwm') == 0):
            reason = status.get('stop_reason') or 'zero PWM while armed (reason not reported)'
        self._record_stop('SERIAL', reason)

    def _record_stop(self, source, reason):
        event = self.stop_history.observe(source, reason)
        if event is not None:
            self.get_logger().warn(f'STOP HISTORY [{event.timestamp}] {event.source}: {event.reason}')

    @staticmethod
    def _wrap_status_line(line, width, scale=0.52):
        """Keep the full reason visible, including long joined mission reasons."""
        rows = []
        remaining = line
        while remaining:
            end = len(remaining)
            while end > 1 and cv2.getTextSize(remaining[:end], cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] > width - 24:
                end -= 1
            if end < len(remaining):
                space = remaining.rfind(' ', 0, end + 1)
                if space > 0:
                    end = space
            rows.append(remaining[:end])
            remaining = remaining[end:].lstrip()
        return rows or ['']

    def _serial_status_lines(self, width=1100, scale=0.52):
        lines = []
        applied = self.command_status
        if applied:
            lines.append(f"SERIAL command PWM L/R={applied.get('left_pwm')}/{applied.get('right_pwm')} "
                         f"steer={applied.get('steering')} armed={applied.get('armed')}")
            age = applied.get('result_age_s')
            age_text = f'{age:.2f}s' if isinstance(age, (int, float)) else '-'
            lines.append(f"PERCEPTION age={age_text} stop={applied.get('stop_reason') or '-'}")
        lines.append('STOP HISTORY: latest 3, retained until tuner restart (newest first)')
        if not self.stop_history.events:
            lines.append('No stop observed yet.')
        for event in self.stop_history.events:
            lines.append(f'[{event.timestamp}] {event.source}: {event.reason}')
        if applied:
            lines.append('Serial values are software commands; physical motor PWM is unmeasured.')
        return [row for line in lines for row in self._wrap_status_line(line, width, scale)]

    def on_debug_image(self, message):
        try:
            self._latest_debug_frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
            self._last_debug_image_at = time.monotonic()
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
        for name, value in changed.items():
            if self.desired_values.get(name) == self.current_values.get(name):
                self.desired_values[name] = value
        self.current_values.update(changed)
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
            regular_index = 0
            advanced_index = 0
            mission = self.parameter_root == 'mission_controller_node'
            if mission:
                self.mission_panel = MissionTuningPanel(self.trackbars, self._on_trackbar, self._handle_key)
                cv2.namedWindow(MISSION_WINDOW_NAME, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
                self.control_windows.append(MISSION_WINDOW_NAME)
                self._draw_mission_status()
                cv2.setMouseCallback(MISSION_WINDOW_NAME, self.mission_panel.on_mouse)
            for name, (label, maximum, _decode, _encode) in self.trackbars.items():
                window = self.tuner_window
                if name in BEV_SOURCE_DEFAULTS:
                    window = f'{self.tuner_window} BEV Source Points'
                elif mission and name in MISSION_PRIMARY_CONTROLS:
                    continue
                elif mission and name not in TRACKBARS:
                    window = f'{self.tuner_window} Advanced {advanced_index // 12 + 1}'
                    advanced_index += 1
                else:
                    if extra := (regular_index // 12 + int(self.mode_controls_separate)):
                        window = f'{self.tuner_window} Controls {extra}'
                    regular_index += 1
                if window != self.tuner_window and window not in self.control_windows:
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
        for name, window in self.parameter_windows.items():
            value = self.desired_values[name]
            cv2.setTrackbarPos(
                self.trackbars[name][0],
                window,
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
            if self.parameter_root == 'mission_controller_node' and 'traffic_min_bottom_y' in saved:
                from .mode_parameters import MISSION
                saved = dict(saved)
                saved.pop('traffic_min_bottom_y')
                saved.setdefault('traffic_max_bottom_y', MISSION['traffic_max_bottom_y'][0])
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
            ('W: start   S: stop   P: save   D: debug   B: BEV   '
             + ('R: parking reset' if self.parking else 'R: auto capture   C: capture')
             + '   Q/ESC: close'),
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
            *self._serial_status_lines(scale=0.62),
            f'loaded: {self.loaded_config_path or "controller defaults"}',
            f'save to: {self.save_path}',
            f'Focus {self.tuner_window}, its control/debug windows or vehicle_start_gate for W/S.',
        ]
        if not self.parking:
            if time.monotonic() - self.capture_status_time <= 2.0:
                status = self.capture_status
                state = 'OFF'
                if status.get('enabled'):
                    state = ('SAVING' if status.get('moving') and status.get('image_fresh')
                             else 'ON (waiting for motion/raw image)')
                lines.append(f'RAW CAPTURE: {state}  interval={status.get("interval")}s  saved={status.get("count")}')
                lines.append(f'dataset: {status.get("output_dir")}')
                if status.get('error'):
                    lines.append(f'CAPTURE ERROR: {status["error"]}')
            else:
                lines.append('RAW CAPTURE unavailable: launch with capture:=true')
        if self.parameter_root != 'mission_controller_node':
            lines.extend(f'{k}: {v}' for k, v in self.mode_status.items())
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
        if MISSION_WINDOW_NAME in self.control_windows:
            self._draw_mission_status()
        source_window = f'{self.tuner_window} BEV Source Points'
        if source_window in self.control_windows:
            point_canvas = np.zeros((150, 620, 3), dtype=np.uint8)
            for index, line in enumerate((
                'Camera points: P1 TL, P2 TR, P3 BR, P4 BL',
                'X: left/right. Y: up/down. Change pairs for width/tilt.',
                'Top Y is base Y + BEV Top Shift.',
                'Orange outline/coordinates on camera show applied points.',
                'P: save accepted points. Invalid shapes are rejected.',
            )):
                cv2.putText(point_canvas, line, (10, 24 + index * 26),
                            cv2.FONT_HERSHEY_SIMPLEX, .48, (220, 220, 220), 1, cv2.LINE_AA)
            cv2.imshow(source_window, point_canvas)

    def _draw_mission_status(self):
        frame = None
        message = 'Debug feed is off. Press D to show it.'
        if self.desired_values.get('publish_debug'):
            message = 'Waiting for a fresh debug frame'
            if time.monotonic() - self._last_debug_image_at <= 1.0:
                frame = self._latest_debug_frame
        syncing = any(self.desired_values[name] != self.current_values[name]
                      for name in MISSION_PRIMARY_CONTROLS)
        canvas = self.mission_panel.render(self.desired_values, self.mode_status,
                                           self.command_status, frame, message, syncing)
        cv2.imshow(MISSION_WINDOW_NAME, canvas)

    def _draw_debug(self):
        enabled = bool(self.desired_values.get('publish_debug', False))
        if self.mission_panel is not None:
            enabled = enabled and self._mission_detail_open
        if not enabled:
            if self._debug_window_open:
                cv2.destroyWindow(self.debug_window)
                self._debug_window_open = False
            return
        if self._latest_debug_frame is None:
            return
        frame = self._latest_debug_frame
        lines = self._serial_status_lines(frame.shape[1])
        footer = np.zeros((12 + len(lines) * 25, frame.shape[1], 3), dtype=np.uint8)
        for index, line in enumerate(lines):
            color = (0, 200, 255) if line.startswith(('STOP HISTORY', '[')) else (220, 220, 220)
            cv2.putText(footer, line, (12, 22 + index * 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, color, 1, cv2.LINE_AA)
        display = np.vstack((frame, footer))
        if not self._debug_window_open:
            cv2.namedWindow(self.debug_window, cv2.WINDOW_NORMAL)
            scale = min(1., 1280 / display.shape[1], 900 / display.shape[0])
            cv2.resizeWindow(
                self.debug_window,
                max(1, round(display.shape[1] * scale)),
                max(1, round(display.shape[0] * scale)),
            )
            self._debug_window_open = True
        cv2.imshow(self.debug_window, display)

    def _close_gui_windows(self, log_continues=True):
        windows = list(self.control_windows)
        if self._ui_ready and not self._window_closed:
            windows.append(self.tuner_window)
        if self._debug_window_open:
            windows.append(self.debug_window)
        self._window_closed = True
        self._debug_window_open = False
        self.control_windows = []
        self.forward_drive_key('s')
        for window in windows:
            try:
                cv2.destroyWindow(window)
            except cv2.error:
                pass  # The window manager may have already removed it.
        if log_continues:
            self.get_logger().info(
                'GUI windows closed; disarm requested. Camera/controller remain running; '
                'reopen the tuner to restore its heartbeat before starting.'
            )

    def on_arm_challenge(self, msg):
        self._arm_challenge = msg.data

    def on_capture_status(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                return
        except (ValueError, TypeError):
            return
        self.capture_status = status
        self.capture_status_time = time.monotonic()

    def forward_capture_key(self, key):
        if time.monotonic() - self.capture_status_time > 2.0:
            self.get_logger().warn('Raw capture unavailable. Launch with capture:=true.')
            return
        self.capture_key_pub.publish(String(data=key))

    def forward_drive_key(self, key):
        self.get_logger().info(f'KEY received in tuner/debug: {key!r}; forwarding to arm gate')
        # Stops do not need a token; they are always permitted. W must carry the
        # challenge known when the physical key was received, never a later one.
        payload = f'w:{self._arm_challenge}' if key == 'w' else key
        self.drive_key_pub.publish(String(data=payload))

    def request_lane_toggle(self):
        """Change the mission target only; never arm or publish motor commands."""
        from std_srvs.srv import Trigger
        if not hasattr(self, '_lane_toggle_client'):
            self._lane_toggle_client = self.create_client(Trigger, f'{self.target_node}/toggle_target_lane')
        if not self._lane_toggle_client.service_is_ready():
            self.get_logger().warn('Target lane toggle unavailable: bbox mission controller is not ready')
            return
        if getattr(self, '_lane_toggle_future', None) is not None:
            return
        self._lane_toggle_future = self._lane_toggle_client.call_async(Trigger.Request())
        self._lane_toggle_future.add_done_callback(self._on_lane_toggled)

    def _on_lane_toggled(self, future):
        self._lane_toggle_future = None
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().warn(response.message)
        except Exception as exc:
            self.get_logger().error(f'Target lane toggle failed: {exc}')

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

        try:
            windows_visible = all(cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) >= 1
                                  for window in [self.tuner_window, *self.control_windows])
        except cv2.error:
            windows_visible = False
        if not windows_visible:
            self._close_gui_windows()
            return

        self._send_pending_parameters()
        self._draw_status()
        self._draw_debug()
        self._handle_key(cv2.waitKey(1) & 0xFF)

    def _handle_key(self, key):
        if key in (ord('w'), ord('W'), ord('s'), ord('S'), ord('x'), ord('X'), 32):
            self.forward_drive_key(chr(key).lower())
        elif key in (ord('l'), ord('L')) and self.parameter_root == 'mission_controller_node':
            self.request_lane_toggle()
        elif key in (ord('r'), ord('R')) and self.parking:
            from std_srvs.srv import Trigger
            if not hasattr(self, 'reset_client'):
                self.reset_client = self.create_client(Trigger, f'{self.target_node}/reset')
            if self.reset_client.service_is_ready():
                self.reset_client.call_async(Trigger.Request())
        elif key in (ord('r'), ord('R'), ord('c'), ord('C')) and not self.parking:
            self.forward_capture_key(chr(key).lower())
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
        elif key in (ord('v'), ord('V')) and self.parameter_root == 'mission_controller_node':
            self._mission_detail_open = not self._mission_detail_open
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
