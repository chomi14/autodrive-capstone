#!/usr/bin/env python3
import json
import re
import signal
import time
import uuid

import rclpy
import serial
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from std_msgs.msg import Bool, String
from interfaces_pkg.msg import MotionCommand
from rcl_interfaces.msg import ParameterDescriptor
from .perception_delay_guard import PerceptionDelayGuard


_CONFIG_RE = re.compile(
    r'CONFIG,left=(-?\d+),right=(-?\d+),center=(-?\d+),max_step=(\d+)'
)
_CAL_RESULT_RE = re.compile(
    r'CAL_RESULT,left=(-?\d+),right=(-?\d+),center=(-?\d+),span=(\d+)'
)
_CAL_APPLIED_RE = re.compile(
    r'CAL_APPLIED,left=(-?\d+),right=(-?\d+),center=(-?\d+)'
)
_STATUS_POT_RE = re.compile(r'STATUS,pot=(\d+)')
_CAL_ERROR_RE = re.compile(r'CAL_ERROR,reason=(.+)')

# Both supported sketches stop PWM after 500 ms with no valid serial frame.
# This is communication loss, separate from camera/controller processing age.
FIRMWARE_COMMAND_TIMEOUT_SEC = 0.5


def encode_command(steering: int, left_speed: int, right_speed: int) -> bytes:
    return f's{int(steering)}l{int(left_speed)}r{int(right_speed)}\n'.encode('ascii')


class SerialSenderNode(Node):
    """ROS2 <-> Arduino bridge with startup steering calibration and an arm gate.

    Startup sequence when auto_calibrate=True:
      1. Open Arduino and force STOP.
      2. Query the firmware's configured baseline steering limits.
      3. Ask Arduino to measure physical LEFT/RIGHT limits and return to center.
      4. Compare the measured limits against the configured baseline.
      5. If the difference is small enough, send the measured pair back to the
         Arduino as the runtime calibration for this run only.
      6. Publish /vehicle/calibration_ready=True.

    Motion is always blocked until BOTH calibration_ready and armed are True.
    This means track/mission nodes may already be publishing commands safely
    while the car waits for the operator to press W in drive_arm_node.
    """

    def __init__(self):
        super().__init__('serial_sender_node_v2')

        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', 'topic_control_signal')
        self.declare_parameter('arm_topic', 'vehicle/armed')
        self.declare_parameter('ready_topic', 'vehicle/calibration_ready')
        self.declare_parameter('status_topic', 'vehicle/calibration_status')
        self.declare_parameter('timeout', 0.05)
        self.declare_parameter('auto_calibrate', False)
        self.declare_parameter('calibration_tolerance', 35)
        self.declare_parameter('min_calibration_span', 80)
        self.declare_parameter('calibration_timeout', 22.0)
        self.declare_parameter('query_retry_sec', 1.0)
        # Canonical launches use controller liveness instead of inference age.
        # Positive command_timeout bounds the health lease; zero uses the
        # existing UI timeout for a finite lease, without checking image age.
        # Legacy producers without independent health keep command-age gating
        # and do not replay. No numerical timeout defaults are changed.
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('ui_timeout', 0.75)
        self.declare_parameter('require_tuner_heartbeat', False)
        self.declare_parameter('require_controller_heartbeat', False)
        restart_policy = ParameterDescriptor(read_only=True,
            description='Measured perception delay policy; set through launch and restart')
        self.declare_parameter('require_perception_progress', False, descriptor=restart_policy)
        self.declare_parameter('perception_stop_s', 0.0, descriptor=restart_policy)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.topic = str(self.get_parameter('topic').value)
        self.arm_topic = str(self.get_parameter('arm_topic').value)
        self.ready_topic = str(self.get_parameter('ready_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        timeout = float(self.get_parameter('timeout').value)
        self.auto_calibrate = bool(self.get_parameter('auto_calibrate').value)
        self.cal_tolerance = int(self.get_parameter('calibration_tolerance').value)
        self.min_cal_span = int(self.get_parameter('min_calibration_span').value)
        self.cal_timeout = float(self.get_parameter('calibration_timeout').value)
        self.query_retry_sec = float(self.get_parameter('query_retry_sec').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.ui_timeout = float(self.get_parameter('ui_timeout').value)
        self.require_tuner_heartbeat = bool(self.get_parameter('require_tuner_heartbeat').value)
        self.require_controller_heartbeat = bool(self.get_parameter('require_controller_heartbeat').value)
        require_progress = bool(self.get_parameter('require_perception_progress').value)
        if require_progress and not self.require_controller_heartbeat:
            raise ValueError('perception progress requires independent controller heartbeat')
        self.delay_guard = PerceptionDelayGuard(
            float(self.get_parameter('perception_stop_s').value)) if require_progress else None
        self.stop_reason = 'startup'
        self.controller_timeout = self.command_timeout or self.ui_timeout
        self.last_controller_time = None
        self.controller_instance = None
        if not (0 <= self.command_timeout < float('inf') and
                0 < self.ui_timeout < float('inf')):
            raise ValueError('command_timeout must be finite and nonnegative; ui_timeout must be finite and positive')
        self.last_command_time = None
        self.last_motion_write_time = None
        self.last_ui_time = None
        self.last_tuner_time = None
        self.serial_fault = False
        self._closed = False
        self._stop_logged = False
        self.challenge = uuid.uuid4().hex
        self.started_at = time.monotonic()

        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.0,
            write_timeout=timeout,
            exclusive=True,
        )
        time.sleep(1.2)  # Mega 2560 generally resets when the serial port opens.

        self._rx = ''
        self.last_pot = None
        self.config_left = None
        self.config_right = None
        self.config_center = None
        self.max_step = 7

        self.calibration_ready = False
        self.armed = False
        self.calibration_state = 'STARTING'
        self.calibration_deadline = 0.0
        self.last_query_time = 0.0
        self.last_cmd = MotionCommand()
        self.last_cmd.steering = 0
        self.last_cmd.left_speed = 0
        self.last_cmd.right_speed = 0
        self.last_applied_cmd = MotionCommand()

        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub = self.create_subscription(MotionCommand, self.topic, self.on_cmd, cmd_qos)
        self.arm_sub = self.create_subscription(Bool, self.arm_topic, self.on_arm, latched_qos)
        # Bool True remains a UI status only. It has no event identity, so an
        # old transient-local True must never authorize motion after a stop.
        # Only an explicit W request carrying the current single-run challenge
        # can arm. Every disarm changes the challenge, invalidating queued W's.
        self.request_sub = self.create_subscription(String, 'vehicle/arm_request', self.on_arm_request, cmd_qos)
        self.ui_sub = self.create_subscription(String, 'vehicle/ui_heartbeat', self.on_ui_heartbeat, cmd_qos)
        self.tuner_sub = self.create_subscription(String, 'vehicle/tuner_heartbeat', self.on_tuner_heartbeat, cmd_qos)
        self.controller_sub = self.create_subscription(String, 'vehicle/controller_heartbeat', self.on_controller_heartbeat, cmd_qos)
        self.challenge_pub = self.create_publisher(String, 'vehicle/arm_challenge', latched_qos)
        self.state_pub = self.create_publisher(Bool, 'vehicle/drive_state', latched_qos)
        self.ready_pub = self.create_publisher(Bool, self.ready_topic, latched_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)
        self.command_status_pub = self.create_publisher(String, 'vehicle/command_status', cmd_qos)

        self.rx_timer = self.create_timer(0.03, self.poll_serial)
        self.state_timer = self.create_timer(0.20, self.update_startup_state)
        self.keep_stop_timer = self.create_timer(0.05, self.enforce_safe_state)

        self.publish_gate_state()
        if self.send_stop(force=True):
            self.start_calibration_sequence()
        else:
            # A failed initial STOP must not be followed by a misleading READY
            # status, even when automatic calibration is disabled.
            self.calibration_state = 'FAILED'
            self.publish_ready(False)
            self.publish_status('LOCKED: initial STOP write failed; repair serial connection and restart')

        self.get_logger().info(
            f'Arduino serial opened: {self.port} @ {self.baud}; '
            f'auto_calibrate={self.auto_calibrate}, tolerance=±{self.cal_tolerance} ADC'
        )
        if self.command_timeout == 0:
            self.get_logger().warn(
                f'Inference command age checking disabled; controller health required='
                f'{self.require_controller_heartbeat}, health lease={self.controller_timeout}s. '
                'S/Ctrl+C, UI and serial/firmware watchdog stops remain enabled.'
            )

    # ------------------------------------------------------------------
    # ROS state publishers
    # ------------------------------------------------------------------
    def publish_ready(self, ready: bool):
        self.calibration_ready = bool(ready)
        msg = Bool()
        msg.data = self.calibration_ready
        self.ready_pub.publish(msg)

    def publish_status(self, text: str):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)
        self.get_logger().info(str(text))

    def start_calibration_sequence(self):
        """Select startup policy without making manual-baseline mode wait forever.

        ``auto_calibrate=False`` means that the operator has chosen to trust the
        steering limits already installed in the Arduino firmware.  Requiring a
        CONFIG reply in that mode made otherwise compatible/legacy firmware stay
        locked in WAIT_CONFIG, so the arm UI could never become READY.

        The CONFIG handshake remains mandatory before the explicit automatic
        endpoint measurement because those values are needed to validate it.
        """
        if not self.auto_calibrate:
            self.calibration_state = 'READY'
            self.calibration_deadline = 0.0
            self.publish_ready(True)
            self.publish_status(
                'READY: automatic calibration disabled; trusting the steering '
                'baseline installed in firmware. Press W to arm/start.'
            )
            return

        self.calibration_state = 'WAIT_CONFIG'
        self.calibration_deadline = time.monotonic() + self.cal_timeout
        self.publish_ready(False)
        self.publish_status('WAIT_CONFIG: querying Arduino baseline')
        self.send_ascii('?')
        self.last_query_time = time.monotonic()

    # ------------------------------------------------------------------
    # Serial helpers
    # ------------------------------------------------------------------
    def send_ascii(self, line: str):
        return self.write_serial((line.rstrip('\n') + '\n').encode('ascii'))

    def send_stop(self, force=False):
        # Zero motion frames keep an explicitly requested calibration alive.
        # S, faults and shutdown always use X to cancel it and zero ALL PWM;
        # s0l0r0 alone would still ask normal steering control to center.
        payload = (encode_command(0, 0, 0)
                   if not force and self.calibration_state in ('WAIT_MEASURE', 'WAIT_APPLY')
                   else b'X\n')
        success = self.write_serial(payload)
        if payload == b'X\n' and (force or not self._stop_logged):
            self.get_logger().warn(
                f'STOP frame X: {"written to serial" if success else "write failed"}; '
                'motor PWM not measured/confirmed')
            self._stop_logged = True
        return success

    def write_serial(self, payload):
        try:
            if self.ser is None or not self.ser.is_open:
                raise serial.SerialException('serial port is closed')
            if self.ser.write(payload) != len(payload):
                raise serial.SerialException('short serial write')
            return True
        except (serial.SerialException, OSError) as exc:
            self.on_serial_fault(str(exc))
            return False

    def on_serial_fault(self, reason):
        # Never recursively try writes from the write-error path. The board's
        # watchdog is the last line of defense if USB cannot deliver X.
        if not self.serial_fault:
            self.get_logger().error(f'Serial fault: {reason}; disarming, new W required')
            self.serial_fault = True
            self.disarm('serial fault', transmit=False)
        self.publish_ready(False)

    def poll_serial(self):
        if self.serial_fault:
            return
        try:
            n = self.ser.in_waiting
            if n <= 0:
                return
            self._rx += self.ser.read(n).decode('ascii', errors='ignore')
            lines = self._rx.replace('\r', '\n').split('\n')
            self._rx = lines.pop()
            for line in lines:
                line = line.strip()
                if line:
                    self.handle_serial_line(line)
            self._rx = self._rx[-4000:]
        except (serial.SerialException, OSError) as exc:
            self.on_serial_fault(str(exc))
            self.send_stop(force=True)

    def handle_serial_line(self, line: str):
        pot_match = _STATUS_POT_RE.search(line)
        if pot_match:
            self.last_pot = int(pot_match.group(1))

        m = _CONFIG_RE.fullmatch(line)
        if m:
            if self.calibration_state != 'WAIT_CONFIG':
                return
            self.config_left = int(m.group(1))
            self.config_right = int(m.group(2))
            self.config_center = int(m.group(3))
            self.max_step = int(m.group(4))
            if not (0 <= min(self.config_left, self.config_right)
                    < self.config_center < max(self.config_left, self.config_right) <= 1023
                    and abs(self.config_left - self.config_right) >= self.min_cal_span
                    and self.max_step == 7):
                self.fail_calibration('Invalid firmware steering baseline')
                return
            self.publish_status(
                f'BASELINE: left={self.config_left}, right={self.config_right}, '
                f'center={self.config_center}, max_step={self.max_step}'
            )

            if self.auto_calibrate:
                self.calibration_state = 'WAIT_MEASURE'
                self.calibration_deadline = time.monotonic() + self.cal_timeout
                self.publish_status('CALIBRATING: measuring physical LEFT/RIGHT limits; drive wheels locked')
                self.send_ascii('C')
            else:
                self.calibration_state = 'READY'
                self.publish_ready(True)
                self.publish_status('READY: auto calibration disabled; using firmware baseline')
            return

        m = _CAL_RESULT_RE.fullmatch(line)
        if m:
            if self.calibration_state != 'WAIT_MEASURE':
                return
            measured_left = int(m.group(1))
            measured_right = int(m.group(2))
            measured_center = int(m.group(3))
            measured_span = int(m.group(4))
            self.evaluate_calibration(
                measured_left, measured_right, measured_center, measured_span
            )
            return

        m = _CAL_APPLIED_RE.fullmatch(line)
        if m:
            if self.calibration_state != 'WAIT_APPLY':
                return
            applied_left = int(m.group(1))
            applied_right = int(m.group(2))
            applied_center = int(m.group(3))
            self.config_left = applied_left
            self.config_right = applied_right
            self.config_center = applied_center
            self.calibration_state = 'READY'
            self.publish_ready(True)
            self.publish_status(
                f'READY: runtime steering correction applied '
                f'L={applied_left}, R={applied_right}, C={applied_center}. '
                f'Press W to arm/start.'
            )
            return

        m = _CAL_ERROR_RE.fullmatch(line)
        if m:
            self.fail_calibration(f'Arduino calibration error: {m.group(1)}')
            return

        if line.startswith('CAL_') or line.startswith('ARDUINO_'):
            self.get_logger().info(f'[Arduino] {line}')

    # ------------------------------------------------------------------
    # Calibration decision on the PC side
    # ------------------------------------------------------------------
    def evaluate_calibration(self, left: int, right: int, center: int, span: int):
        if self.config_left is None or self.config_right is None:
            self.fail_calibration('Measured limits arrived before baseline CONFIG')
            return

        if not (0 <= min(left, right) < center < max(left, right) <= 1023
                and span == abs(left - right)
                and (right - left) * (self.config_right - self.config_left) > 0):
            self.fail_calibration('Invalid measured steering calibration')
            return

        ref_span = abs(self.config_left - self.config_right)
        delta_left = left - self.config_left
        delta_right = right - self.config_right
        span_delta = span - ref_span

        self.publish_status(
            f'CAL_RESULT: measured L={left} ({delta_left:+d}), '
            f'R={right} ({delta_right:+d}), C={center}, span={span} ({span_delta:+d})'
        )

        if span < self.min_cal_span:
            self.fail_calibration(
                f'Calibration rejected: span {span} < minimum {self.min_cal_span}'
            )
            return

        # A small whole-potentiometer shift is exactly what runtime correction is
        # meant to handle.  A large difference indicates wiring/mechanics/baseline
        # mismatch and should NOT be silently accepted.
        if abs(delta_left) > self.cal_tolerance or abs(delta_right) > self.cal_tolerance:
            self.fail_calibration(
                'Calibration rejected: endpoint shift is larger than allowed '
                f'(L {delta_left:+d}, R {delta_right:+d}, tolerance ±{self.cal_tolerance}). '
                'Run steering_limit_calibration.ino and update DEFAULT_LEFT/DEFAULT_RIGHT.'
            )
            return

        self.calibration_state = 'WAIT_APPLY'
        self.calibration_deadline = time.monotonic() + 4.0
        self.publish_status(
            f'CAL_ACCEPTED: sending runtime correction to Arduino: L={left}, R={right}'
        )
        self.send_ascii(f'K{left},{right}')

    def fail_calibration(self, reason: str):
        self.calibration_state = 'FAILED'
        self.publish_ready(False)
        self.disarm('calibration failed')
        self.publish_status(f'LOCKED: {reason}')

    def update_startup_state(self):
        now = time.monotonic()

        if self.calibration_state == 'WAIT_CONFIG':
            if now > self.calibration_deadline:
                self.fail_calibration('No CONFIG response from Arduino')
                return
            if now - self.last_query_time >= self.query_retry_sec:
                self.send_ascii('?')
                self.last_query_time = now

        elif self.calibration_state in ('WAIT_MEASURE', 'WAIT_APPLY'):
            if now > self.calibration_deadline:
                self.fail_calibration(f'Timeout in state {self.calibration_state}')

    # ------------------------------------------------------------------
    # Arm + motion gating
    # ------------------------------------------------------------------
    def on_arm(self, msg: Bool):
        if not msg.data:
            self.disarm('operator/UI disarm')

    def publish_gate_state(self):
        # Publishing is advisory; shutdown safety never depends on ROS being
        # alive. Direct serial stop is attempted before the port is closed.
        if rclpy.ok(context=self.context):
            self.challenge_pub.publish(String(data=self.challenge))
            self.state_pub.publish(Bool(data=self.armed))

    def disarm(self, reason, transmit=True):
        self.armed = False
        self.stop_reason = reason
        if self.delay_guard is not None:
            self.delay_guard.reset_run()
        self.last_applied_cmd = MotionCommand(steering=self.last_applied_cmd.steering)
        # Discard authorization to reuse the previous run's command. Even with
        # the age timeout disabled, initial/re-arming needs at least one valid
        # command received after this stop, plus a new explicit W event.
        self.last_command_time = None
        self.last_motion_write_time = None
        self.challenge = uuid.uuid4().hex
        self.last_ui_time = None
        self.last_tuner_time = None
        self.last_controller_time = None
        self.started_at = time.monotonic()
        abort_calibration = self.calibration_state in ('WAIT_MEASURE', 'WAIT_APPLY')
        if abort_calibration:
            self.calibration_state = 'FAILED'
            self.calibration_ready = False
        self.get_logger().warn(f'ARM state=False: {reason}; fresh W required')
        # Transmit first: a ROS publish error must not prevent the stop frame.
        if transmit:
            self.send_stop(force=True)
        if abort_calibration and rclpy.ok(context=self.context):
            self.publish_ready(False)
        self.publish_gate_state()

    def on_ui_heartbeat(self, msg):
        self.check_watchdogs(time.monotonic())
        if msg.data == self.challenge:
            self.last_ui_time = time.monotonic()

    def on_tuner_heartbeat(self, msg):
        self.check_watchdogs(time.monotonic())
        if msg.data == self.challenge:
            self.last_tuner_time = time.monotonic()

    def ui_alive(self, now):
        return (self.last_ui_time is not None and now - self.last_ui_time < self.ui_timeout
                and (not self.require_tuner_heartbeat or
                     (self.last_tuner_time is not None and now - self.last_tuner_time < self.ui_timeout)))

    def on_controller_heartbeat(self, msg):
        # Check BEFORE accepting late health: expiry/restart cannot revive W.
        now = time.monotonic()
        self.check_watchdogs(now)
        if not self.require_controller_heartbeat:
            return
        try:
            health = json.loads(msg.data)
            token, instance, active = health['challenge'], health['instance'], health['active']
        except (ValueError, TypeError, KeyError):
            return
        if token != self.challenge or not isinstance(instance, str) or not instance or type(active) is not bool:
            return
        if not active:
            if instance == self.controller_instance:
                self.disarm('controller shutdown')
            return
        if self.controller_instance is not None and instance != self.controller_instance:
            self.controller_instance = instance
            if self.delay_guard is not None:
                self.delay_guard.reset_owner()
            self.disarm('controller process replaced')
            return
        self.controller_instance = instance
        self.last_controller_time = now
        if self.delay_guard is not None:
            self.delay_guard.observe(health.get('perception'), now)

    def controller_alive(self, now):
        return (self.last_controller_time is not None and
                now - self.last_controller_time < self.controller_timeout)

    def command_alive(self, now):
        # Zero disables command AGE checking, not the first-command condition.
        # The initialized all-zero object is not a received control command.
        return (self.last_command_time is not None and
                (self.controller_alive(now) if self.require_controller_heartbeat else
                 self.command_timeout == 0 or
                 now - self.last_command_time < self.command_timeout))

    def on_arm_request(self, msg):
        now = time.monotonic()
        self.check_watchdogs(now)
        if (msg.data != self.challenge or self.serial_fault or
                not self.calibration_ready or not self.ui_alive(now) or
                not self.command_alive(now) or
                (self.delay_guard is not None and not self.delay_guard.ready(now))):
            self.get_logger().warn('W request rejected: stale token or calibration/command/UI not ready; press W again')
            return
        self.armed = True
        self.stop_reason = ''
        # Give the first motion write one firmware watchdog interval. Later
        # successful writes update this timestamp. If the bridge's own event
        # loop is suspended past that interval, a resumed timer must disarm
        # before repeating anything: the board may already have stopped PWM.
        self.last_motion_write_time = now
        self._stop_logged = False
        self.get_logger().warn('ARM state=True: fresh W accepted; motion commands allowed')
        self.publish_gate_state()

    def on_cmd(self, msg: MotionCommand):
        # Invalid input neither drives motors nor feeds the command watchdog.
        if not (-7 <= msg.steering <= 7 and -255 <= msg.left_speed <= 255
                and -255 <= msg.right_speed <= 255):
            return
        # Check expiry BEFORE recording this arrival: recovery must not erase
        # a timeout that happened while the executor was busy or suspended.
        self.check_watchdogs(time.monotonic())
        self.last_command_time = time.monotonic()
        self.last_cmd = msg
        if not self.require_controller_heartbeat:
            self.write_gated_command(msg)
        # Canonical motion is sent by the 50 ms timer, not by inference FPS.
        # An intentional zero command can take effect immediately; it stays
        # cached and cannot be replaced by an earlier motion during a delay.
        elif msg.left_speed == 0 and msg.right_speed == 0:
            self.write_gated_command(self.effective_command(msg, time.monotonic()))

    def effective_command(self, msg, now):
        if self.delay_guard is None or not self.armed:
            return msg
        steering, left, right = self.delay_guard.output(msg.steering, msg.left_speed, msg.right_speed, now)
        return MotionCommand(steering=steering, left_speed=left, right_speed=right)

    def publish_command_status(self, now):
        msg = self.last_applied_cmd
        self.command_status_pub.publish(String(data=json.dumps({
            'armed': self.armed, 'steering': msg.steering,
            'left_pwm': msg.left_speed, 'right_pwm': msg.right_speed,
            'stop_reason': self.stop_reason,
            **(self.delay_guard.status(now) if self.delay_guard is not None else {})})))

    def write_gated_command(self, msg: MotionCommand):
        if self.calibration_ready and self.armed:
            steering = int(msg.steering)
            left_speed = int(msg.left_speed)
            right_speed = int(msg.right_speed)
        else:
            self.send_stop()
            return

        if self.write_serial(encode_command(steering, left_speed, right_speed)):
            self.last_motion_write_time = time.monotonic()
            self.last_applied_cmd = msg

    def check_watchdogs(self, now):
        if self.armed:
            if not self.command_alive(now):
                self.disarm('controller health timeout' if self.require_controller_heartbeat else 'control command timeout')
            elif (self.last_motion_write_time is not None and
                  now - self.last_motion_write_time >= FIRMWARE_COMMAND_TIMEOUT_SEC):
                self.disarm('serial transmission gap exceeded firmware watchdog; fresh W required')
            elif not self.ui_alive(now):
                self.disarm('UI heartbeat timeout')
            elif self.delay_guard is not None and self.delay_guard.expired(now):
                # Final speed-zero frame preserves the last steering step.
                # Then X turns every PWM off and latches a fresh-W requirement.
                reason = self.delay_guard.reason(now)
                self.write_gated_command(self.effective_command(self.last_cmd, now))
                if self.armed:
                    self.disarm(f'perception stale: {reason}')
        elif (self.calibration_state in ('WAIT_MEASURE', 'WAIT_APPLY') and
              now - self.started_at >= self.ui_timeout and not self.ui_alive(now)):
            self.disarm('UI heartbeat timeout during calibration')

    def enforce_safe_state(self):
        # If no controller command is arriving, still actively hold STOP while
        # calibrating/disarmed so a stale Arduino command cannot remain active.
        self.check_watchdogs(time.monotonic())
        if not (self.calibration_ready and self.armed):
            self.send_stop()
        elif self.require_controller_heartbeat:
            # Bounded replay only while the independent controller lease and
            # both UI leases remain valid. All checks precede every write.
            self.write_gated_command(self.effective_command(self.last_cmd, time.monotonic()))
        self.publish_command_status(time.monotonic())

    def close_serial(self):
        """Idempotent, ROS-independent cleanup, also safe after partial init."""
        if getattr(self, '_closed', False):
            return
        self._closed = True
        self.armed = False
        port = getattr(self, 'ser', None)
        try:
            if port is not None and port.is_open:
                # No DDS or ROS context access here, including error handling.
                # write_timeout bounds the attempt; flush() can hang on a lost
                # device and is intentionally avoided. Firmware handles loss.
                count = port.write(b'X\n')
                print(f'[serial_sender_node_v2] shutdown STOP X write={count}/2; PWM unconfirmed')
        except Exception as exc:
            print(f'[serial_sender_node_v2] shutdown STOP write failed: {exc}')
        finally:
            if port is not None:
                try:
                    port.close()
                except Exception as exc:
                    print(f'[serial_sender_node_v2] serial close failed: {exc}')

    def destroy_node(self):
        try:
            self.close_serial()
        finally:
            super().destroy_node()


def main(args=None):
    # Own both signals so SIGTERM, like Ctrl+C, reaches finally BEFORE closing
    # serial. SIGKILL cannot run Python cleanup and needs the board watchdog.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def stop_signal(signum, frame):
        raise KeyboardInterrupt
    previous = {sig: signal.signal(sig, stop_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        # Keep the object reachable if init fails after opening the USB port.
        node = SerialSenderNode.__new__(SerialSenderNode)
        node.__init__()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        print(f'[serial_sender_node_v2] ERROR: {exc}')
    finally:
        # A second Ctrl+C must not interrupt the first stop/close attempt.
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        if node is not None:
            try:
                node.close_serial()
            finally:
                if hasattr(node, '_Node__node'):
                    node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
