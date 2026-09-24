#!/usr/bin/env python3
import re
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from std_msgs.msg import Bool, String
from interfaces_pkg.msg import MotionCommand


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

        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.0,
            write_timeout=timeout,
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
        self.ready_pub = self.create_publisher(Bool, self.ready_topic, latched_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos)

        self.rx_timer = self.create_timer(0.03, self.poll_serial)
        self.state_timer = self.create_timer(0.20, self.update_startup_state)
        self.keep_stop_timer = self.create_timer(0.20, self.enforce_safe_state)

        self.send_ascii('X')
        self.start_calibration_sequence()

        self.get_logger().info(
            f'Arduino serial opened: {self.port} @ {self.baud}; '
            f'auto_calibrate={self.auto_calibrate}, tolerance=±{self.cal_tolerance} ADC'
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
        try:
            self.ser.write((line.rstrip('\n') + '\n').encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial command failed ({line!r}): {exc}')

    def send_stop(self):
        try:
            if self.ser is not None and self.ser.is_open:
                # Calibration needs a heartbeat; ordinary STOP must not recenter steering.
                if self.calibration_state in ('WAIT_MEASURE', 'WAIT_APPLY'):
                    self.ser.write(encode_command(0, 0, 0))
                else:
                    self.ser.write(b'X\n')
        except Exception:
            pass

    def poll_serial(self):
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
        except serial.SerialException as exc:
            self.publish_ready(False)
            self.armed = False
            self.get_logger().error(f'Serial read failed: {exc}')

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
        self.armed = False
        self.send_ascii('X')
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
        requested = bool(msg.data)
        if not requested:
            if self.armed:
                self.get_logger().warn('Vehicle DISARMED')
            self.armed = False
            self.send_stop()
            return

        if not self.calibration_ready:
            self.armed = False
            self.get_logger().warn('Arm request ignored: steering calibration is not READY')
            return

        if not self.armed:
            self.get_logger().warn('Vehicle ARMED - motion commands are now allowed')
        self.armed = True

    def on_cmd(self, msg: MotionCommand):
        self.last_cmd = msg
        self.write_gated_command(msg)

    def write_gated_command(self, msg: MotionCommand):
        if self.calibration_ready and self.armed:
            steering = int(msg.steering)
            left_speed = int(msg.left_speed)
            right_speed = int(msg.right_speed)
        else:
            self.send_stop()
            return

        payload = encode_command(steering, left_speed, right_speed)
        try:
            self.ser.write(payload)
        except serial.SerialTimeoutException:
            self.get_logger().error(f'Serial write timeout: {payload!r}')
            try:
                self.ser.reset_output_buffer()
            except Exception:
                pass
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')

    def enforce_safe_state(self):
        # If no controller command is arriving, still actively hold STOP while
        # calibrating/disarmed so a stale Arduino command cannot remain active.
        if not (self.calibration_ready and self.armed):
            self.send_stop()

    def destroy_node(self):
        try:
            self.armed = False
            self.send_ascii('X')
            self.send_stop()
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialSenderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'[serial_sender_node_v2] ERROR: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
