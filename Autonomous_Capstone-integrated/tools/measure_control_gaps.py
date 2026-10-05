#!/usr/bin/env python3
"""Observe control-command gaps without publishing or opening any hardware.

Run while track_drive_tuning is already running and DISARMED (do not press W).
The controller computes and publishes commands in that state too. Measuring
actual receive gaps, including camera/DDS/processing delays, is more useful for
watchdog sizing than the camera's configured FPS or mean inference duration.
This observer does not authorize motion, change parameters or resend commands.
"""

import argparse
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from interfaces_pkg.msg import MotionCommand


def percentile(values, fraction):
    """Linear interpolation of sorted observations, reported in milliseconds."""
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    return round((ordered[low] + (ordered[high] - ordered[low]) * (position - low)) * 1000, 3)


class ControlGapObserver(Node):
    def __init__(self, topic, threshold):
        super().__init__('control_gap_observer')
        self.threshold = threshold
        self.received = 0
        self.invalid = 0
        self.last_received = None
        self.gaps = []
        self.max_silence = 0.0
        # Match the serial bridge's reliable latest-command subscription. The
        # observer is a different subscriber, so its measurements are evidence
        # about the stream, not proof of the bridge's own exact receive timing.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(MotionCommand, topic, self.on_command, qos)

    def on_command(self, msg):
        if not (-7 <= msg.steering <= 7 and -255 <= msg.left_speed <= 255
                and -255 <= msg.right_speed <= 255):
            self.invalid += 1
            return
        now = time.monotonic()
        if self.last_received is not None:
            gap = now - self.last_received
            self.gaps.append(gap)
            self.max_silence = max(self.max_silence, gap)
        self.last_received = now
        self.received += 1

    def observe_silence(self):
        # Include silence at the END of the observation. Counting only gaps
        # between two arrivals would miss a controller that stops completely.
        if self.last_received is not None:
            self.max_silence = max(self.max_silence, time.monotonic() - self.last_received)

    def summary(self):
        self.observe_silence()
        return {
            'valid_commands': self.received,
            'invalid_commands': self.invalid,
            'gap_p95_ms': percentile(self.gaps, 0.95),
            'gap_p99_ms': percentile(self.gaps, 0.99),
            'gap_max_ms': round(max(self.gaps) * 1000, 3) if self.gaps else None,
            'max_observed_silence_ms': round(self.max_silence * 1000, 3) if self.received else None,
            'watchdog_threshold_ms': round(self.threshold * 1000, 3),
            'completed_gaps_at_or_above_threshold': sum(g >= self.threshold for g in self.gaps),
            'silence_reached_threshold': self.max_silence >= self.threshold if self.received else None,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/topic_control_signal')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--threshold', type=float, default=0.5,
                        help='Comparison only; does not change any watchdog setting')
    options, ros_args = parser.parse_known_args()
    for name in ('duration', 'threshold'):
        value = getattr(options, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'--{name} must be finite and positive')

    rclpy.init(args=ros_args)
    node = ControlGapObserver(options.topic, options.threshold)
    started = time.monotonic()
    print('Observing commands only. Keep vehicle disarmed; this tool never publishes.', flush=True)
    try:
        while rclpy.ok() and time.monotonic() - started < options.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.observe_silence()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(json.dumps(node.summary(), indent=2, sort_keys=True))
        if node.received == 0:
            print('No valid commands received; watchdog suitability is unmeasured.')
        print('Observed gaps do not guarantee that a future delay will stay below the threshold.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
