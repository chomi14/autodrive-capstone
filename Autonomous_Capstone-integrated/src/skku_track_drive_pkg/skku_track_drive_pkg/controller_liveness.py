"""Controller process health, independent of the inference callback/lock.

This is not image freshness. A wedged process/executor loses its lease; a slow
inference callback may keep its last command while the health callback runs.
"""
import json
import signal
import threading
import uuid

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


class ControllerLiveness:
    def __init__(self, node, progress=None):
        self.node = node
        self.progress = progress
        self.instance = uuid.uuid4().hex
        self.challenge = ''
        self.closed = False
        self.lock = threading.Lock()
        self.group = MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, 'vehicle/controller_heartbeat', qos)
        self.subscription = node.create_subscription(String, 'vehicle/arm_challenge',
            self.on_challenge, latched, callback_group=self.group)
        self.timer = node.create_timer(.1, self.tick, callback_group=self.group)

    def on_challenge(self, message):
        with self.lock:
            self.challenge = message.data

    def _publish(self, active):
        self.publisher.publish(String(data=json.dumps({
            'challenge': self.challenge, 'instance': self.instance, 'active': active,
            **({'perception': self.progress()} if self.progress is not None else {})})))

    def tick(self):
        # Never acquire the controller's parameter/inference lock here.
        with self.lock:
            if not self.closed and self.challenge:
                self._publish(True)

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            if self.challenge and rclpy.ok(context=self.node.context):
                self._publish(False)


def run_controller(factory, args=None):
    """Two callback workers: inference/control and independent health.

    On callback failure or termination, stop health BEFORE waiting for any
    in-flight inference. SIGKILL/SIGSTOP are handled by the receiver's lease.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    for sig in previous:
        signal.signal(sig, interrupted)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = factory()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        try:
            if node is not None:
                node.liveness.close()
            # A long inference must not extend its authorization during exit.
            executor.shutdown(timeout_sec=0.0)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
