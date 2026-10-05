"""Health is not image age; no ROS participant or hardware is constructed."""
import json
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from skku_track_drive_pkg.mission_controller_node import MissionControllerNode
from skku_track_drive_pkg.controller_liveness import ControllerLiveness, run_controller


def test_image_timeout_is_diagnostic_and_preserves_mission_stop():
    node = object.__new__(MissionControllerNode)
    node.last_mission_frame = 1.0
    node.mission = SimpleNamespace(p={'image_timeout_s': .75}, status={'state': 'RED_STOP'})
    node.publish_stop = Mock()
    node.status_pub = Mock()
    with patch('skku_track_drive_pkg.mission_controller_node.time.monotonic', return_value=3.0):
        node.check_camera_freshness()
    node.publish_stop.assert_not_called()
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['state'] == 'RED_STOP'
    assert status['system_state'] == 'IMAGE_DELAY_HOLD'
    assert status['image_age_s'] == 2.0


def test_health_ignores_inference_lock_and_cannot_publish_alive_after_close():
    node = Mock()
    health = ControllerLiveness(node)
    health.on_challenge(SimpleNamespace(data='w-token'))
    health.tick()
    payload = json.loads(health.publisher.publish.call_args.args[0].data)
    assert payload['active'] and payload['challenge'] == 'w-token'
    with patch('skku_track_drive_pkg.controller_liveness.rclpy.ok', return_value=True):
        health.close()
        count = health.publisher.publish.call_count
        health.tick()
        health.close()
    assert health.publisher.publish.call_count == count
    assert not json.loads(health.publisher.publish.call_args.args[0].data)['active']


def test_unhandled_executor_failure_stops_health_before_callback_cleanup():
    node, executor = Mock(), Mock()
    order = []
    executor.spin.side_effect = RuntimeError('unhandled control callback failure')
    node.liveness.close.side_effect = lambda: order.append('stop-health')
    executor.shutdown.side_effect = lambda **_kwargs: order.append('shutdown-executor')
    node.destroy_node.side_effect = lambda: order.append('destroy-node')
    with patch('skku_track_drive_pkg.controller_liveness.rclpy.init'), \
         patch('skku_track_drive_pkg.controller_liveness.rclpy.shutdown'), \
         patch('skku_track_drive_pkg.controller_liveness.rclpy.ok', return_value=True), \
         patch('skku_track_drive_pkg.controller_liveness.signal.signal'), \
         patch('skku_track_drive_pkg.controller_liveness.signal.getsignal'), \
         patch('skku_track_drive_pkg.controller_liveness.MultiThreadedExecutor', return_value=executor):
        with pytest.raises(RuntimeError, match='unhandled control callback'):
            run_controller(lambda: node)
    assert order == ['stop-health', 'shutdown-executor', 'destroy-node']
