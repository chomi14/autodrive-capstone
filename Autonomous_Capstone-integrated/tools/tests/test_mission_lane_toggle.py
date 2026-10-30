"""L key -> mission service -> target change, without ROS/hardware I/O."""
import json
from concurrent.futures import Future
from threading import RLock
from unittest.mock import Mock, patch

import pytest
from std_srvs.srv import Trigger

from skku_track_drive_pkg.mission_controller_node import MissionControllerNode
from skku_track_drive_pkg.mission_core import MissionCore
from skku_track_drive_pkg.track_tuner_node import TrackTunerNode


def controller():
    node = object.__new__(MissionControllerNode)
    node._parameter_lock = RLock()
    node.mission = MissionCore()
    node.status_pub = Mock()
    node.cmd_pub = Mock()
    node.get_logger = Mock(return_value=Mock())
    return node


def tuner():
    node = object.__new__(TrackTunerNode)
    node.parameter_root = 'mission_controller_node'
    node.target_node = '/mission_controller_node'
    node._window_closed = False
    node._arm_challenge = ''
    node._get_future = Mock()
    node._ui_ready = True
    node._send_pending_parameters = node._draw_status = node._draw_debug = Mock()
    node.forward_drive_key = Mock()
    node.forward_capture_key = Mock()
    node.get_logger = Mock(return_value=Mock())
    return node


def test_lower_and_upper_l_toggle_via_service_without_arming_or_motor_commands():
    control = controller()
    ui = tuner()
    client = Mock()
    client.service_is_ready.return_value = True
    ui.create_client = Mock(return_value=client)

    def call(request):
        response = control.on_toggle_target_lane(request, Trigger.Response())
        future = Future()
        future.set_result(response)
        return future

    client.call_async.side_effect = call
    with patch('skku_track_drive_pkg.track_tuner_node.cv2.getWindowProperty', return_value=1), \
         patch('skku_track_drive_pkg.track_tuner_node.cv2.waitKey', side_effect=[ord('l'), ord('L')]), \
         patch('skku_track_drive_pkg.modes.bbox_mission.mission_controller_node.time.monotonic', return_value=20.):
        for target in (1, 2):
            ui.on_timer()
            assert control.mission.target_lane == target
            status = json.loads(control.status_pub.publish.call_args.args[0].data)
            assert status['target_lane'] == target
            assert status['lane_change_cooldown_remaining_s'] == 5.
            assert status['last_lane_switch_source'] == 'manual'
            assert not status['stop']
    ui.create_client.assert_called_once_with(Trigger, '/mission_controller_node/toggle_target_lane')
    assert client.call_async.call_count == 2
    ui.forward_drive_key.assert_not_called()
    ui.forward_capture_key.assert_not_called()
    control.cmd_pub.publish.assert_not_called()


@pytest.mark.parametrize('root', ['track_controller_node', 'parallel_parking_controller_node'])
def test_l_does_not_change_other_driving_modes(root):
    ui = tuner()
    ui.parameter_root = root
    ui.request_lane_toggle = Mock()
    with patch('skku_track_drive_pkg.track_tuner_node.cv2.getWindowProperty', return_value=1), \
         patch('skku_track_drive_pkg.track_tuner_node.cv2.waitKey', return_value=ord('L')):
        ui.on_timer()
    ui.request_lane_toggle.assert_not_called()
    ui.forward_drive_key.assert_not_called()


def test_unavailable_service_and_pending_request_do_not_queue_l_presses():
    ui = tuner()
    client = Mock()
    ui.create_client = Mock(return_value=client)
    client.service_is_ready.return_value = False
    ui.request_lane_toggle()
    client.call_async.assert_not_called()
    client.service_is_ready.return_value = True
    pending = Future()
    client.call_async.return_value = pending
    ui.request_lane_toggle()
    ui.request_lane_toggle()
    assert client.call_async.call_count == 1
    pending.set_result(Trigger.Response(success=True, message='lane1'))
    assert ui._lane_toggle_future is None
    ui.forward_drive_key.assert_not_called()
