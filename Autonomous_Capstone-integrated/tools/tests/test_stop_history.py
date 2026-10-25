"""Transient stop reasons remain readable after recovery; no ROS or GUI started."""
import json
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np

from skku_track_drive_pkg.stop_history import StopHistory
from skku_track_drive_pkg.track_tuner_node import TrackTunerNode


def tuner():
    node = object.__new__(TrackTunerNode)
    node.stop_history = StopHistory()
    node.mode_status = {}
    node.command_status = {}
    node.get_logger = Mock(return_value=Mock())
    return node


def serial(node, armed=True, left_pwm=80, right_pwm=80, stop_reason=''):
    node.on_serial_status(SimpleNamespace(data=json.dumps(dict(
        armed=armed, left_pwm=left_pwm, right_pwm=right_pwm, stop_reason=stop_reason))))


def mode(node, stop, reason):
    node.on_mode_status(SimpleNamespace(data=json.dumps(dict(stop=stop, reason=reason))))


def test_disarm_reason_survives_rearm_without_freezing_live_status():
    node = tuner()
    serial(node)
    with patch('skku_track_drive_pkg.stop_history.time.strftime', return_value='12:34:56'):
        serial(node, False, 0, 0, 'UI heartbeat timeout')
    event = node.stop_history.events[0]
    for _ in range(100):
        serial(node, False, 0, 0, 'UI heartbeat timeout')
    serial(node)
    assert node.command_status['armed'] is True
    assert list(node.stop_history.events) == [event]
    assert event.timestamp == '12:34:56'
    lines = '\n'.join(node._serial_status_lines())
    assert 'armed=True' in lines and 'UI heartbeat timeout' in lines
    node.get_logger().warn.assert_called_once()


def test_same_stop_after_recovery_is_a_new_event_and_history_is_bounded():
    node = tuner()
    for reason in ('UI heartbeat timeout', 'UI heartbeat timeout', 'serial fault', 'controller health timeout'):
        serial(node)
        serial(node, False, 0, 0, reason)
    assert [event.reason for event in node.stop_history.events] == [
        'controller health timeout', 'serial fault', 'UI heartbeat timeout']
    assert node.get_logger().warn.call_count == 4


def test_mission_stop_and_armed_zero_output_both_survive_resume():
    node = tuner()
    mode(node, True, 'LANE_SWITCH_PAUSE')
    serial(node, True, 0, 0)
    mode(node, False, 'NORMAL')
    serial(node)
    assert {event.source for event in node.stop_history.events} == {'MISSION', 'SERIAL'}
    assert 'LANE_SWITCH_PAUSE' in '\n'.join(node._serial_status_lines())
    assert node.mode_status['stop'] is False


def test_malformed_status_does_not_erase_live_state_or_stop_deduplication():
    node = tuner()
    serial(node, False, 0, 0, 'serial fault')
    for payload in ('invalid json', '[]', 'null'):
        node.on_serial_status(SimpleNamespace(data=payload))
        node.on_mode_status(SimpleNamespace(data=payload))
    serial(node, False, 0, 0, 'serial fault')
    assert len(node.stop_history.events) == 1
    assert node.command_status['stop_reason'] == 'serial fault'


def test_long_reason_is_complete_and_fits_both_status_and_debug_widths():
    node = tuner()
    reason = 'ALTERNATE_LANE_NOT_VISIBLE|AVOIDANCE_PATH_NOT_CURRENTLY_VALID|' * 3
    mode(node, True, reason)
    for width, scale in ((640, .52), (1280, .52), (1100, .62)):
        rows = node._serial_status_lines(width, scale)
        assert reason in ''.join(rows)
        assert all(cv2.getTextSize(row, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] <= width - 24 for row in rows)


def test_debug_footer_keeps_reason_after_recovery_and_preserves_video_pixels():
    node = tuner()
    serial(node, False, 0, 0, 'UI heartbeat timeout')
    serial(node)
    node.desired_values = {'publish_debug': True}
    node._latest_debug_frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    node._debug_window_open = True
    original = node._latest_debug_frame.copy()
    labels = []
    original_put_text = cv2.putText

    def capture(image, text, *args):
        labels.append(text)
        return original_put_text(image, text, *args)

    with patch('skku_track_drive_pkg.track_tuner_node.cv2.imshow') as show, \
         patch('skku_track_drive_pkg.track_tuner_node.cv2.putText', side_effect=capture):
        node._draw_debug()
    rendered = show.call_args.args[1]
    assert rendered.shape[0] > 480
    np.testing.assert_array_equal(rendered[:480], original)
    np.testing.assert_array_equal(node._latest_debug_frame, original)
    assert any('UI heartbeat timeout' in text for text in labels)
    assert any('armed=True' in text for text in labels)
