"""Real sender callbacks, original FakeSerial safety fixture. No DDS or hardware."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/tests'))
from test_drive_safety import DriveSafetyTests
from interfaces_pkg.msg import TeacherCommand,MotionCommand


def test_trace_ids_retransmission_and_fault():
    fixture=DriveSafetyTests();fixture.setUp()
    try:
        fixture.canonical_sender();node=fixture.node
        node.get_clock=Mock(return_value=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1790000000123456789)))
        node.command_status_pub.reset_mock()
        trace=TeacherCommand(command_id='controller:42',camera_instance='camera:1',source_sequence=200,
                             command=MotionCommand(steering=-2,left_speed=80,right_speed=80))
        node.on_traced_cmd(trace)
        for _ in range(2):
            fixture.clock+=.05;fixture.feed_ui();fixture.feed_controller();node.enforce_safe_state()
        records=[json.loads(c.args[0].data) for c in node.command_status_pub.publish.call_args_list]
        tx=[r for r in records if r.get('kind')=='transmission']
        assert len(tx)==2 and all(t['command_id']=='controller:42' for t in tx)
        assert tx[0]['transmission_id']!=tx[1]['transmission_id']
        assert all(t['payload']=='s-2l80r80\n' and t['write_success'] for t in tx)
        assert all(t['wheel_angle_deg'] is None and t['mcu_received'] is None for t in tx)
        fixture.port.fail=True;node.enforce_safe_state();assert not node.armed
        tx=[json.loads(c.args[0].data) for c in node.command_status_pub.publish.call_args_list if json.loads(c.args[0].data).get('kind')=='transmission']
        assert tx[-1]['write_success'] is False and tx[-1]['error']
    finally:fixture.doCleanups()
