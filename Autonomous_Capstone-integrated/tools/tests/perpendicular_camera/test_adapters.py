"""Exercise actual ROS-adapter method bodies with in-memory transport doubles.

This does NOT claim ROS middleware/launch validation. Imports are omitted by
AST so the control adapter can be checked on a machine without ROS installed.
"""
import ast
from dataclasses import fields
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from skku_track_drive_pkg.messages import MotionCommand
from skku_track_drive_pkg.modes.perpendicular_camera.core import CameraParkingCore, ControlConfig
from skku_track_drive_pkg.modes.perpendicular_camera.observations import Line, Observation

ROOT = Path(__file__).resolve().parents[3]
MODULE = ROOT/'src/skku_track_drive_pkg/skku_track_drive_pkg/modes/perpendicular_camera/controller_node.py'


def adapter():
    definition = next(n for n in ast.parse(MODULE.read_text()).body if isinstance(n, ast.ClassDef))
    fake_time = SimpleNamespace(value=10.0)
    fake_time.monotonic = lambda: fake_time.value
    ns = dict(Node=object, time=fake_time, json=json, MotionCommand=MotionCommand,
              Observation=Observation, ControlConfig=ControlConfig, fields=fields,
              String=lambda **kw: SimpleNamespace(**kw),
              SetParametersResult=lambda **kw: SimpleNamespace(**kw))
    exec(compile(ast.Module(body=[definition], type_ignores=[]), str(MODULE), 'exec'), ns)
    obj = ns['CameraParkingController'].__new__(ns['CameraParkingController'])
    obj.clock = SimpleNamespace(ns=2_000_000_000)
    obj.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=obj.clock.ns))
    obj.get_logger = lambda: Mock()
    obj.cmd_pub = Mock()
    obj.status_pub = Mock()
    obj.core = CameraParkingCore(ControlConfig(calibration_confirmed=True))
    obj.core.was_armed = True
    obj.core.state = 'PARKING_IN'
    obj.armed, obj.preview = True, True
    obj.observations = {'front': None, 'rear': None}
    obj.received = {'front': 0.0, 'rear': 0.0}
    obj.pipeline_seen = set()
    obj.last_ros_ns = obj.clock.ns
    obj.last_tick = fake_time.value-.1
    return obj, fake_time


def send(obj, role, stamp, **values):
    obj.on_observation(role, SimpleNamespace(data=Observation(stamp, role, **values).to_json()))


def test_adapter_rejects_reordered_wrong_role_future_and_duplicate_observations():
    obj, timer = adapter()
    send(obj, 'rear', 1_900_000_000, space=Line(True, .5, 1., 1.57))
    first = obj.observations['rear']
    timer.value += .2
    send(obj, 'rear', 1_900_000_000)
    send(obj, 'rear', 1_800_000_000)
    send(obj, 'rear', 3_000_000_000)
    obj.on_observation('rear', SimpleNamespace(data=Observation(1_950_000_000, 'front').to_json()))
    assert obj.observations['rear'] is first
    assert obj.received['rear'] == 10.0


def test_both_pipelines_are_required_and_paused_clock_arrival_age_stops_output():
    obj, timer = adapter()
    send(obj, 'rear', 1_900_000_000, space=Line(True, .5, 1., 1.57))
    obj.on_timer()
    assert obj.cmd_pub.publish.call_args.args[0].left_speed == 0
    send(obj, 'front', 1_950_000_000)
    timer.value += .1
    obj.on_timer()  # first active tick establishes a post-W barrier
    obj.clock.ns += 100_000_000
    timer.value += .1
    send(obj, 'rear', obj.clock.ns-1, space=Line(True, .5, 1., 1.57))
    obj.on_timer()
    assert obj.cmd_pub.publish.call_args.args[0].left_speed < 0
    elapsed = obj.core.elapsed
    timer.value += .6  # ROS clock remains paused, monotonic arrival clock advances
    obj.on_timer()
    assert obj.cmd_pub.publish.call_args.args[0].left_speed == 0
    assert obj.core.elapsed == elapsed


def test_fast_s_w_cannot_keep_old_confirmation_or_move_on_old_capture():
    obj, timer = adapter()
    obj.core.state = 'ALIGN_TO_SPACE'
    obj.core.align_count = 2
    obj.pipeline_seen = {'front', 'rear'}
    send(obj, 'rear', obj.clock.ns-1, space=Line(True, .5, 1., 1.57))
    obj.on_arm(SimpleNamespace(data=False))
    obj.on_arm(SimpleNamespace(data=True))
    obj.on_timer()
    assert obj.core.align_count == 0 and obj.core.state == 'ALIGN_TO_SPACE'
    assert obj.cmd_pub.publish.call_args.args[0].left_speed == 0


def test_backwards_clock_fault_requires_explicit_reset():
    obj, timer = adapter()
    obj.pipeline_seen = {'front', 'rear'}
    obj.clock.ns -= 100
    obj.on_timer()
    assert obj.core.state == 'FAULT_STOP'
    assert obj.cmd_pub.publish.call_args.args[0].left_speed == 0


def test_live_parameter_updates_are_rejected_and_disarmed_update_resets():
    obj, timer = adapter()
    p = [SimpleNamespace(name='rear_position_gain', value=6.0)]
    assert not obj.on_parameters(p).successful
    obj.armed = False
    assert obj.on_parameters(p).successful
    assert obj.core.p.rear_position_gain == 6.0
    assert obj.core.state == 'SEARCHING_SPACE'
    assert not obj.on_parameters([SimpleNamespace(name='rear_position_gain', value=6)]).successful


def test_model_adapter_maps_actual_nonfixed_ids_and_rejects_box_only(monkeypatch, tmp_path):
    from skku_track_drive_pkg.modes.perpendicular_camera.model import SegmentModel
    import sys
    weights = tmp_path/'parking.pt'
    weights.write_bytes(b'test double - never loaded')
    boxes = SimpleNamespace(cls=SimpleNamespace(cpu=lambda: SimpleNamespace(tolist=lambda: [17.])))
    class Boxes:
        cls = boxes.cls
        def __len__(self):
            return 1
    output = SimpleNamespace(boxes=Boxes(), masks=SimpleNamespace(xy=[[[1, 1], [3, 1], [3, 3]]]))
    model = SimpleNamespace(task='segment', names={9: 'slot', 17: 'finish'}, predict=lambda *a, **kw: [output])
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=lambda *a, **kw: model))
    cfg = {'model_path': str(weights), 'classes': {'parking_space': 'slot', 'end_line': 'finish'},
           'model_device': 'cpu', 'confidence': .5}
    adapter = SegmentModel(cfg)
    assert len(adapter.predict(None)['end_line']) == 1
    output.masks = None
    with pytest.raises(RuntimeError, match='without segmentation'):
        adapter.predict(None)
    cfg['classes']['parking_space'] = 'missing'
    with pytest.raises(ValueError, match='missing required'):
        SegmentModel(cfg)
    model.task = 'detect'
    with pytest.raises(ValueError, match='segmentation model'):
        SegmentModel(cfg)


def test_perception_preserves_stamp_rejects_duplicate_and_accepts_replay_clock_reset():
    file = MODULE.with_name('perception_node.py')
    definition = next(n for n in ast.parse(file.read_text()).body if isinstance(n, ast.ClassDef))
    ns = dict(Node=object, Observation=Observation, time=SimpleNamespace(monotonic=lambda: 1.0),
              String=lambda **kw: SimpleNamespace(**kw),
              orient_image=lambda frame, *a: frame,
              extract=lambda frame, polygons, cfg, role, stamp, frame_id:
                  (Observation(stamp, role, frame_id), frame, frame))
    exec(compile(ast.Module(body=[definition], type_ignores=[]), str(file), 'exec'), ns)
    obj = ns['ParkingPerceptionNode'].__new__(ns['ParkingPerceptionNode'])
    obj.role, obj.timeout = 'rear', .5
    obj.last_stamp, obj.last_clock_ns = 0, 2_000_000_000
    clock = SimpleNamespace(ns=2_000_000_000)
    obj.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock.ns))
    obj.get_logger = lambda: Mock()
    obj.cfg = dict(rotate_deg=0, flip_horizontal=False)
    obj.model = SimpleNamespace(predict=Mock(return_value={}))
    obj.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *a: None,
                                 cv2_to_imgmsg=lambda *a, **kw: SimpleNamespace())
    obj.obs_pub, obj.status_pub = Mock(), Mock()
    obj.views = {name: Mock() for name in ('oriented', 'segmentation', 'bev')}
    header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=900_000_000), frame_id='rear-camera')
    message = SimpleNamespace(header=header)
    obj.on_image(message)
    first = Observation.from_json(obj.obs_pub.publish.call_args.args[0].data)
    assert first.stamp_ns == 1_900_000_000 and first.frame_id == 'rear-camera'
    obj.on_image(message)
    assert obj.model.predict.call_count == 1
    clock.ns = 1_000_000_000
    header.stamp.sec, header.stamp.nanosec = 0, 900_000_000
    obj.on_image(message)
    assert obj.model.predict.call_count == 2
    assert Observation.from_json(obj.obs_pub.publish.call_args.args[0].data).stamp_ns == 900_000_000
