"""Exercise actual ROS-adapter method bodies with in-memory transport doubles.

This does NOT claim ROS middleware/launch validation. Imports are omitted by
AST so the control adapter can be checked on a machine without ROS installed.
"""
import ast
from dataclasses import fields, replace
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from skku_track_drive_pkg.messages import MotionCommand
from skku_track_drive_pkg.modes.perpendicular_camera.configuration import (
    CAPTURE_FIELDS, FRONT_VISION_FIELDS, REAR_VISION_FIELDS, VISION_FIELDS, load_config, validate_config)
from skku_track_drive_pkg.modes.perpendicular_camera.core import CameraParkingCore, ControlConfig
from skku_track_drive_pkg.modes.perpendicular_camera.observations import Line, Observation
from skku_track_drive_pkg.modes.perpendicular_camera.status import gate_reason_text, reason_text

ROOT = Path(__file__).resolve().parents[3]
MODULE = ROOT/'src/skku_track_drive_pkg/skku_track_drive_pkg/modes/perpendicular_camera/controller_node.py'
CONFIG = ROOT/'src/vehicle_bringup_pkg/config/perpendicular_camera.yaml'


def adapter():
    definition = next(n for n in ast.parse(MODULE.read_text()).body if isinstance(n, ast.ClassDef))
    fake_time = SimpleNamespace(value=10.0)
    fake_time.monotonic = lambda: fake_time.value
    ns = dict(Node=object, time=fake_time, json=json, MotionCommand=MotionCommand,
              Observation=Observation, ControlConfig=ControlConfig, fields=fields,
              CAPTURE_FIELDS=CAPTURE_FIELDS, VISION_FIELDS=VISION_FIELDS,
              FRONT_VISION_FIELDS=FRONT_VISION_FIELDS, REAR_VISION_FIELDS=REAR_VISION_FIELDS,
              validate_config=validate_config,
              String=lambda **kw: SimpleNamespace(**kw),
              SetParametersResult=lambda **kw: SimpleNamespace(**kw))
    exec(compile(ast.Module(body=[definition], type_ignores=[]), str(MODULE), 'exec'), ns)
    obj = ns['CameraParkingController'].__new__(ns['CameraParkingController'])
    obj.clock = SimpleNamespace(ns=2_000_000_000)
    obj.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=obj.clock.ns))
    obj.get_logger = lambda: Mock()
    obj.cmd_pub = Mock()
    obj.status_pub = Mock()
    obj.tuning_pub = Mock()
    obj.tuning_revision = 0
    obj.data = load_config(CONFIG)
    obj.active_capture = {
        role: {key: obj.data[role][key] for key in CAPTURE_FIELDS} for role in ('front', 'rear')
    }
    obj.core = CameraParkingCore(ControlConfig(calibration_confirmed=True))
    obj.core.was_armed = True
    obj.core.state = 'PARKING_IN'
    obj.armed, obj.preview = True, True
    obj.operator_stopped = False
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
    assert reason_text(json.loads(obj.status_pub.publish.call_args.args[0].data)['reason']) == '후방 영상 지연'


def test_fast_s_w_cannot_keep_old_confirmation_or_move_on_old_capture():
    obj, timer = adapter()
    obj.core.state = 'SEARCHING_SPACE'
    obj.core.missing_count = 2
    obj.pipeline_seen = {'front', 'rear'}
    send(obj, 'front', obj.clock.ns-1)
    obj.on_arm(SimpleNamespace(data=False))
    obj.on_arm(SimpleNamespace(data=True))
    obj.on_timer()
    assert obj.core.missing_count == 0 and obj.core.state == 'SEARCHING_SPACE'
    assert obj.cmd_pub.publish.call_args.args[0].left_speed == 0


def test_detection_between_timer_ticks_breaks_the_missing_run():
    obj, _ = adapter()
    obj.core.state = 'SEARCHING_SPACE'
    obj.pipeline_seen = {'front', 'rear'}
    for i in range(1, 5):
        send(obj, 'front', 1_900_000_000 + i)
    send(obj, 'front', 1_900_000_005, space_found=True)
    send(obj, 'front', 1_900_000_006)
    obj.on_timer()
    assert obj.core.state == 'SEARCHING_SPACE' and obj.core.missing_count == 1


def test_preview_operator_stop_is_shown_even_before_first_images():
    obj, _ = adapter()
    obj.on_arm(SimpleNamespace(data=False))
    obj.on_timer()
    status = json.loads(obj.status_pub.publish.call_args.args[0].data)
    assert status['pwm'] == 0
    assert reason_text(status['reason']) == 'S 입력으로 정지'


@pytest.mark.parametrize('raw,text', [
    ('operator/UI disarm', 'S 입력으로 정지'),
    ('UI heartbeat timeout', 'GUI heartbeat 시간 초과'),
    ('control command timeout', '제어 명령 수신 시간 초과'),
])
def test_live_gate_stop_reasons(raw, text):
    assert gate_reason_text(raw) == text


def test_viewer_summary_keeps_approach_reason_visible_during_pause():
    file = MODULE.with_name('viewer_node.py')
    definition = next(n for n in ast.parse(file.read_text()).body if isinstance(n, ast.ClassDef))
    draw = next(n for n in definition.body if isinstance(n, ast.FunctionDef) and n.name == 'draw')
    namespace = dict(reason_text=reason_text, gate_reason_text=gate_reason_text,
                     time=SimpleNamespace(monotonic=lambda: 0.0))
    exec(compile(ast.Module(body=[draw], type_ignores=[]), str(file), 'exec'), namespace)
    viewer = SimpleNamespace(
        video=SimpleNamespace(winfo_width=lambda: 0, winfo_height=lambda: 0),
        preview=True, gate_status={}, summary=Mock(), snapshot=None, closed=False, challenge='',
        status={'state': 'DIRECTION_PAUSE', 'pwm': 0, 'reason': 'DIRECTION_PAUSE',
                'transition': 'ALIGN_TO_SPACE → DIRECTION_PAUSE / ' + reason_text('FRONT_LIMIT_REACHED')},
    )
    namespace['draw'](viewer)
    displayed = viewer.summary.set.call_args.args[0]
    assert 'DIRECTION_PAUSE   PWM 0' in displayed
    assert '전진→후진 전환 대기' in displayed
    assert '전방 평행주차선 접근 — 후진 전환을 위해 정지' in displayed


def test_backwards_clock_fault_requires_explicit_reset():
    obj, timer = adapter()
    obj.pipeline_seen = {'front', 'rear'}
    obj.clock.ns -= 100
    obj.on_timer()
    assert obj.core.state == 'FAULT_STOP'
    assert obj.cmd_pub.publish.call_args.args[0].left_speed == 0


def test_live_parameter_updates_are_rejected_and_disarmed_update_resets():
    obj, timer = adapter()
    p = [SimpleNamespace(name='align_fallback_s', value=2.5),
         SimpleNamespace(name='reverse_right_s', value=1.25)]
    assert not obj.on_parameters(p).successful
    obj.armed = False
    assert obj.on_parameters(p).successful
    assert obj.core.p.align_fallback_s == 2.5
    assert obj.core.state == 'SEARCHING_SPACE'
    applied = json.loads(obj.tuning_pub.publish.call_args.args[0].data)
    assert applied['config']['control']['align_fallback_s'] == 2.5
    assert applied['config']['control']['reverse_right_s'] == obj.core.p.reverse_right_s == 1.25
    assert not obj.on_parameters([SimpleNamespace(name='align_fallback_s', value=0.0)]).successful


def test_tuning_is_atomic_and_motion_waits_for_both_new_perception_revisions():
    obj, _ = adapter()
    obj.armed = False
    obj.pipeline_seen.update(('front', 'rear'))

    def change(revision, updates):
        return SimpleNamespace(name='camera_tuning_json', value=json.dumps({
            'revision': revision, 'cameras': {'rear': updates},
        }))

    gain = SimpleNamespace(name='rear_position_gain', value=9.0)
    assert not obj.on_parameters([gain, change(0, {'rotate_deg': 45})]).successful
    assert obj.core.p.rear_position_gain == 14.0 and obj.tuning_revision == 0
    assert obj.on_parameters([gain, change(0, {'rotate_deg': 90, 'width': 640})]).successful
    assert obj.core.p.rear_position_gain == 9.0 and obj.capture_restart_pending()
    applied = json.loads(obj.tuning_pub.publish.call_args.args[0].data)
    assert applied['revision'] == 1 and applied['config']['rear']['rotate_deg'] == 90
    assert applied['active_capture']['rear']['width'] == 1280
    assert not obj.on_parameters([change(0, {'rotate_deg': 180})]).successful
    for role in ('front', 'rear'):
        send(obj, role, obj.clock.ns-2, tuning_revision=0)
    assert not obj.pipeline_seen
    send(obj, 'front', obj.clock.ns-1, tuning_revision=1)
    assert obj.pipeline_seen == {'front'}
    send(obj, 'rear', obj.clock.ns-1, tuning_revision=1)
    assert obj.pipeline_seen == {'front', 'rear'}
    obj.preview, obj.armed = False, True
    obj.on_timer()
    assert obj.cmd_pub.publish.call_args.args[0].left_speed == 0
    assert json.loads(obj.status_pub.publish.call_args.args[0].data)['capture_restart_pending']


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
              replace=replace, json=json, VISION_FIELDS=VISION_FIELDS,
              FRONT_VISION_FIELDS=FRONT_VISION_FIELDS, REAR_VISION_FIELDS=REAR_VISION_FIELDS,
              String=lambda **kw: SimpleNamespace(**kw),
              orient_image=lambda frame, *a: frame,
              extract=lambda frame, polygons, cfg, role, stamp, frame_id:
                  (Observation(stamp, role, frame_id), frame, frame))
    exec(compile(ast.Module(body=[definition], type_ignores=[]), str(file), 'exec'), ns)
    obj = ns['ParkingPerceptionNode'].__new__(ns['ParkingPerceptionNode'])
    obj.role, obj.timeout = 'rear', .5
    obj.tuning_revision = 0
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
    config = load_config(CONFIG)
    config['rear']['rotate_deg'] = 90
    config['rear']['confidence'] = .7
    obj.on_tuning(SimpleNamespace(data=json.dumps({'revision': 1, 'config': config})))
    assert obj.cfg['rotate_deg'] == 90 and obj.cfg['confidence'] == .7
    clock.ns = 1_000_000_000
    header.stamp.sec, header.stamp.nanosec = 0, 900_000_000
    obj.on_image(message)
    assert obj.model.predict.call_count == 2
    latest = Observation.from_json(obj.obs_pub.publish.call_args.args[0].data)
    assert latest.stamp_ns == 900_000_000 and latest.tuning_revision == 1
