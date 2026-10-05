"""Observability and replay isolation, without hardware or ROS spinning."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from launch import LaunchContext
from launch_ros.actions import Node
from skku_track_drive_pkg.tuning_analysis import path_change, frame_analysis
from skku_track_drive_pkg.mission_core import MissionCore
import test_four_mode_launches as modes


ROOT=Path(__file__).parents[2]


def test_path_delta_aligns_y_coordinates_instead_of_point_indices():
    result=path_change([[10,0],[30,20]],[[20,0],[25,5],[30,10],[40,20]])
    assert result['path_delta_mean_px']==pytest.approx(10.0)
    assert result['path_delta_max_px']==pytest.approx(10.0)
    assert path_change([],[[10,0],[10,20]])['path_delta_max_px'] is None


def test_transition_record_separates_path_lane_from_next_target_and_keeps_units():
    c=SimpleNamespace(get_name=lambda:'mission_controller_node',_processed_lane=2,
       mission=SimpleNamespace(status={'target_lane':1,'state':'AVOIDING','reason':'LANE_SWITCH_PAUSE',
                                       'lane_switch_requested':True}),lane=SimpleNamespace(missed_frames=0),
       motion=SimpleNamespace(last_cte=12.,last_heading_deg=-15.,last_reference_point=(1,2)))
    path=SimpleNamespace(x_points=[10,10],y_points=[0,20])
    record=frame_analysis(c,path,SimpleNamespace(steering=0,left_speed=0,right_speed=0))
    assert record['path_lane']==2 and record['target_lane']==1 and record['lane_switch_requested']
    c._processed_lane=1
    changed=frame_analysis(c,SimpleNamespace(x_points=[30,30],y_points=[0,20]),
                           SimpleNamespace(steering=-7,left_speed=80,right_speed=80),record)
    assert changed['lane_switch_applied']
    assert changed['change_from_previous']['path_delta_mean_px']==20
    assert changed['change_from_previous']['steering_delta_steps']==-7
    json.dumps(changed)


@pytest.mark.parametrize('mode',list(modes.FILES))
def test_recording_adds_sidecar_without_another_command_producer(mode,tmp_path):
    context,_=modes.topology(mode)
    context.launch_configurations['record_dir']=str(tmp_path/mode)
    from vehicle_bringup_pkg.recording import recording_nodes,analysis_enabled
    nodes=recording_nodes(context,mode)
    assert len(nodes)==1 and nodes[0].node_executable=='tuning_recorder_node'
    assert analysis_enabled(context)


@pytest.mark.parametrize('mode',['track','mission','perpendicular','parallel','calibration'])
def test_replay_forces_existing_launch_to_dry_run_and_never_replays_motor_topics(mode,tmp_path):
    path=ROOT/'src/vehicle_bringup_pkg/launch/tuning_replay.launch.py'
    spec=importlib.util.spec_from_file_location('replay_test',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    values={'mode':mode,'input_kind':'bag','input_path':str(tmp_path),'gui':'false','device':'cpu',
            'rate':'1.0','loop':'false','record_dir':'','tuning_config':'','processed_csv':'','saved_tuning_path':''}
    context.launch_configurations.update(values)
    launch,source,_=module.assemble(context)
    assert dict(launch.launch_arguments)['dry_run']=='true'
    assert source.node_executable=='tuning_bag_replay_node'
    implementation=(ROOT/'src/skku_track_drive_pkg/skku_track_drive_pkg/tuning_bag_replay_node.py').read_text()
    assert "'--topics'" in implementation and 'topic_control_signal' not in implementation
    assert 'arm_request' not in implementation and 'drive_key' not in implementation


def test_measurement_worksheets_do_not_invent_dimensions_or_confirm_geometry():
    import yaml
    template=yaml.safe_load((ROOT/'tools/forms/parking_measurements.template.yaml').read_text())
    assert template['geometry_confirmed']==0
    assert all(entry['measured_value'] is None for entry in template['geometry'].values())
    assert all(value is None for value in template['derived_values_to_review'].values())


def test_recorded_bag_replay_waits_for_original_input_recorder_subscription():
    from unittest.mock import Mock
    from skku_track_drive_pkg.tuning_bag_replay_node import TuningBagReplayNode
    subscriptions=[SimpleNamespace(node_name='track_controller_node')]
    values={'mode':'track','input_topic':'/track/image_raw','target_node':'track_controller_node',
            'wait_for_recorder':True}
    node=SimpleNamespace(process=None,reader=object(),tick_processed=Mock(),
        get_parameter=lambda name:SimpleNamespace(value=values[name]),
        get_subscriptions_info_by_topic=lambda topic:subscriptions)
    TuningBagReplayNode.tick(node)
    node.tick_processed.assert_not_called()
    subscriptions.append(SimpleNamespace(node_name='rosbag2_recorder'))
    TuningBagReplayNode.tick(node)
    node.tick_processed.assert_called_once()


def test_recorded_video_does_not_decode_first_frame_before_recorder_is_ready():
    from unittest.mock import Mock
    from sensor_bringup_pkg.video_replay_node import VideoReplayNode
    subscriptions=[SimpleNamespace(node_name='track_controller_node')]
    node=SimpleNamespace(_stop_deadline=None,_started=False,wait_for_subscriber=True,
        wait_for_recorder=True,required_subscriber_node='track_controller_node',topic='/track/image_raw',
        publisher=Mock(),get_subscriptions_info_by_topic=lambda topic:subscriptions,
        _waiting_logged=False,get_logger=Mock(),cap=Mock(),loop=False,frame_count=0,_finish=Mock())
    node.publisher.get_subscription_count.return_value=1
    node.cap.read.return_value=(False,None)
    VideoReplayNode.on_timer(node)
    node.cap.read.assert_not_called()
    subscriptions.append(SimpleNamespace(node_name='rosbag2_recorder'))
    VideoReplayNode.on_timer(node)
    node.cap.read.assert_called_once()


def test_calibration_log_uses_emitted_zero_pwm_while_configured_pwm_is_nonzero():
    from unittest.mock import Mock
    from std_msgs.msg import String
    from skku_track_drive_pkg.tuning_recorder_node import TuningRecorderNode
    recorder = SimpleNamespace(mode='calibration', event=Mock(), on_frame=Mock())
    TuningRecorderNode.on_status(recorder, String(data=json.dumps({'state':'DISARMED','pwm':-60,
        'steering_step':7,'steering':0,'left_pwm':0,'right_pwm':0,'frame_stamp_ns':None})))
    frame = json.loads(recorder.on_frame.call_args.args[0].data)
    assert frame['left_pwm']==frame['right_pwm']==frame['steering']==0


def test_stop_reason_distinguishes_clear_confirmation_from_a_current_obstacle():
    from skku_track_drive_pkg.messages import DetectionArray, Detection, BoundingBox2D, Pose2D, Point2D, Vector2
    core=MissionCore()
    frame=np.zeros((480,640,3),np.uint8)
    obstacle=Detection(class_name='obstacle',bbox=BoundingBox2D(Pose2D(Point2D(420,380)),Vector2(40,40)))
    path=[(420,180),(420,479)]
    for i in range(3):core.decide(DetectionArray([obstacle]),frame,path,float(i))
    stop,_=core.decide(DetectionArray(),frame,path,3.)
    assert stop and not core.status['path_blocked']
    assert core.status['reason']=='WAIT_OBSTACLE_CLEAR_CONFIRMATION'


def test_legacy_track_video_replay_keeps_commands_and_vehicle_keys_private(tmp_path):
    from vehicle_bringup_pkg.tuning_configuration import tuning_launch_arguments
    path=ROOT/'src/vehicle_bringup_pkg/launch/track_video_replay.launch.py'
    spec=importlib.util.spec_from_file_location('legacy_replay_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    from launch.actions import DeclareLaunchArgument
    for argument in module.generate_launch_description().entities:
        if isinstance(argument,DeclareLaunchArgument):
            argument.execute(context)
    video=tmp_path/'video.mp4';video.touch()
    context.launch_configurations.update({'video_path':str(video),'enable_visualization':'true',
        'publish_debug':'false','debug':'false','show_bev':'true','enable_tuner':'true'})
    actions=module._launch_nodes(context)
    for action in actions:
        if isinstance(action,Node) and action.node_executable in ('track_controller_node','track_tuner_node'):
            from launch_ros.utilities import evaluate_parameters
            values=evaluate_parameters(context,action._Node__parameters)[0]
            assert values['cmd_topic']=='/dry_run/topic_control_signal'
            assert action._Node__remappings
