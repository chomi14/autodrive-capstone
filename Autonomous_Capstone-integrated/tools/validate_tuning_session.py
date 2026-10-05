#!/usr/bin/env python3
"""Real video/CUDA recording overhead and input-only bag replay; no hardware."""
import csv
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


VIDEO='/home/autolab/autodrive_ws/h-mobility-class/src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/driving_simulation.mp4'
OUTPUT=Path('/tmp/tuning_session_validation')


def distribution(values):
    return dict(zip(('count','median_ms','p95_ms','max_ms'),
                    (len(values),*map(float,np.percentile(values,[50,95,100])))))


def run(mode,case,bag=None):
    directory=OUTPUT/f'{mode}_{case}'
    if directory.exists():
        raise ValueError(f'Remove only your prior simulation output before rerunning: {directory}')
    rclpy.init()
    monitor=Node('tuning_session_validator');samples=[];names=set()
    monitor.create_subscription(String,f'/{mode}_controller_node/perception_timing',
                                lambda msg:samples.append(json.loads(msg.data)),100)
    cmd=['ros2','launch','vehicle_bringup_pkg','tuning_replay.launch.py',f'mode:={mode}','gui:=false','device:=cuda:0']
    if bag is None:
        cmd.extend(['input_kind:=video',f'input_path:={VIDEO}','playback_fps:=20.0','max_frames:=100'])
    else:
        cmd.extend(['input_kind:=bag',f'input_path:={bag}',f'tuning_config:={bag.parent}/replay_tuning.yaml'])
    if case=='matched':cmd.append(f'processed_csv:={bag.parent}/control.csv')
    if case!='off':cmd.append(f'record_dir:={directory}')
    # Off case overrides the replay launch's analysis=true, keeping default
    # controller instrumentation only; it does not change driving parameters.
    if case=='off':
        cmd[3]='track_drive_tuning.launch.py' if mode=='track' else 'mission_drive_tuning.launch.py'
        cmd=['ros2','launch','vehicle_bringup_pkg',cmd[3],'dry_run:=true','gui:=false','device:=cuda:0']
        source_cmd=['ros2','run','sensor_bringup_pkg','video_replay_node','--ros-args',
                    '-p',f'video_path:={VIDEO}','-p',f'topic:=/{mode}/image_raw','-p','playback_fps:=20.0',
                    '-p','max_frames:=100','-p','wait_for_subscriber:=true','-p',f'required_subscriber_node:={mode}_controller_node']
    log=(OUTPUT/f'{mode}_{case}.log').open('w')
    process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
    source=None
    if case=='off':source=subprocess.Popen(source_cmd,stdout=log,stderr=subprocess.STDOUT)
    try:
        end=time.monotonic()+90
        while time.monotonic()<end:
            rclpy.spin_once(monitor,timeout_sec=.02)
            names.update(monitor.get_node_names())
            if case=='off' and source.poll() is not None:
                process.send_signal(signal.SIGINT);break
            if process.poll() is not None:break
        assert 'serial_sender_node_v2' not in names and 'drive_arm_node' not in names
        if process.poll() is None:process.send_signal(signal.SIGINT)
        process.wait(timeout=20)
        if source is not None:source.wait(timeout=5)
        assert samples and 'Traceback' not in (OUTPUT/f'{mode}_{case}.log').read_text()
        steady=samples[10:]
        result={'results':len(samples),'pipeline':distribution([r['pipeline_duration_s']*1000 for r in steady]),
                'inference':distribution([r['inference_duration_s']*1000 for r in steady]),
                'analysis_build':distribution([r['analysis_build_ms'] for r in steady]) if case!='off' else None,
                'serial_or_gate_started':False}
        if case!='off':
            manifest=json.loads((directory/'manifest.json').read_text())
            assert manifest['rows']>0 and 'bag_exited_early' not in manifest and 'bag_forced_exit' not in manifest
            assert (directory/'replay_tuning.yaml').is_file()
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(directory/'input_bag'),storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('',''))
            stamps=set()
            while reader.has_next():
                topic,data,_=reader.read_next()
                if topic==f'/{mode}/image_raw':
                    image=deserialize_message(data,Image)
                    stamps.add(image.header.stamp.sec*1000000000+image.header.stamp.nanosec)
            rows=list(csv.DictReader((directory/'control.csv').open()))
            matched=sum(int(r['frame_stamp_ns']) in stamps for r in rows)
            assert matched==len(rows), (matched,len(rows))
            result.update(csv_rows=len(rows),bag_frames=len(stamps),source_stamp_joined_rows=matched,
                          initial_speed=manifest['initial_speed'])
        return result
    finally:
        for child in (process,source):
            if child is not None and child.poll() is None:child.kill();child.wait()
        log.close();monitor.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID')!='149':raise RuntimeError('Use ROS_DOMAIN_ID=149')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUTPUT)
    OUTPUT=parser.parse_args().output.expanduser().resolve()
    OUTPUT.mkdir(parents=True,exist_ok=True)
    results={}
    for mode in ('track','mission'):
        results[mode]={}
        for case in ('off','on','replay','matched'):
            results[mode][case]=run(mode,case,OUTPUT/f'{mode}_on/input_bag' if case in ('replay','matched') else None)
            print(mode,case,json.dumps(results[mode][case]),flush=True)
        a=list(csv.DictReader((OUTPUT/f'{mode}_on/control.csv').open()))
        original={r['frame_stamp_ns']:r for r in a}
        fields=('path_points_bev_px','cte_px','heading_error_deg','steering','left_pwm','right_pwm','mission_state','target_lane')
        for case in ('replay','matched'):
            b=list(csv.DictReader((OUTPUT/f'{mode}_{case}/control.csv').open()))
            joined=[(original[r['frame_stamp_ns']],r) for r in b if r['frame_stamp_ns'] in original]
            disagreements=sum(any(json.loads(x['details_json']).get(k)!=json.loads(y['details_json']).get(k) for k in fields) for x,y in joined)
            results[mode][case+'_comparison']={'original_frames':len(a),'replayed_frames':len(b),
                                              'joined_frames':len(joined),'frames_with_any_difference':disagreements,
                                              'note':'source-stamp comparison; timing replay can omit frames and change filter/counter history'}
            assert joined
            if case=='matched':assert len(joined)==len(a)==len(b) and disagreements==0
    (OUTPUT/'results.json').write_text(json.dumps(results,indent=2))
