"""ROS DDS + original sidecar recorder lifecycle. ISOLATED DOMAIN; no serial nodes."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import sys
import sqlite3
import rclpy
from rclpy.serialization import deserialize_message
from interfaces_pkg.msg import SourceFrame,TeacherCommand
from std_msgs.msg import String

root=Path(__file__).resolve().parents[1];out=root/'work'/('ros_record_'+str(time.time_ns()))
rclpy.init();node=rclpy.create_node('synthetic_record_validation')
pubs={"interfaces_pkg/msg/SourceFrame":node.create_publisher(SourceFrame,'/e2e/source_frame',10),
      "interfaces_pkg/msg/TeacherCommand":node.create_publisher(TeacherCommand,'/e2e/teacher_command',10),
      "std_msgs/msg/String":node.create_publisher(String,'/vehicle/command_status',10)}
classes={'interfaces_pkg/msg/SourceFrame':SourceFrame,'interfaces_pkg/msg/TeacherCommand':TeacherCommand,'std_msgs/msg/String':String}
log=(root/'docs/ros_record_process.log').open('w')
proc=subprocess.Popen(['ros2','run','skku_track_drive_pkg','tuning_recorder_node','--ros-args','-p','output_dir:='+str(out),'-p','demo:=true','-p','input_topic:=/e2e/source_frame','-p','vehicle_prefix:=/vehicle'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        if all(p.get_subscription_count()>0 for p in pubs.values()):break
    assert all(p.get_subscription_count()>0 for p in pubs.values()),'Bag subscribers not discovered'
    db=next((root/'work/demo/demo_run_1/input_bag').glob('*.db3'))
    with sqlite3.connect(db) as con:
        topics={i:t for i,t in con.execute('select id,type from topics')}
        for tid,data in con.execute('select topic_id,data from messages order by timestamp,id'):
            kind=topics[tid];pubs[kind].publish(deserialize_message(data,classes[kind]));rclpy.spin_once(node,timeout_sec=.015)
    for _ in range(15):rclpy.spin_once(node,timeout_sec=.1)
finally:
    os.killpg(proc.pid,signal.SIGKILL if '--abrupt' in sys.argv else signal.SIGINT);proc.wait(timeout=25);log.close();node.destroy_node();rclpy.shutdown()
manifest=json.loads((out/'manifest.json').read_text());assert manifest['closed_cleanly']==('--abrupt' not in sys.argv),manifest
from e2e.core import import_session
index=root/'work/sessions'/out.name
s=import_session(out,index);assert s['stats'].get('direct_source_id')==18,s['stats'];assert s['stats']['transmissions']==36,s['stats']
result=dict(synthetic=True,hardware_opened=False,session=str(out),stats=s['stats'],manifest=manifest)
(root/('docs/ros_recovery_validation.json' if '--abrupt' in sys.argv else 'docs/ros_record_validation.json')).write_text(json.dumps(result,indent=2));print(json.dumps(result))
