"""Actual CPU YOLO/controller, synthetic images, no camera/serial/arm node."""
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time
import rclpy
from rclpy.serialization import deserialize_message
from interfaces_pkg.msg import SourceFrame,TeacherCommand
root=Path(__file__).resolve().parents[1]
rclpy.init();node=rclpy.create_node('synthetic_teacher_validation');pub=node.create_publisher(SourceFrame,'/e2e/source_frame',10);received=[];expected={}
sub=node.create_subscription(TeacherCommand,'/e2e/teacher_command',lambda m:received.append(m),10)
log=(root/'docs/ros_teacher_process.log').open('w')
proc=subprocess.Popen(['ros2','run','skku_track_drive_pkg','track_controller_node','--ros-args','--params-file',str(root/'src/vehicle_bringup_pkg/config/track_tuning.yaml'),'--params-file',str(root/'src/vehicle_bringup_pkg/config/track_local_snapshot.yaml'),
 '-p','device:=cpu','-p','cmd_topic:=/validation/teacher_commands','-p','start_enabled:=true','-p','publish_debug:=false','-p','publish_bev_debug:=false','-p','publish_analysis:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 deadline=time.monotonic()+35
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.1)
  if pub.get_subscription_count():break
 assert pub.get_subscription_count(),'Controller source subscriber not ready; inspect ros_teacher_process.log'
 db=next((root/'work/demo/demo_run_1/input_bag').glob('*.db3'))
 with sqlite3.connect(db) as con:
  for (raw,) in con.execute("select data from messages where topic_id=(select id from topics where type='interfaces_pkg/msg/SourceFrame') order by timestamp limit 4"):
   sf=deserialize_message(raw,SourceFrame);expected[sf.sequence]=sf;pub.publish(sf)
   for _ in range(8):rclpy.spin_once(node,timeout_sec=.08)
 for _ in range(10):rclpy.spin_once(node,timeout_sec=.1)
 traced=[m for m in received if m.camera_instance]
 assert traced,'No traced teacher command generated'
 for m in traced:
  sf=expected[m.source_sequence]
  assert m.camera_instance==sf.camera_instance and m.source_stamp==sf.image.header.stamp and m.source_clock_domain==sf.clock_domain
 assert not any('serial_sender' in n for n in node.get_node_names())
 result=dict(synthetic=True,hardware_opened=False,actual_cpu_yolo=True,outputs=[dict(command_id=m.command_id,source_sequence=m.source_sequence,
  source_stamp_ns=m.source_stamp.sec*1000000000+m.source_stamp.nanosec,steering=m.command.steering,left_pwm=m.command.left_speed,right_pwm=m.command.right_speed,path_valid=m.path_valid) for m in traced])
finally:
 os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=15);log.close();node.destroy_node();rclpy.shutdown()
(root/'docs/ros_teacher_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
