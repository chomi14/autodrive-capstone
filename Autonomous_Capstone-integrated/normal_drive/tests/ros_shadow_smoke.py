import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time
import rclpy
from rclpy.serialization import deserialize_message
from interfaces_pkg.msg import SourceFrame
from std_msgs.msg import String

root=Path(__file__).resolve().parents[1];report=json.loads((root/'docs/browser_validation.json').read_text());checkpoint=report['models'][0]['checkpoint']
rclpy.init();node=rclpy.create_node('synthetic_shadow_validation');pub=node.create_publisher(SourceFrame,'/e2e/source_frame',10);received=[]
sub=node.create_subscription(String,'/e2e/prediction',lambda m:received.append(json.loads(m.data)),10)
log=(root/'docs/ros_shadow_process.log').open('w')
proc=subprocess.Popen(['ros2','run','e2e_learning_pkg','e2e_shadow','--ros-args','-p','checkpoint:='+checkpoint],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 deadline=time.monotonic()+20
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.1)
  if pub.get_subscription_count():break
 assert pub.get_subscription_count()>0,'Shadow subscriber not ready'
 db=next((root/'work/demo/demo_run_1/input_bag').glob('*.db3'))
 with sqlite3.connect(db) as con:
  for (raw,) in con.execute("select data from messages where topic_id=(select id from topics where type='interfaces_pkg/msg/SourceFrame') order by timestamp limit 4"):
   pub.publish(deserialize_message(raw,SourceFrame))
   for _ in range(5):rclpy.spin_once(node,timeout_sec=.05)
 assert received and all(r['shadow'] and not r['actuation_enabled'] and abs(r['prediction_steps'])<=7 for r in received),received
 assert not node.get_publishers_info_by_topic('/topic_control_signal')
 assert not node.get_publishers_info_by_topic('/e2e/teacher_command')
finally:
 os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=10);log.close();node.destroy_node();rclpy.shutdown()
result=dict(synthetic=True,hardware_opened=False,predictions=received,motion_publishers=0)
(root/'docs/shadow_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
