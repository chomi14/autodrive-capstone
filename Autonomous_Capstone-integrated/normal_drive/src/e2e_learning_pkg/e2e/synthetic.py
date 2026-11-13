"""Synthetic rosbag fixtures, explicitly marked DEMO; never driving evidence."""
from pathlib import Path
import json
import math
import time
import uuid
import cv2
import numpy as np
from rosbags.rosbag2 import Writer
from .core import typestore,save_json


def make_demo(output,frames=18):
    root=Path(output);root.mkdir(parents=True,exist_ok=False);store=typestore();T=store.types;paths=[]
    base=1_790_000_000_123_456_789
    for run in range(3):
        folder=root/f'demo_run_{run+1}';folder.mkdir();uid='DEMO-'+str(uuid.uuid4());cam='DEMO-camera-'+str(run);paths.append(str(folder.resolve()))
        save_json(folder/'manifest.json',dict(schema=2,run_id=uid,origin_run_id=uid,demo=True,mode='track',closed_cleanly=True,
            started_ros_ns=base,initial_speed=80,teacher_model={'name':'SYNTHETIC sinusoid; no real teacher inference'},frame_stamp_semantics='synthetic integer ROS system ns'))
        with Writer(folder/'input_bag',version=8) as writer:
            fc=writer.add_connection('/e2e/source_frame','interfaces_pkg/msg/SourceFrame',typestore=store)
            cc=writer.add_connection('/e2e/teacher_command','interfaces_pkg/msg/TeacherCommand',typestore=store)
            tc=writer.add_connection('/vehicle/command_status','std_msgs/msg/String',typestore=store)
            for i in range(frames):
                ns=base+run*10_000_000_000+i*100_000_000
                tm=T['builtin_interfaces/msg/Time'](ns//1_000_000_000,ns%1_000_000_000)
                image=np.zeros((120,200,3),np.uint8);image[:]=[35,40,48];bend=int(25*math.sin((i+run)/4))
                cv2.line(image,(40,119),(85+bend,30),(235,235,235),4);cv2.line(image,(160,119),(115+bend,30),(0,200,240),4)
                cv2.putText(image,f'DEMO {run+1}/{i}',(5,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,150,255),1)
                im=T['sensor_msgs/msg/Image'](T['std_msgs/msg/Header'](tm,'camera_frame'),120,200,'bgr8',0,600,image.reshape(-1))
                sf=T['interfaces_pkg/msg/SourceFrame'](cam,i,'ros_host_system',i*100_000_000,i*100_000_000+5_000_000,im)
                writer.write(fc,ns,store.serialize_cdr(sf,fc.msgtype))
                step=int(round(5*math.sin((i+run)/4)));cmd=T['interfaces_pkg/msg/MotionCommand'](step,80,80)
                teacher=T['interfaces_pkg/msg/TeacherCommand'](f'{uid}:cmd:{i}',cam,i,tm,'ros_host_system',ns+45_000_000,
                    i*100_000_000+6_000_000,i*100_000_000+45_000_000,cmd,True,True,2,True,'TRACK')
                writer.write(cc,ns+45_000_000,store.serialize_cdr(teacher,cc.msgtype))
                for rep in range(2):
                    t=dict(kind='transmission',command_id=teacher.command_id,transmission_id=f'{uid}:tx:{i}:{rep}',
                        received_ros_ns=ns+46_000_000,received_monotonic_ns=i*100_000_000+46_000_000,
                        write_started_monotonic_ns=i*100_000_000+50_000_000+rep*10_000_000,write_completed_monotonic_ns=i*100_000_000+51_000_000+rep*10_000_000,
                        payload=f's{step}l80r80\n',write_success=True,armed=True,stop_reason='',wheel_angle_deg=None,speed_m_s=None,mcu_received=None)
                    sm=T['std_msgs/msg/String'](json.dumps(t));writer.write(tc,ns+51_000_000+rep*10_000_000,store.serialize_cdr(sm,tc.msgtype))
    return paths
