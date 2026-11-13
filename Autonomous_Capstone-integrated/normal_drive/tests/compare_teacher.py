"""Compare original and isolated algorithms on identical synthetic lane detections."""
from pathlib import Path
import hashlib
import importlib.util
import json
import sys
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
def package(name,path):
    spec=importlib.util.spec_from_file_location(name,path/'__init__.py',submodule_search_locations=[str(path)])
    m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m)

def run(name):
    from importlib import import_module as im
    Msg=im(name+'.messages');Lane=im(name+'.lane_processing').LaneInfoExtractor;PathPlanner=im(name+'.path_planner').PathPlanner;Motion=im(name+'.motion_planner').MotionPlanner
    base=yaml.safe_load((ROOT/'src/vehicle_bringup_pkg/config/track_tuning.yaml').read_text())['track_controller_node']['ros__parameters']
    saved=yaml.safe_load((ROOT/'normal_drive/src/vehicle_bringup_pkg/config/track_local_snapshot.yaml').read_text())['track_controller_node']['ros__parameters'];base.update(saved)
    points=[[base[f'bev_src_{corner}_x'],base[f'bev_src_{corner}_y']] for corner in ('tl','tr','br','bl')]
    lane=Lane(show_image=False,bev_source_points=points,bev_top_shift=base['bev_top_shift'],roi_cut=base['roi_cut'],look_shift=base['look_shift'],slope_ema_alpha=base['ema_alpha'],virtual_lane_width=base['virtual_lane_width'],capture_debug=False)
    planner=PathPlanner(car_center_point=(325,179));motion=Motion(stanley_gain=base['stanley_gain'],heading_gain=base['heading_gain'],max_steering=7,max_steering_angle_deg=50,lookahead_steps=base['lookahead_index'],heading_step=3,car_center_x=325,car_center_y=179,default_left_speed=255,default_right_speed=255,stop_on_red=False)
    results=[]
    for shift in [0,0,-20,-30,30,30,0]:
        bitmap=np.zeros((480,640),np.uint8)
        import cv2
        poly=np.array([[235+shift,310],[425+shift,310],[580+shift,479],[80+shift,479]],np.int32)
        cv2.fillPoly(bitmap,[poly],255)
        d=Msg.Detection(class_name='lane2',mask=Msg.Mask(data=[Msg.Point2D(float(x),float(y)) for x,y in poly],width=640,height=480,bitmap=bitmap))
        dets=Msg.DetectionArray(detections=[d]);info=lane.process(dets,np.zeros((480,640,3),np.uint8));path=planner.plan(info);cmd=motion.plan(dets,path,'None',False)
        results.append(dict(steering=cmd.steering,left_pwm=cmd.left_speed,right_pwm=cmd.right_speed,x=path.x_points,y=path.y_points))
    return results

original=ROOT/'src/skku_track_drive_pkg/skku_track_drive_pkg';isolated=ROOT/'normal_drive/src/skku_track_drive_pkg/skku_track_drive_pkg'
package('original_track',original);package('isolated_track',isolated)
files=['yolo_perception.py','lane_processing.py','camera_perception_core.py','bev_geometry.py','path_planner.py','motion_planner.py','messages.py']
hashes={f:hashlib.sha256((original/f).read_bytes()).hexdigest() for f in files}
assert all((original/f).read_bytes()==(isolated/f).read_bytes() for f in files)
a,b=run('original_track'),run('isolated_track');assert a==b
result=dict(input='synthetic lane segmentation masks, not recorded driving',samples=len(a),commands_identical=True,paths_identical=True,
    commands=[{k:v for k,v in x.items() if k not in ('x','y')} for x in a],unchanged_sha256=hashes)
(ROOT/'normal_drive/docs/teacher_equivalence.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
