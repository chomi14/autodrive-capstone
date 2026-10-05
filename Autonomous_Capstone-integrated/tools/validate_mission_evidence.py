#!/usr/bin/env python3
"""Production mission/path/Stanley with scripted detections, no hardware."""
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from skku_track_drive_pkg import track_controller_node as track
from skku_track_drive_pkg.mission_controller_node import MissionControllerNode
from skku_track_drive_pkg.mission_core import MissionCore
from skku_track_drive_pkg.messages import Detection, DetectionArray, BoundingBox2D, Pose2D, Point2D, Vector2, Mask


def box(name,x,y=378,width=20,height=40):
    return Detection(class_name=name,bbox=BoundingBox2D(Pose2D(Point2D(x,y)),Vector2(width,height)))


class Detector:
    def __init__(self,*a,**k):
        self.yolo=SimpleNamespace(task='segment',names={0:'lane1',1:'lane2',2:'obstacle',3:'traffic_light'})
        self.objects=[];self.alternate=True
    def detect(self,frame):
        masks=[]
        for name,polygon in [('lane1',[[170,315],[260,315],[320,479],[110,479]]),
                             ('lane2',[[305,315],[405,315],[505,479],[320,479]])]:
            if name=='lane1' and not self.alternate:
                continue
            mask=np.zeros((480,640),np.uint8);cv2.fillPoly(mask,[np.asarray(polygon,np.int32)],1)
            masks.append(Detection(class_name=name,mask=Mask(bitmap=mask,height=480,width=640)))
        return DetectionArray(masks+self.objects)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '148':
        raise RuntimeError('Use ROS_DOMAIN_ID=148')
    out=Path('reports/tuning_analysis_2026-10-05/mission_scenarios');out.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=['--ros-args','-p','cmd_topic:=/dry_run/evidence/command','-p','start_enabled:=true',
                    '-p','publish_debug:=true','-p','publish_analysis:=true','-p','speed:=250'])
    node=None
    try:
        with patch.object(track,'YoloDetector',Detector):
            node=MissionControllerNode()
        node.timing_pub.publish=Mock()
        node.debug_pub.publish=Mock()
        frame=np.zeros((480,640,3),np.uint8);bridge=CvBridge();records=[]
        def step(label):
            message=bridge.cv2_to_imgmsg(frame,encoding='bgr8')
            message.header.stamp.nanosec=len(records)*1000000
            node.timing_pub.publish.reset_mock();node.on_image(message)
            assert node.timing_pub.publish.called, label
            record=json.loads(node.timing_pub.publish.call_args.args[0].data)
            record['scenario']=label;records.append(record)
            if node.debug_pub.publish.called:
                cv2.imwrite(str(out/f'{len(records):02d}_{label}.png'),bridge.imgmsg_to_cv2(node.debug_pub.publish.call_args.args[0],desired_encoding='bgr8'))
            return record
        def reset():
            node.mission=MissionCore();node.yolo.objects=[];node.yolo.alternate=True
            # Independent scenarios are not a real lane transition back to lane2.
            node._previous_analysis=None
            frame[:]=0
        baseline=step('normal')
        assert baseline['path_valid']
        camera_path=baseline['mission']['camera_path']
        py=398
        ordered=sorted(camera_path,key=lambda p:p[1])
        px=float(np.interp(py,[p[1] for p in ordered],[p[0] for p in ordered]))
        node.yolo.objects=[box('obstacle',20)]
        outside=step('outside_path')
        assert not outside['mission']['path_blocked'] and outside['left_pwm']==250
        reset();node.yolo.objects=[box('obstacle',px)]
        for i in range(3):
            requested=step('current_path_clear_alternate')
        assert requested['lane_switch_requested'] and requested['target_lane']==1 and requested['left_pwm']==0
        applied=step('after_lane_switch')
        assert applied['lane_switch_applied'] and applied['path_lane']==1
        assert applied['change_from_previous']['path_delta_mean_px'] is not None
        for available in (False,True):
            reset();node.yolo.alternate=available
            node.yolo.objects=[box('obstacle',px)]+([box('obstacle',200,width=100)] if available else [])
            for i in range(3):
                result=step('alternate_blocked' if available else 'alternate_missing')
            assert result['left_pwm']==0 and result['target_lane']==2
            assert result['mission']['reason'].find('ALTERNATE_LANE')>=0
        reset();node.yolo.objects=[box('traffic_light',320,y=200,width=60,height=60)]
        for color,bgr in [('Red',(0,0,255)),('Unknown',(0,0,0)),('Green',(0,255,0))]:
            frame[170:230,290:350]=bgr
            for i in range(3):
                result=step('light_'+color)
            assert result['mission']['traffic_color']==color
            assert (result['left_pwm']==0)==(color in ('Red','Unknown'))
            if color!='Unknown':
                assert result['mission']['traffic_boxes'][0]['ratios'][color]==1.0
        (out/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
        (out/'switch.json').write_text(json.dumps({'request':requested,'applied':applied},indent=2))
        (out/'summary.json').write_text(json.dumps({'scenarios_passed': ['outside_path','current_path','alternate_missing',
            'alternate_blocked','Red->Unknown->Green'],'synthetic_only':True,'frame_count':len(records),
            'lane_switch_request_delta':requested['change_from_previous'],
            'lane_switch_applied_delta':applied['change_from_previous']},indent=2))
        print(json.dumps(json.loads((out/'summary.json').read_text()),indent=2))
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
