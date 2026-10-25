"""Position-rule parity with 0708; no camera, serial port or motor is opened."""
import ast
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from scipy.interpolate import CubicSpline

ROOT=Path(__file__).resolve().parent


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec)
    sys.modules[name]=m
    spec.loader.exec_module(m)
    m.np=np; m.cv2=cv2; m.CubicSpline=CubicSpline
    m.SRC_MAT_BASE=np.float32(m.SRC_MAT_BASE)
    return m


class Tensor:
    def __init__(self,values):self.values=np.array(values,dtype=float)
    def cpu(self):return self
    def numpy(self):return self.values


def detection(name,y_top,x=300,shape=(480,640)):
    scale_y,scale_x=shape[0]/480,shape[1]/640
    bbox=SimpleNamespace(cls=[0],xywh=[Tensor([x*scale_x,(y_top+20)*scale_y,40*scale_x,40*scale_y])])
    return SimpleNamespace(names={0:name},boxes=[bbox],orig_shape=shape)


class ObstacleTests(unittest.TestCase):
    def setUp(self):
        self.new=load('bbox_0708_test',ROOT/'0708_bbox_obstacle.py')
        self.old=load('original_0708_test',ROOT/'original/0708_250.py')

    def test_obstacle_aliases_match_original_traffic_light_switch_conditions(self):
        for lane in ('lane1','lane2'):
            for y_top in (0,10,10.9,11,50,99,100,149,150,200):
                for x in (100,185,186,300):
                    for name in ('obstacle','obstacles'):
                        with self.subTest(lane=lane,y=y_top,x=x,name=name):
                            with patch.object(self.new.time,'time',return_value=100):
                                original=self.old.maybe_switch_lane(detection('traffic_light',y_top,x),lane,0,10)
                                actual=self.new.maybe_switch_lane(detection(name,y_top,x),lane,0,10)
                            self.assertEqual(actual,original)

    def test_traffic_light_no_longer_triggers_switch(self):
        with patch.object(self.new.time,'time',return_value=100):
            self.assertEqual(self.new.maybe_switch_lane(detection('traffic_light',50),'lane2',0,10),
                             ('lane2',0,False))

    def test_cooldown_prevents_immediate_toggle_but_allows_later_toggle(self):
        obj=detection('obstacle',50)
        with patch.object(self.new.time,'time',return_value=100):
            lane,t,changed=self.new.maybe_switch_lane(obj,'lane2',0,10)
        self.assertEqual(lane,'lane1');self.assertTrue(changed)
        with patch.object(self.new.time,'time',return_value=105):
            self.assertEqual(self.new.maybe_switch_lane(obj,lane,t,10),(lane,t,False))
        with patch.object(self.new.time,'time',return_value=111):
            self.assertEqual(self.new.maybe_switch_lane(obj,lane,t,10),('lane2',111,True))

    def test_resized_detection_uses_original_reference_coordinates(self):
        with patch.object(self.new.time,'time',return_value=100):
            a=self.new.maybe_switch_lane(detection('obstacle',50,300),'lane2',0,10)
            b=self.new.maybe_switch_lane(detection('obstacle',50,300,(960,1280)),'lane2',0,10)
        self.assertEqual(a,b)

    def test_empty_or_non_obstacle_result_does_not_change_lane(self):
        for result in (None,SimpleNamespace(boxes=None),detection('lane2',50)):
            self.assertEqual(self.new.maybe_switch_lane(result,'lane2',0,10),('lane2',0,False))

    def test_transformed_empty_roi_does_not_create_fake_path(self):
        edge=np.zeros((480,640),np.uint8)
        cv2.line(edge,(10,10),(20,50),255,2)
        _,roi,_,pts=self.new.process_lane_edge(edge)
        self.assertEqual(np.count_nonzero(roi),0)
        self.assertEqual(pts,[])
        self.assertEqual(self.new.plan_path(pts),([],[]))

    def test_single_boundary_uses_image_width_not_lane_width_as_coordinate_limit(self):
        mask=np.zeros((180,640),np.uint8);mask[:,479:482]=255
        self.assertEqual(self.new.get_lane_center(mask,55,10,0),480-self.new.LANE_WIDTH/2)

    def test_centered_two_boundaries_match_original_path_and_steering(self):
        mask=np.zeros((180,640),np.uint8);mask[:,170:173]=255;mask[:,468:471]=255
        oldpts=[];newpts=[]
        for y in self.new.TARGET_Y_LIST:
            oldpts.append(self.old.TargetPoint(round(self.old.get_lane_center(mask,y,10,0)),y))
            newpts.append(self.new.TargetPoint(round(self.new.get_lane_center(mask,y,10,0)),y))
        oldpath,newpath=self.old.plan_path(oldpts),self.new.plan_path(newpts)
        np.testing.assert_allclose(oldpath,newpath)
        self.assertEqual(self.old.calculate_steering_detail_from_path(*oldpath),
                         self.new.calculate_steering_detail_from_path(*newpath))

    def test_path_and_steering_algorithms_unchanged(self):
        def functions(path):
            return {n.name:ast.dump(n) for n in ast.parse(path.read_text(encoding='utf-8')).body
                    if isinstance(n,ast.FunctionDef)}
        old,new=functions(ROOT/'original/0708_250.py'),functions(ROOT/'0708_bbox_obstacle.py')
        for name in ('plan_path','calculate_slope_between_points','calculate_steering_detail_from_path',
                     'limit_steering_change','quantize_steering','choose_speed','apply_curve_slowdown'):
            self.assertEqual(old[name],new[name],name)

    def test_default_enable_and_disable_argument(self):
        with patch.object(sys,'argv',['drive']):
            args=self.new.parse_args()
        self.assertTrue(args.enable_lane_change)
        self.assertTrue(args.wait_start)
        self.assertEqual(args.launch_burst,0)
        self.assertEqual(args.lane_change_cooldown,10)
        self.assertTrue(Path(args.model).is_file())
        self.assertEqual(args.baudrate,115200)
        with patch.object(sys,'argv',['drive','--disable-obstacle-avoidance']):
            self.assertFalse(self.new.parse_args().enable_lane_change)

    def test_serial_formats_and_zero_command(self):
        m=self.new
        self.assertEqual(m.serial_message(m.MotionCommand(2.35,80,90)),'s2.35l80r90\n')
        m.SERIAL_FORMAT='integer'
        self.assertEqual(m.serial_message(m.MotionCommand(2.35,80,90)),'s2l80r90\n')
        self.assertEqual(m.serial_message(m.MotionCommand(0,0,0)),'X\n')


if __name__=='__main__':unittest.main()
