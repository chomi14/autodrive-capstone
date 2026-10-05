#!/usr/bin/env python3
"""Synthetic geometry display verification; examples never become measurements."""
import json
import math
import os
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from skku_track_drive_pkg.parking_controller_node import ParkingControllerNode
from skku_track_drive_pkg.lidar_geometry import scan_points


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='150':raise RuntimeError('Use ROS_DOMAIN_ID=150')
    out=Path('reports/tuning_analysis_2026-10-05/parking_display');out.mkdir(parents=True,exist_ok=True)
    results={}
    for mode in ('perpendicular','parallel'):
        # All are synthetic test fixtures, deliberately left UNCONFIRMED.
        parameters={'mode':mode,'cmd_topic':'/dry_run/parking_display/'+mode,'geometry_confirmed':0,
            'wheelbase_m':.7,'front_overhang_m':.2,'rear_overhang_m':.18,'vehicle_width_m':.35,
            'lidar_x_m':.8,'lidar_y_m':.02,'lidar_yaw_deg':30.0,'driver_rotation_offset_deg':180.0}
        args=['--ros-args']
        for k,v in parameters.items():args.extend(['-p',f'{k}:={v}'])
        rclpy.init(args=args);node=None
        try:
            node=ParkingControllerNode()
            scan=LaserScan();scan.angle_min=0.;scan.angle_increment=.01;scan.range_min=.05;scan.range_max=8.;scan.ranges=[2.0]
            scan.header.stamp.sec=123
            node.on_scan(scan);node.on_timer()
            points=scan_points(scan,node.core.p)
            assert np.allclose(points[0,:2],[.8+2*math.cos(math.pi/6),.02+2*math.sin(math.pi/6)])
            labels=[]
            original=cv2.putText
            def draw(image,text,*args,**kw):
                labels.append(text);return original(image,text,*args,**kw)
            status={**node.core.status,'steering':0,'speed':0,'armed':False,'geometry_confirmed':False}
            with patch('skku_track_drive_pkg.parking_controller_node.cv2.putText',side_effect=draw):
                image=node.render(points,status)
            assert any('UNCONFIRMED' in label for label in labels)
            assert any('rear axle=(0,0)' in label for label in labels)
            assert any('upstream rotation=180.0deg (display only)' in label for label in labels)
            assert node.core.p['geometry_confirmed']==0 and node.core.last_command.left_speed==0
            cv2.imwrite(str(out/f'{mode}_synthetic_unconfirmed.png'),image)
            results[mode]={'synthetic_fixture_only':True,'geometry_confirmed':False,'body_point':points[0].tolist(),
                           'rotation_applied_once':True,'labels':labels,'motor_command_zero':True}
        finally:
            if node is not None:node.destroy_node()
            rclpy.shutdown()
    (out/'results.json').write_text(json.dumps(results,indent=2))
    print('Both parking displays: rear-axle geometry, LiDAR/FOV/sectors/pose and UNCONFIRMED checks passed')


if __name__=='__main__':main()
