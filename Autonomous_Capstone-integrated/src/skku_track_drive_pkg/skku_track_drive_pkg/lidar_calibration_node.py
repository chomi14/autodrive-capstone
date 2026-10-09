"""LiDAR bird's-eye GUI: draggable sliders, live shared calibration, P save."""
import json
import time
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from . import lidar_camera_fusion as cfg

# slider: (최솟값, 최댓값, 드래그 한 칸 간격). 기본값은 cfg.SETTINGS.
SLIDERS = {
    'yaw_deg': (-180, 180, 1), 'angle_sign': (-1, 1, 2),
    'camera_yaw_deg': (-180, 180, 1), 'camera_fx_px': (100, 1500, 5),
    'camera_cx_px': (0, 640, 1), 'lidar_x_m': (-1, 1, .01),
    'lidar_y_m': (-1, 1, .01), 'range_min_m': (.05, .5, .01),
    'range_max_m': (1, 12, .1), 'near_m': (.3, 5, .05),
    'stop_m': (.2, 2, .05), 'min_points': (1, 20, 1),
    'margin_px': (0, 100, 1), 'max_cluster_gap_m': (.02, .6, .01),
    'scan_timeout_s': (.1, 2, .05), 'image_timeout_s': (.1, 3, .05),
    'max_pair_delta_s': (.05, 1, .01), 'confirmed': (0, 1, 1),
}
PANEL_ROWS = 9                    # 슬라이더를 두 창에 나누어 표시
WINDOW = 'LiDAR Calibration'


class LidarCalibrationNode(Node):
    def __init__(self):
        super().__init__('lidar_calibration_node')
        self.declare_parameter('calibration_path', cfg.CALIBRATION_PATH)
        self.path = self.get_parameter('calibration_path').value
        self.p = cfg.load_settings(self.path)
        self.scan, self.received = None, 0.0
        self.error, self.saved = '', ''
        self.pub = self.create_publisher(String, cfg.CALIBRATION_TOPIC,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, cfg.SCAN_TOPIC, self.on_scan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self.controls = [f'{WINDOW} Controls {i+1}' for i in range(math.ceil(len(SLIDERS)/PANEL_ROWS))]
        self.locations = {}
        for name in self.controls:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 700, 450)
        self.loading = True
        for i, (name, (lo, hi, step)) in enumerate(SLIDERS.items()):
            panel = self.controls[i//PANEL_ROWS]
            self.locations[name] = panel
            pos = int(round((self.p[name]-lo)/step))
            cv2.createTrackbar(name, panel, pos, int(round((hi-lo)/step)), self.changed)
        self.loading = False
        self.publish()
        self.create_timer(cfg.GUI_PERIOD_S, self.draw)

    def on_scan(self, scan):
        self.scan, self.received = scan, time.monotonic()

    def publish(self):
        self.pub.publish(String(data=json.dumps(self.p)))

    def changed(self, _=0):
        if self.loading:
            return
        values = dict(self.p)
        for name, (lo, _, step) in SLIDERS.items():
            value = round(lo + cv2.getTrackbarPos(name, self.locations[name])*step, 6)
            values[name] = int(value) if isinstance(cfg.SETTINGS[name], int) else value
        try:
            self.p = cfg.validate(values)
            self.error = ''
            self.publish()
        except ValueError as exc:
            self.error = str(exc)  # 잘못된 조합은 마지막 승인값 유지
            self.p['confirmed'] = 0
            self.publish()

    def draw(self):
        if any(cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1 for name in [WINDOW, *self.controls]):
            self.p['confirmed'] = 0
            self.publish()
            rclpy.shutdown()
            return
        size = cfg.WINDOW_SIZE
        canvas = np.zeros((size, size, 3), np.uint8)
        origin = (size//2, size//2)
        scale = (size*.40)/self.p['range_max_m']
        def pixel(x, y):
            return (int(origin[0]-y*scale), int(origin[1]-x*scale))
        for distance in range(1, int(self.p['range_max_m'])+1):
            cv2.circle(canvas, origin, int(distance*scale), (65, 65, 65), 1)
            cv2.putText(canvas, f'{distance}m', (origin[0]+5, origin[1]-int(distance*scale)), cv2.FONT_HERSHEY_SIMPLEX, .4, (130,130,130), 1)
        cv2.line(canvas, (origin[0], 100), (origin[0], size-60), (80,80,80), 1)
        cv2.line(canvas, (40, origin[1]), (size-40, origin[1]), (80,80,80), 1)
        age = time.monotonic()-self.received
        fresh = self.scan is not None and age <= self.p['scan_timeout_s']
        count = 0
        if self.scan is not None:
            xy = cfg.points(self.scan, self.p)
            count = len(xy)
            for x, y in xy:
                cv2.circle(canvas, pixel(x, y), 2, (0,255,0) if fresh else (70,70,150), -1)
        # Camera origin at center; LiDAR location and camera horizontal FOV.
        cv2.circle(canvas, pixel(self.p['lidar_x_m'], self.p['lidar_y_m']), 6, (0,0,255), -1)
        yaw = math.radians(self.p['camera_yaw_deg'])
        for u in (0, 320, 640):
            bearing = yaw + math.atan2(self.p['camera_cx_px']-u, self.p['camera_fx_px'])
            r = self.p['range_max_m']
            cv2.line(canvas, origin, pixel(r*math.cos(bearing), r*math.sin(bearing)), (255,180,0), 1)
        lines = ['UP=forward(+X) LEFT=left(+Y); red=LiDAR; blue=camera FOV',
                 f'{"LIVE" if fresh else "NO SCAN / STALE"} points={count} confirmed={self.p["confirmed"]}',
                 f'yaw={self.p["yaw_deg"]} sign={self.p["angle_sign"]} near={self.p["near_m"]}m stop={self.p["stop_m"]}m',
                 'Drag sliders: live update. P: save. Q/Esc: close + unconfirm.',
                 self.error or self.saved]
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (10, 22+i*22), cv2.FONT_HERSHEY_SIMPLEX, .45, (220,220,220), 1)
        cv2.imshow(WINDOW, canvas)
        key = cv2.waitKey(1) & 0xff
        if key in (ord('p'), ord('P')):
            if not self.error:
                cfg.save_settings(self.p, self.path)
                self.saved = 'Saved calibration JSON'
                self.get_logger().info(f'Saved {self.path}')
        elif key in (27, ord('q'), ord('Q')):
            self.p['confirmed'] = 0
            self.publish()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarCalibrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.p['confirmed'] = 0
                node.publish()
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
