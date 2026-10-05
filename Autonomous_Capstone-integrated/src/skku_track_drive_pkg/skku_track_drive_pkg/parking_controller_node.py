"""One parking command publisher, using the common vehicle I/O/arm gate."""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from interfaces_pkg.msg import MotionCommand

from .mode_parameters import PERPENDICULAR, PARALLEL
from .lidar_geometry import scan_points
from .parking_core import ParkingCore
from .command_lease import CommandLease
from .controller_liveness import ControllerLiveness, run_controller


class ParkingControllerNode(Node):
    def __init__(self):
        super().__init__('parking_controller_node')
        self.declare_parameter('mode', 'perpendicular')
        self.mode = str(self.get_parameter('mode').value)
        if self.mode not in ('perpendicular', 'parallel'):
            raise RuntimeError('mode must be perpendicular or parallel')
        self.specs = PERPENDICULAR if self.mode == 'perpendicular' else PARALLEL
        self.declare_parameter('scan_topic', '/parking/scan')
        self.declare_parameter('cmd_topic', 'topic_control_signal')
        self.declare_parameter('loaded_tuning_config', '')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('publish_bev_debug', False)
        self.declare_parameter('driver_rotation_offset_deg', 0.0)  # Display only; already applied upstream.
        for name, spec in self.specs.items():
            self.declare_parameter(name, spec[0])
        self.core = ParkingCore(self.mode, {name: self.get_parameter(name).value for name in self.specs})
        reason = self._invalid_values(self.core.p)
        if reason:
            raise RuntimeError(reason)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.command_lease = CommandLease(str(self.get_parameter('cmd_topic').value), self.get_name())
        self.cmd_pub = self.create_publisher(MotionCommand, str(self.get_parameter('cmd_topic').value), 1)
        self.status_pub = self.create_publisher(String, '/parking/status', 1)
        debug_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.debug_pub = self.create_publisher(Image, '/parking/debug_image', debug_qos)
        self.create_subscription(LaserScan, str(self.get_parameter('scan_topic').value), self.on_scan, 1)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'vehicle/drive_state', self.on_armed, state_qos)
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.armed = False
        self.scan = None
        self.scan_received = None
        self.scan_number = self.processed_scan = 0
        self.last_tick = time.monotonic()
        self.last_debug = 0.0
        self.pose_trace = []
        self.bridge = CvBridge()
        self.timer = self.create_timer(.05, self.on_timer)
        self.liveness = ControllerLiveness(self)

    def _invalid_values(self, settings):
        for name, value in settings.items():
            default, minimum, maximum, _step = self.specs[name]
            if type(value) is not type(default) or not math.isfinite(value) or not minimum <= value <= maximum:
                return f'{name} must be {type(default).__name__} in [{minimum}, {maximum}]'
        for name in ('side', 'steering_sign', 'scan_angle_sign'):
            if settings[name] not in (-1, 1):
                return f'{name} must be -1 or 1'
        if settings['range_min_m'] >= settings['range_max_m']:
            return 'range_min_m must be below range_max_m'
        if self.mode == 'perpendicular' and not (settings['case1_max_m'] < settings['case2_max_m'] < settings['case3_max_m'] < settings['start_detect_m']):
            return 'distance case boundaries must be ordered below start_detect_m'
        return ''

    def on_parameters(self, parameters):
        updates = {}
        for parameter in parameters:
            if parameter.name in ('publish_debug', 'publish_bev_debug'):
                if not isinstance(parameter.value, bool):
                    return SetParametersResult(successful=False, reason='display parameters must be bool')
            elif parameter.name in self.specs:
                updates[parameter.name] = parameter.value
            else:
                return SetParametersResult(successful=False, reason=f'{parameter.name} requires restart')
        merged = {**self.core.p, **updates}
        reason = self._invalid_values(merged)
        if reason:
            return SetParametersResult(successful=False, reason=reason)
        if self.mode == 'parallel' and self.core.p['geometry_confirmed'] and {'forward_pwm', 'reverse_pwm'}.intersection(updates) and merged['geometry_confirmed']:
            return SetParametersResult(successful=False, reason='Set geometry_confirmed=0 and remeasure m/s before changing calibrated parallel PWM')
        # Geometry changes invalidate a maneuver already planned from old values.
        geometry = {'geometry_confirmed', 'lidar_x_m', 'lidar_y_m', 'lidar_yaw_deg', 'scan_angle_sign', 'side', 'steering_sign',
                    'wheelbase_m', 'front_overhang_m', 'rear_overhang_m', 'vehicle_width_m',
                    'forward_mps', 'reverse_mps', 'max_wheel_angle_deg', 'entry_lateral_m', 'slot_depth_m',
                    'entry_advance_m', 'steer_step', 'gap_margin_m'}
        if self.armed and geometry.intersection(updates):
            return SetParametersResult(successful=False, reason='S/disarm before changing parking geometry')
        self.core.p.update(updates)
        if geometry.intersection(updates):
            self.core.reset()
        return SetParametersResult(successful=True)

    def on_scan(self, message):
        self.scan = message
        self.scan_received = time.monotonic()
        self.scan_number += 1

    def on_armed(self, message):
        self.armed = bool(message.data)
        if not self.armed:
            self.core.last_command = self.core.command(moving=False)

    def on_reset(self, _request, response):
        if self.armed:
            response.success = False
            response.message = 'S/disarm before R/reset'
        else:
            self.core.reset()
            response.success = True
            response.message = 'Parking search reset; W starts after valid measurements'
        return response

    def on_timer(self):
        processing_started = time.monotonic()
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        points = scan_points(self.scan, self.core.p) if self.scan is not None else np.empty((0, 3))
        fresh = self.scan_received is not None and now - self.scan_received <= self.core.p['scan_timeout_s'] and len(points) > 0
        new_scan = self.scan_number != self.processed_scan
        self.processed_scan = self.scan_number
        command = self.core.step(dt, points, self.armed, fresh, new_scan)
        self.cmd_pub.publish(MotionCommand(steering=command.steering, left_speed=command.left_speed, right_speed=command.right_speed))
        status = {**self.core.status, 'steering': command.steering, 'speed': command.left_speed,
                  'armed': self.armed, 'valid_points': len(points),
                  'frame_stamp_ns': None if self.scan is None else self.scan.header.stamp.sec * 1000000000 + self.scan.header.stamp.nanosec,
                  'processing_ros_ns': self.get_clock().now().nanoseconds,
                  'pipeline_duration_s': time.monotonic() - processing_started,
                  'geometry_confirmed': bool(self.core.p['geometry_confirmed']),
                  'pose_estimate_valid': bool(self.core.p['geometry_confirmed']) and all(self.core.p[key] > 0
                      for key in ('wheelbase_m', 'forward_mps', 'reverse_mps', 'max_wheel_angle_deg')),
                  'geometry_display_status': 'CONFIRMED' if self.core.p['geometry_confirmed'] and not self.core.ready_reason() else 'UNCONFIRMED_CONFIGURED_VALUES'}
        self.status_pub.publish(String(data=json.dumps(status)))
        if self.get_parameter('publish_debug').value and now - self.last_debug >= .1:
            image = self.render(points, status)
            debug = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            if self.scan is not None:
                debug.header = self.scan.header
            self.debug_pub.publish(debug)
            self.last_debug = now

    def render(self, points, status):
        p = self.core.p
        canvas = np.zeros((900, 1000, 3), np.uint8)
        scale, origin = min(110.0, 330.0 / max(1.0, p['range_max_m'] + abs(p['lidar_x_m']))), (500, 550)
        def pixel(x, y):
            return int(origin[0] - y * scale), int(origin[1] - x * scale)
        for x in np.arange(-2.0, 4.1, .5):
            cv2.line(canvas, pixel(x, -4), pixel(x, 4), (40, 40, 40), 1)
        for y in np.arange(-4.0, 4.1, .5):
            cv2.line(canvas, pixel(-2, y), pixel(4, y), (40, 40, 40), 1)
        # Display headings are already in the same rear-axle axes as scan_points.
        # Do not apply lidar_yaw/driver rotation a second time to FOV or sectors.
        fov = [pixel(p['lidar_x_m'], p['lidar_y_m'])]
        for angle in np.linspace(p['visible_min_deg'], p['visible_max_deg'], 90):
            fov.append(pixel(p['lidar_x_m']+p['range_max_m']*math.cos(math.radians(angle)),
                             p['lidar_y_m']+p['range_max_m']*math.sin(math.radians(angle))))
        cv2.fillPoly(canvas, [np.asarray(fov, np.int32)], (22, 22, 22))
        # Inspection sectors originate at the grille LiDAR, not the rear axle.
        for center, color in ((p['side']*p['side_center_deg'], (100, 60, 0)),
                              (-p['side']*p['side_center_deg'], (30, 70, 30))):
            vertices = [pixel(p['lidar_x_m'], p['lidar_y_m'])]
            for angle in np.linspace(center-p['side_half_width_deg'], center+p['side_half_width_deg'], 30):
                r = p['side_detect_m']
                vertices.append(pixel(p['lidar_x_m']+r*math.cos(math.radians(angle)), p['lidar_y_m']+r*math.sin(math.radians(angle))))
            cv2.fillPoly(canvas, [np.asarray(vertices, np.int32)], color)
            if self.mode == 'perpendicular':
                boundary = []
                for angle in np.linspace(center-p['side_half_width_deg'], center+p['side_half_width_deg'], 30):
                    radius = p['start_detect_m']
                    boundary.append(pixel(p['lidar_x_m']+radius*math.cos(math.radians(angle)),
                                          p['lidar_y_m']+radius*math.sin(math.radians(angle))))
                cv2.polylines(canvas, [np.asarray(boundary, np.int32)], False, (255, 140, 0), 2)
        cv2.circle(canvas, pixel(p['lidar_x_m'], p['lidar_y_m']), int(p['range_min_m']*scale), (0, 0, 0), -1)
        for x, y, _angle in points:
            cv2.circle(canvas, pixel(x, y), 2, (0, 220, 220), -1)
        dimensions_available = p['wheelbase_m'] > 0 and p['vehicle_width_m'] > 0
        if dimensions_available:
            cv2.rectangle(canvas, pixel(p['wheelbase_m']+p['front_overhang_m'], p['vehicle_width_m']/2),
                          pixel(-p['rear_overhang_m'], -p['vehicle_width_m']/2),
                          (200, 200, 200) if p['geometry_confirmed'] else (0, 165, 255), 2)
        cv2.drawMarker(canvas, pixel(0, 0), (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.circle(canvas, pixel(p['lidar_x_m'], p['lidar_y_m']), 5, (0, 0, 255), -1)
        yaw = math.radians(p['lidar_yaw_deg'])
        cv2.arrowedLine(canvas, pixel(p['lidar_x_m'], p['lidar_y_m']),
                        pixel(p['lidar_x_m']+.5*math.cos(yaw), p['lidar_y_m']+.5*math.sin(yaw)), (0, 0, 255), 2)
        self.pose_trace.append(tuple(status['pose_estimate']))
        self.pose_trace = self.pose_trace[-300:]
        # Separate starting-pose inset; main point cloud stays in current body axes.
        cv2.rectangle(canvas, (730, 580), (995, 835), (120, 120, 120), 1)
        pose = status['pose_estimate']
        span = max(1.0, *(abs(value) for sample in self.pose_trace for value in sample[:2]))
        def world(x, y):
            return int(860-y*100/span), int(710-x*100/span)
        trace = np.asarray([world(x, y) for x, y, _ in self.pose_trace], np.int32)
        if len(trace) > 1:
            cv2.polylines(canvas, [trace], False, (255, 150, 0), 1)
        cv2.circle(canvas, world(0, 0), 3, (255, 255, 255), -1)
        pose_valid = bool(p['geometry_confirmed']) and all(p[key] > 0
            for key in ('wheelbase_m', 'forward_mps', 'reverse_mps', 'max_wheel_angle_deg'))
        if pose_valid:
            cv2.arrowedLine(canvas, world(pose[0], pose[1]),
                            world(pose[0]+.3*math.cos(pose[2]), pose[1]+.3*math.sin(pose[2])), (255, 150, 0), 2)
        cv2.putText(canvas, 'ESTIMATED / start axes' if pose_valid else 'UNCONFIRMED pose model',
                    (735, 602), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 150, 0), 1)
        cv2.putText(canvas, '+X forward   +Y left   rear axle=(0,0)   red=grille LiDAR', (15, 865), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
        lines = [f"{self.mode.upper()} {status['state']}  {status['waiting']}",
                 f"steering={status['steering']:+d} PWM={status['speed']} armed={self.armed}",
                 f"side/opposite(m)={status['side_m']}/{status['opposite_m']} gap={status['gap_m']}",
                 f"visible={p['visible_min_deg']}..{p['visible_max_deg']}deg  sign={p['scan_angle_sign']} yaw={p['lidar_yaw_deg']}",
                 f"pose estimate={status['pose_estimate']} case={status['distance_case']}",
                 f"GEOMETRY {'CONFIRMED' if p['geometry_confirmed'] and not self.core.ready_reason() else 'UNCONFIRMED / configured examples, not measurements'}",
                 f"LiDAR x/y={p['lidar_x_m']}/{p['lidar_y_m']}m range={p['range_min_m']}..{p['range_max_m']}m",
                 f"upstream rotation={self.get_parameter('driver_rotation_offset_deg').value}deg (display only); red arrow=processed scan zero",
                 f"inspection center=+/-{p['side_center_deg']} half={p['side_half_width_deg']}deg radius={p['side_detect_m']}m",
                 f"perpendicular start threshold={p.get('start_detect_m', 'N/A')}m (blue arc); dark gray=FOV, inner black=minimum range",
                 'pose inset: PWM bicycle estimate, not odometry; missing dimensions are not drawn',
                 'W=start/resume S=stop P=save R=reset while disarmed',
                 'Rear/body occlusion is UNKNOWN, not a clear rear corridor.']
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (12, 25+i*26), cv2.FONT_HERSHEY_SIMPLEX, .52, (255, 255, 255), 1)
        return canvas

    def destroy_node(self):
        self.liveness.close()
        if rclpy.ok():
            self.cmd_pub.publish(MotionCommand())
        self.command_lease.close()
        super().destroy_node()


def main(args=None):
    run_controller(ParkingControllerNode, args)
