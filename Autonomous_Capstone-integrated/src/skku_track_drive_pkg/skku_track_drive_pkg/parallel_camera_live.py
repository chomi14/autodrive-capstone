"""Markerless white-line localization and feedback for a surveyed static bay."""

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .messages import MotionCommand
from .parallel_camera_planner import Box, Pose, Scene, Vehicle, advance, is_clear, parked, plan, wrap


def scalar(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('expected a finite measured number')
    return float(value)


def vector(values, length):
    if len(values) != length:
        raise ValueError(f'expected {length} measured values')
    return tuple(scalar(value) for value in values)


def rectangle(values):
    b = Box(*vector(values, 4))
    if b.xmin >= b.xmax or b.ymin >= b.ymax:
        raise ValueError('invalid rectangle; expected xmin, ymin, xmax, ymax')
    return b


def fit_pose(local, world, prior, max_error, max_jump, max_yaw):
    """Rigid metric fit from observed ground points to surveyed paint corners."""
    local, world = np.asarray(local, float), np.asarray(world, float)
    a, b = local - local.mean(axis=0), world - world.mean(axis=0)
    if np.linalg.svd(b, compute_uv=False)[-1] < max_error:
        raise ValueError('visible paint corners do not give independent position constraints')
    yaw = math.atan2(float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])), float(np.sum(a * b)))
    c, s = math.cos(yaw), math.sin(yaw)
    rotated = local @ np.array([[c, s], [-s, c]])
    xy = (world - rotated).mean(axis=0)
    if np.linalg.norm(world - rotated - xy, axis=1).max() > max_error:
        raise ValueError('front/rear paint observations disagree')
    if math.hypot(xy[0] - prior.x, xy[1] - prior.y) > max_jump or abs(wrap(yaw - prior.yaw)) > max_yaw:
        raise ValueError('position jump outside the tracking window')
    return Pose(float(xy[0]), float(xy[1]), yaw)


class GroundCamera:
    def __init__(self, config, error):
        self.size = tuple(config['image_size'])
        if len(self.size) != 2 or any(type(n) is not int or n < 32 for n in self.size):
            raise ValueError('image_size must be [width, height]')
        px = np.array([vector(p, 2) for p in config['pixel_points']], dtype=np.float64)
        xy = np.array([vector(p, 2) for p in config['vehicle_points_m']], dtype=np.float64)
        if len(px) != len(xy) or len(px) < 4 or np.any(px < 0) or np.any(px >= self.size):
            raise ValueError('each camera needs four or more matched ground points in the image')
        hom, _ = cv2.findHomography(px, xy)
        if hom is None or not np.isfinite(hom).all() or np.linalg.matrix_rank(hom) != 3:
            raise ValueError('degenerate ground calibration')
        projected = cv2.perspectiveTransform(px[None], hom)[0]
        if np.linalg.norm(projected - xy, axis=1).max() > error:
            raise ValueError('camera ground calibration exceeds pose_error_m')
        self.image_from_ground = np.linalg.inv(hom)
        self.hull = cv2.convexHull(px.astype(np.float32))
        self.anchors = tuple(vector(p, 2) for p in config['corners_world_m'])
        if len(set(self.anchors)) != len(self.anchors) or not self.anchors:
            raise ValueError('provide distinct surveyed paint corners for each camera')

    def observe(self, frame, prior, radius, sat, value):
        if frame.shape[1::-1] != self.size:
            raise ValueError('camera frame size differs from the calibration')
        side = 81
        center = (side - 1) / 2
        scale = radius / center
        c, s = math.cos(prior.yaw), math.sin(prior.yaw)
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), (0, 0, value), (179, sat, 255))
        result = []
        for anchor in self.anchors:
            dx, dy = anchor[0] - prior.x, anchor[1] - prior.y
            x, y = dx * c + dy * s, -dx * s + dy * c
            patch_to_ground = np.array([[scale, 0, x - radius], [0, -scale, y + radius], [0, 0, 1.]])
            source = self.image_from_ground @ patch_to_ground
            boundary = np.array([[0., 0., 1.], [80., 0., 1.], [80., 80., 1.], [0., 80., 1.]]) @ source.T
            denominators = boundary[:, 2]
            if np.min(np.abs(denominators)) < 1e-9 or not np.all(denominators * denominators[0] > 0):
                continue
            pixels = boundary[:, :2] / denominators[:, None]
            if any(cv2.pointPolygonTest(self.hull, tuple(point), False) < 0 for point in pixels.tolist()):
                continue
            patch = cv2.warpPerspective(mask, source, (side, side), flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP)
            corners = cv2.goodFeaturesToTrack(patch, maxCorners=12, qualityLevel=.08, minDistance=6, blockSize=5)
            if corners is None:
                continue
            candidates = sorted((math.hypot(u - center, v - center), u, v) for u, v in corners.reshape(-1, 2)
                                if math.hypot(u - center, v - center) <= center * .65)
            if not candidates or (len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 5):
                continue
            _, u, v = candidates[0]
            result.append(((x + (u - center) * scale, y - (v - center) * scale), anchor))
        return result


class LiveConfig:
    def __init__(self, raw):
        self.vehicle = Vehicle(**{key: scalar(value) for key, value in raw['vehicle'].items()})
        self.scene = Scene(rectangle(raw['slot']), rectangle(raw['bounds']), tuple(map(rectangle, raw['obstacles'])),
                           scalar(raw['margin_m']), scalar(raw['segment_m']), scalar(raw['heading_tolerance_deg']))
        x, y, yaw = vector(raw['start_pose_m_deg'], 3)
        self.start = Pose(x, y, math.radians(yaw))
        gate = raw['tracking']
        self.error = scalar(gate['pose_error_m'])
        self.radius = scalar(gate['corner_window_m'])
        self.jump = scalar(gate['max_pose_jump_m'])
        self.yaw_jump = math.radians(scalar(gate['max_yaw_jump_deg']))
        self.path_error = scalar(gate['max_path_error_m'])
        self.path_yaw = math.radians(scalar(gate['max_path_error_deg']))
        self.frame_age = scalar(gate['max_frame_age_s'])
        self.frame_skew = scalar(gate['max_frame_skew_s'])
        self.saturation = gate['paint_s_max']
        self.brightness = gate['paint_v_min']
        self.stop_distance = scalar(gate['worst_stop_travel_m'])
        self.settle = scalar(gate['settle_s'])
        self.settle_drift = scalar(gate['settle_drift_m'])
        self.move_timeout = scalar(gate['segment_timeout_s'])
        self.forward_pwm, self.reverse_pwm = gate['forward_pwm'], gate['reverse_pwm']
        self.steer_by_step = vector(gate['steering_deg_by_step_minus7_to_plus7'], 15)
        self.commissioned = raw.get('commissioned') is True
        v, s = self.vehicle, self.scene
        if not (v.wheelbase > 0 and v.width > 0 and min(v.front_overhang, v.rear_overhang) >= 0
                and 0 < v.steer_left_deg < 80 and 0 < v.steer_right_deg < 80
                and s.segment_m > 0 and 0 < s.heading_tolerance_deg < 45 and len(s.obstacles) >= 2
                and s.bounds.contains((s.slot.xmin, s.slot.ymin)) and s.bounds.contains((s.slot.xmax, s.slot.ymax))):
            raise ValueError('invalid vehicle, slot, or measured front/rear obstacles')
        if not (s.margin >= self.stop_distance + self.path_error + self.error
                and 0 < self.error < self.radius and 0 < self.jump < self.radius
                and 0 < self.yaw_jump < self.path_yaw < math.pi / 4
                and 0 < self.frame_skew <= self.frame_age <= .25 and self.settle >= self.frame_age
                and min(self.stop_distance, self.settle, self.settle_drift, self.move_timeout, self.path_error) > 0):
            raise ValueError('invalid tracking limits or insufficient measured safety margin')
        if any(type(n) is not int or not 0 <= n <= 255 for n in (self.saturation, self.brightness)):
            raise ValueError('paint HSV thresholds must be integers 0..255')
        if any(type(n) is not int or not 1 <= n <= 255 for n in (self.forward_pwm, self.reverse_pwm)):
            raise ValueError('PWM must be a measured integer 1..255')
        steps = self.steer_by_step
        if (steps[7] != 0 or not (all(a <= b for a, b in zip(steps, steps[1:]))
                                  or all(a >= b for a, b in zip(steps, steps[1:])))
                or min(steps) != -v.steer_right_deg or max(steps) != v.steer_left_deg):
            raise ValueError('steering map must be monotonic, center zero, and match vehicle steering limits')
        self.cameras = tuple(GroundCamera(raw[role], self.error) for role in ('front', 'rear'))

    def locate(self, images, prior):
        groups = [model.observe(frame, prior, self.radius, self.saturation, self.brightness)
                  for model, frame in zip(self.cameras, images)]
        pairs = groups[0] + groups[1]
        if not all(groups) or len({world for _, world in pairs}) < 3:
            raise ValueError('need visible paint corners in both cameras; at least three distinct total')
        local, world = zip(*pairs)
        return fit_pose(local, world, prior, self.error, self.jump, self.yaw_jump)


class ParkingFollower:
    def __init__(self, config):
        self.cfg = config
        self.reset()

    def reset(self):
        self.route = None
        self.index = 0
        self.progress = 0
        self.active_since = None
        self.pause = None
        self.status = 'WAIT_PLAN'

    def prepare(self, pose):
        self.reset()
        route = plan(pose, self.cfg.vehicle, self.cfg.scene)
        if route is None:
            self.status = 'NO_PATH'
        elif not route:
            self.status = 'PARKED'
        else:
            self.route = route
            self.status = 'READY'

    def stop(self, reason):
        self.reset()
        self.status = reason
        return MotionCommand()

    def _near_path(self, pose, reference):
        return (math.hypot(pose.x - reference.x, pose.y - reference.y) <= self.cfg.path_error
                and abs(wrap(pose.yaw - reference.yaw)) <= self.cfg.path_yaw)

    def step(self, pose, now):
        if self.route is None:
            return MotionCommand()
        cfg = self.cfg
        if not is_clear(pose, cfg.vehicle, cfg.scene, cfg.error):
            return self.stop('POSE_OUTSIDE_CLEAR_SPACE')
        move = self.route[self.index]
        last = len(move.poses) - 1
        if self.pause is not None:
            stopped_pose, then = self.pause
            if (math.hypot(pose.x - stopped_pose.x, pose.y - stopped_pose.y) > cfg.settle_drift
                    or abs(wrap(pose.yaw - stopped_pose.yaw)) > cfg.yaw_jump):
                return self.stop('MOVEMENT_DURING_STOP')
            if now - then < cfg.settle:
                return MotionCommand()
            if self.index == len(self.route) - 1:
                return self.stop('PARKED' if parked(pose, cfg.vehicle, cfg.scene) else 'FINAL_POSE_NOT_ALIGNED')
            self.pause = None
            self.index += 1
            self.progress = 0
            self.active_since = now
            move = self.route[self.index]
            last = len(move.poses) - 1
        if self.active_since is None:
            self.active_since = now
        elif now - self.active_since > cfg.move_timeout:
            return self.stop('SEGMENT_TIMEOUT')

        def error(i):
            sample = move.poses[i]
            return math.hypot(pose.x - sample.x, pose.y - sample.y)

        # Never jump backward on a segment or skip the initial left-steer action.
        self.progress = min(range(self.progress, last + 1), key=error)
        if not self._near_path(pose, move.poses[self.progress]):
            return self.stop('OUTSIDE_TRACKING_TOLERANCE')
        if self.progress == last:
            if self.index == len(self.route) - 1 or self.route[self.index + 1].direction != move.direction:
                self.pause = (pose, now)
                self.status = 'SETTLING_AT_STOP'
                return MotionCommand()
            self.index += 1
            self.progress = 0
            self.active_since = now
            move = self.route[self.index]
        target = move.poses[-1]
        dx, dy = target.x - pose.x, target.y - pose.y
        x, y = dx * math.cos(pose.yaw) + dy * math.sin(pose.yaw), -dx * math.sin(pose.yaw) + dy * math.cos(pose.yaw)
        steer = move.steering_deg
        if self.index > 0 and x * move.direction > cfg.error:
            steer = math.degrees(math.atan2(2 * cfg.vehicle.wheelbase * y, x * x + y * y))
        step = min(range(-7, 8), key=lambda i: (abs(cfg.steer_by_step[i + 7] - steer), abs(i)))
        curvature = math.tan(math.radians(cfg.steer_by_step[step + 7])) / cfg.vehicle.wheelbase
        # Check the direction and actual quantized steering through the worst
        # measured travel before a stop takes effect, not just the nominal path.
        distance = cfg.stop_distance
        radius = math.hypot(max(cfg.vehicle.wheelbase + cfg.vehicle.front_overhang, cfg.vehicle.rear_overhang),
                            cfg.vehicle.width / 2) + 2 * cfg.error
        extra = distance * (1 + abs(curvature) * radius) / 16
        for i in range(1, 9):
            look = advance(pose, move.direction * distance * i / 8, curvature)
            if not is_clear(look, cfg.vehicle, cfg.scene, cfg.error + extra):
                return self.stop('STOPPING_ENVELOPE_BLOCKED')
        pwm = cfg.forward_pwm if move.direction > 0 else -cfg.reverse_pwm
        self.status = f'{"REVERSE" if pwm < 0 else "FORWARD"} {self.index + 1}/{len(self.route)}'
        return MotionCommand(step, pwm, pwm)
