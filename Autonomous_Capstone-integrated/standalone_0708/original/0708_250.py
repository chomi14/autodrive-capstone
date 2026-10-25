#!/usr/bin/env python3
r"""
ROS-like standalone autonomous driver.

This file runs the original ROS 2 driving flow in one Python process:
  camera/video -> YOLO -> lane edge -> BEV/ROI -> target points
  -> CubicSpline path -> ROS motion_planner steering gain -> Arduino serial

Arduino serial command format:
  s{steering}l{left_speed}r{right_speed}\n

PowerShell dry-run:
  python .\ros_like_autonomous_driver.py --no-serial --camera 0

PowerShell with Arduino:
  python .\ros_like_autonomous_driver.py --port COM3 --camera 0
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass

cv2 = None
np = None
YOLO = None
CubicSpline = None

try:
    import serial
except ImportError:
    serial = None

# =========================
# EDIT HERE: run parameters
# =========================
DEFAULT_MODEL = r"C:\LaneProject\yijin_1.pt"
DEFAULT_PORT = "COM3"
DEFAULT_BAUDRATE = 9600
DEFAULT_CAMERA = 1
DEFAULT_IMGSZ = 256
DEFAULT_DEVICE = "cpu"
DEFAULT_CONF = 0.5
DEFAULT_INFER_EVERY = 1
DEFAULT_SERIAL_INTERVAL = 0.10
DEFAULT_SERIAL_WARMUP = 2.0
DEFAULT_START_DELAY = 0.0
DEFAULT_FRAME_SAVE_DIR = r"C:\LaneProject\captures"
DEFAULT_FRAME_SAVE_INTERVAL = 1.0

# =========================
# EDIT HERE: vision/path parameters
# =========================
WIDTH = 640
HEIGHT = 480
SRC_MAT_BASE = [[235, 316], [462, 313], [552, 476], [144, 476]]
CUTTING_IDX = 300
LANE_WIDTH = 295
THETA_LIMIT = 70
TARGET_Y_LIST = [5, 55, 105]
DETECTION_THICKNESS = 10

CAR_CENTER_POINT = (315, 170)


# =========================
# EDIT HERE: driving parameters
# =========================
STEERING_GAIN = 0.075
MAX_STEERING_STEP = 7
STEERING_COMMAND_QUANTUM = 1.0
DEFAULT_TARGET_LANE = "lane2"
NORMAL_SPEED = 250
CURVE_SLOWDOWN_DEFAULT = False
CURVE_SPEED = 250              # speed when steering is larger than CURVE_STEER_THRESHOLD
SHARP_CURVE_SPEED = 250        # speed when steering is larger than SHARP_CURVE_STEER_THRESHOLD
CURVE_STEER_THRESHOLD = 2.5    # lower value = slows down earlier
SHARP_CURVE_STEER_THRESHOLD = 4.0
LOW_SPEED = 150                # lane-change speed, not normal curve speed
LANE_CHANGE_SPEED_DURATION = 20.0
STEERING_MAX_DELTA_PER_FRAME = 2
LANE_MISS_SLOWDOWN_FRAMES = 15
LANE_MISS_STOP_FRAMES = 100
LANE_MISS_SPEED = 150


@dataclass
class TargetPoint:
    target_x: int
    target_y: int


@dataclass
class MotionCommand:
    steering: float
    left_speed: int
    right_speed: int


def clamp(value, low, high):
    return max(low, min(high, value))


def serial_message(cmd: MotionCommand) -> str:
    steering = clamp(float(cmd.steering), -MAX_STEERING_STEP, MAX_STEERING_STEP)
    left = clamp(int(cmd.left_speed), -255, 255)
    right = clamp(int(cmd.right_speed), -255, 255)
    return f"s{steering:.2f}l{left}r{right}\n"


class ArduinoSerial:
    def __init__(self, port, baudrate, no_serial=False, interval=0.10, warmup=1.0):
        self.no_serial = no_serial
        self.interval = float(interval)
        self.last_send = 0.0
        self.last_message = None
        self.ser = None

        if self.no_serial:
            print("[SERIAL] disabled")
            return

        if serial is None:
            raise RuntimeError("pyserial is missing. Install with: pip install pyserial")

        self.ser = serial.Serial(port, baudrate, timeout=0)
        time.sleep(max(0.0, float(warmup)))
        print(f"[SERIAL] connected {port} @ {baudrate}")

    def drain(self):
        if self.ser is None:
            return
        try:
            waiting = int(getattr(self.ser, "in_waiting", 0))
            if waiting > 0:
                self.ser.read(min(waiting, 4096))
        except Exception:
            pass

    def send(self, cmd: MotionCommand, force=False):
        msg = serial_message(cmd)
        now = time.time()
        if not force and now - self.last_send < self.interval:
            return

        self.last_message = msg
        self.last_send = now

        if self.no_serial:
            print("[DRY]", msg.strip())
            return

        self.drain()
        self.ser.write(msg.encode("ascii"))
        print("[SEND]", msg.strip())

    def stop(self):
        self.send(MotionCommand(0, 0, 0), force=True)

    def close(self):
        if self.ser is not None:
            self.ser.close()


def open_capture(args):
    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        if sys.platform.startswith("win"):
            cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap = cv2.VideoCapture(args.camera)
        else:
            cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        raise RuntimeError("Cannot open camera/video source")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def src_mat_with_offset(src_y_offset):
    src = SRC_MAT_BASE.copy()
    src[:, 1] = np.clip(src[:, 1] + float(src_y_offset), 0, HEIGHT - 1)
    return src.astype(np.float32)


def dominant_gradient(image, theta_limit=THETA_LIMIT):
    right_limit = np.deg2rad(90 + (90 - theta_limit))
    left_limit = np.deg2rad(90 - (90 - theta_limit))

    if image is None or image.size == 0 or image.max() == 0:
        return 0.0

    if image.dtype != np.uint8:
        image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype("uint8")

    width = image.shape[1]
    lines = cv2.HoughLines(image, 1, np.pi / 180, int(width * (25 / 640)))
    angles = []

    if lines is not None:
        for line in lines:
            for rho, theta in line:
                if left_limit < theta < right_limit:
                    continue

                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * a)
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * a)
                if y1 == y2:
                    continue
                angle = np.arctan((x2 - x1) / (y1 - y2)) * 180 / np.pi
                angles.append(angle)

    return float(np.median(angles)) if angles else 0.0


def bird_convert(img, src_y_offset=0):
    h, w = img.shape[:2]
    src_mat = src_mat_with_offset(src_y_offset)
    dst_mat = np.float32(
        [
            [round(w * 0.3), 0],
            [round(w * 0.7), 0],
            [round(w * 0.7), h],
            [round(w * 0.3), h],
        ]
    )
    transform = cv2.getPerspectiveTransform(src_mat, dst_mat)
    return cv2.warpPerspective(img, transform, (w, h))


def roi_rectangle_below(img, cutting_idx=CUTTING_IDX):
    return img[cutting_idx:]


def get_lane_center(cv_image, detection_height, detection_thickness, road_gradient, lane_width=LANE_WIDTH):
    upper = detection_height - int(detection_thickness / 2)
    lower = detection_height + int(detection_thickness / 2)
    upper = clamp(upper, 0, cv_image.shape[0] - 1)
    lower = clamp(lower, upper + 1, cv_image.shape[0])

    detected_x = np.sort(np.where(cv_image[upper:lower, :] != 0)[1])

    if detected_x.shape[0] < 5:
        return lane_width / 2

    cut_x = detected_x[1:-1]
    if cut_x.shape[0] < 2:
        return lane_width / 2

    diff = cut_x[1:] - cut_x[:-1]
    left_idx = int(np.argmax(diff))
    right_idx = left_idx + 1
    left_val = cut_x[left_idx]
    right_val = cut_x[right_idx]

    if abs(left_val - right_val) < (lane_width / 3):
        line_x_axis_pixel = cut_x[round(cut_x.shape[0] / 2)]
        center_pixel = None
    else:
        line_x_axis_pixel = None
        center_pixel = (left_val + right_val) / 2

    if center_pixel is None and line_x_axis_pixel is None:
        road_target_point_x = lane_width / 2
    else:
        road_target_point_x = center_pixel

        if road_target_point_x is None and line_x_axis_pixel is not None:
            if road_gradient > 0:
                road_target_point_x = line_x_axis_pixel + (lane_width / 2)
            else:
                road_target_point_x = line_x_axis_pixel - (lane_width / 2)

            if road_target_point_x > (lane_width - 1):
                road_target_point_x = lane_width - 1
            elif road_target_point_x < 0:
                road_target_point_x = 0

    return float(road_target_point_x)


def yolo_result_to_edge(frame_shape, result, cls_name):
    h, w = frame_shape[:2]
    edge_img = np.zeros((h, w), dtype=np.uint8)

    if result is None or result.masks is None or result.boxes is None:
        return edge_img

    for box, mask in zip(result.boxes, result.masks):
        cls_id = int(box.cls[0])
        detected_name = result.names[cls_id]
        if detected_name != cls_name:
            continue

        if mask.xy is None or len(mask.xy) == 0:
            continue

        for poly in mask.xy:
            points = poly.astype(np.int32)
            if len(points) > 0:
                cv2.polylines(edge_img, [points], isClosed=True, color=255, thickness=1, lineType=cv2.LINE_AA)

    return edge_img


def process_lane_edge(edge_img, src_y_offset=0):
    if edge_img is None or edge_img.max() == 0:
        return None, None, 0.0, []

    bird_img = bird_convert(edge_img, src_y_offset=src_y_offset)
    roi_img = roi_rectangle_below(bird_img, CUTTING_IDX)
    grad = dominant_gradient(roi_img, theta_limit=THETA_LIMIT)

    target_points = []
    for target_y in TARGET_Y_LIST:
        target_x = get_lane_center(
            roi_img,
            detection_height=target_y,
            detection_thickness=DETECTION_THICKNESS,
            road_gradient=grad,
            lane_width=LANE_WIDTH,
        )
        target_points.append(TargetPoint(round(target_x), round(target_y)))

    return bird_img, roi_img, grad, target_points


def plan_path(target_points):
    if len(target_points) < 3:
        return [], []

    x_points = [tp.target_x for tp in target_points]
    y_points = [tp.target_y for tp in target_points]

    x_points.append(CAR_CENTER_POINT[0])
    y_points.append(CAR_CENTER_POINT[1])

    sorted_points = sorted(zip(y_points, x_points), key=lambda point: point[0])
    y_points, x_points = zip(*sorted_points)

    y_new = np.linspace(min(y_points), max(y_points), 100)

    if CubicSpline is not None:
        try:
            cs = CubicSpline(y_points, x_points, bc_type="natural")
            x_new = cs(y_new)
            return list(x_new), list(y_new)
        except Exception as exc:
            print(f"[WARN] CubicSpline failed, fallback to np.interp: {exc}")

    x_new = np.interp(y_new, np.asarray(y_points, dtype=np.float32), np.asarray(x_points, dtype=np.float32))
    return list(x_new), list(y_new)


def calculate_slope_between_points(p1, p2):
    p1_x, p1_y = p1
    p2_x, p2_y = p2
    if p1_y == p2_y:
        return 0.0
    return np.arctan((p2_x - p1_x) / (p1_y - p2_y)) * 180 / np.pi


def calculate_steering_from_path(x_path, y_path):
    current_path = list(zip(x_path, y_path))
    if len(current_path) < 10:
        return 0.0

    slope = calculate_slope_between_points(current_path[-10], current_path[-1])
    steering = slope * STEERING_GAIN
    return float(clamp(steering, -MAX_STEERING_STEP, MAX_STEERING_STEP))


def calculate_steering_detail_from_path(x_path, y_path):
    current_path = list(zip(x_path, y_path))
    if len(current_path) < 10:
        return 0.0, 0.0, 0.0

    slope = calculate_slope_between_points(current_path[-10], current_path[-1])
    raw_float = slope * STEERING_GAIN
    return float(slope), float(raw_float), float(clamp(raw_float, -MAX_STEERING_STEP, MAX_STEERING_STEP))


def limit_steering_change(raw_steering, previous_steering, max_delta=None):
    if max_delta is None:
        max_delta = STEERING_MAX_DELTA_PER_FRAME
    delta = clamp(raw_steering - previous_steering, -max_delta, max_delta)
    return float(previous_steering + delta)


def quantize_steering(steering, quantum):
    quantum = float(quantum)
    if quantum <= 0:
        return float(steering)
    return round(float(steering) / quantum) * quantum


def choose_speed(lane_change_time):
    now = time.time()
    if now - lane_change_time <= LANE_CHANGE_SPEED_DURATION:
        return LOW_SPEED
    return NORMAL_SPEED


def apply_curve_slowdown(speed, steering, args):
    if not args.curve_slowdown:
        return speed

    abs_steering = abs(float(steering))
    if abs_steering >= args.sharp_curve_steer_threshold:
        return min(speed, int(args.sharp_curve_speed))
    if abs_steering >= args.curve_steer_threshold:
        return min(speed, int(args.curve_speed))
    return speed


def find_traffic_light_y_top(result):
    if result is None or result.boxes is None:
        return None, None

    for box in result.boxes:
        cls_id = int(box.cls[0])
        if result.names[cls_id] != "traffic_light":
            continue
        cx, cy, bw, bh = box.xywh[0].cpu().numpy()
        y_top = int(cy - bh / 2)
        x_avg = int(cx)
        return y_top, x_avg

    return None, None


def maybe_switch_lane(result, target_lane, last_lane_change_time, cooldown):
    y_top, x_avg = find_traffic_light_y_top(result)
    if y_top is None:
        return target_lane, last_lane_change_time, False

    should_change = (
        (10 < y_top < 100 and target_lane == "lane2")
        or (10 < y_top < 150 and x_avg is not None and x_avg > 185 and target_lane == "lane1")
    )

    now = time.time()
    if should_change and now - last_lane_change_time > cooldown:
        new_lane = "lane1" if target_lane == "lane2" else "lane2"
        return new_lane, now, True

    return target_lane, last_lane_change_time, False


def draw_debug(frame, lane_outputs, target_lane, steering, speed, src_y_offset):
    result = frame.copy()
    for lane_name, color in (("lane1", (0, 255, 0)), ("lane2", (0, 0, 255))):
        x_path = lane_outputs[lane_name]["x_path"]
        y_path = lane_outputs[lane_name]["y_path"]
        for i in range(max(0, len(x_path) - 1)):
            p1 = (int(x_path[i]), int(y_path[i] + CUTTING_IDX))
            p2 = (int(x_path[i + 1]), int(y_path[i + 1] + CUTTING_IDX))
            if 0 <= p1[0] < WIDTH and 0 <= p1[1] < HEIGHT and 0 <= p2[0] < WIDTH and 0 <= p2[1] < HEIGHT:
                cv2.line(result, p1, p2, color, 2)

    car = (WIDTH // 2, HEIGHT - 45)
    end = (int(car[0] + steering * 25), car[1] - 75)
    cv2.circle(result, car, 12, (170, 200, 255), -1)
    cv2.arrowedLine(result, car, end, (180, 170, 255), 4, tipLength=0.35)
    cv2.putText(result, f"lane={target_lane} steer={steering:+.2f} speed={speed} src_y_offset={src_y_offset}",
                (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    return result


def parse_args():
    parser = argparse.ArgumentParser(description="ROS-like one-file autonomous driver")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--camera", type=int, default=DEFAULT_CAMERA)
    parser.add_argument("--video", default=None)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF)
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    parser.add_argument("--infer-every", type=int, default=DEFAULT_INFER_EVERY)
    parser.add_argument("--steering-gain", type=float, default=STEERING_GAIN)
    parser.add_argument("--steering-max-delta", type=float, default=STEERING_MAX_DELTA_PER_FRAME)
    parser.add_argument("--steering-command-quantum", type=float, default=STEERING_COMMAND_QUANTUM,
                        help="Quantize serial steering command. Default 0.0 keeps true float steering.")
    parser.add_argument("--normal-speed", type=int, default=NORMAL_SPEED)
    parser.add_argument("--low-speed", type=int, default=LOW_SPEED)
    parser.add_argument("--lane-miss-speed", type=int, default=LANE_MISS_SPEED)
    parser.add_argument("--curve-slowdown", action="store_true", default=CURVE_SLOWDOWN_DEFAULT)
    parser.add_argument("--curve-speed", type=int, default=CURVE_SPEED)
    parser.add_argument("--sharp-curve-speed", type=int, default=SHARP_CURVE_SPEED)
    parser.add_argument("--curve-steer-threshold", type=float, default=CURVE_STEER_THRESHOLD)
    parser.add_argument("--sharp-curve-steer-threshold", type=float, default=SHARP_CURVE_STEER_THRESHOLD)
    parser.add_argument("--lane-width", type=int, default=LANE_WIDTH)
    parser.add_argument("--car-center-x", type=int, default=CAR_CENTER_POINT[0])
    parser.add_argument("--car-center-y", type=int, default=CAR_CENTER_POINT[1])
    parser.add_argument("--target-lane", choices=["lane1", "lane2"], default=DEFAULT_TARGET_LANE)
    parser.add_argument("--src-y-offset", type=float, default=0.0,
                        help="BEV source-point vertical offset. Use if camera points more forward.")
    parser.add_argument("--enable-lane-change", action="store_true",
                        help="Enable traffic-light based lane switching. Disabled by default.")
    parser.add_argument("--lane-change-cooldown", type=float, default=10.0)
    parser.add_argument("--serial-interval", type=float, default=DEFAULT_SERIAL_INTERVAL)
    parser.add_argument("--serial-warmup", type=float, default=DEFAULT_SERIAL_WARMUP)
    parser.add_argument("--start-delay", type=float, default=DEFAULT_START_DELAY)
    parser.add_argument("--launch-burst", type=float, default=0.8,
                        help="Send a straight forward command briefly after start.")
    parser.add_argument("--left-speed-sign", type=int, choices=[-1, 1], default=1)
    parser.add_argument("--right-speed-sign", type=int, choices=[-1, 1], default=1)
    parser.add_argument("--save-camera-frames", action="store_true",
                        help="Save the raw camera frame once per interval while running.")
    parser.add_argument("--frame-save-interval", type=float, default=DEFAULT_FRAME_SAVE_INTERVAL)
    parser.add_argument("--frame-save-dir", default=DEFAULT_FRAME_SAVE_DIR)
    parser.add_argument("--no-serial", action="store_true")
    parser.add_argument("--preview", dest="no_preview", action="store_false",
                        help="Show OpenCV preview windows.")
    parser.add_argument("--no-preview", dest="no_preview", action="store_true",
                        help="Run without OpenCV preview windows.")
    parser.set_defaults(no_preview=True)
    parser.add_argument("--wait-start", dest="wait_start", action="store_true",
                        help="Wait for Enter before driving.")
    parser.add_argument("--no-wait-start", dest="wait_start", action="store_false",
                        help="Start driving immediately without waiting for Enter.")
    parser.set_defaults(wait_start=True)
    return parser.parse_args()


def main():
    boot_time = time.time()
    args = parse_args()

    global cv2, np, YOLO, CubicSpline, SRC_MAT_BASE
    global STEERING_GAIN, STEERING_MAX_DELTA_PER_FRAME, NORMAL_SPEED, LOW_SPEED, LANE_MISS_SPEED
    global LANE_WIDTH, CAR_CENTER_POINT
    try:
        import cv2 as cv2_module
        import numpy as np_module
        from ultralytics import YOLO as yolo_class
    except ImportError as exc:
        raise RuntimeError(
            "Missing dependency. Install with: "
            "pip install ultralytics opencv-python numpy pyserial scipy"
        ) from exc

    try:
        from scipy.interpolate import CubicSpline as cubic_spline_class
    except ImportError:
        cubic_spline_class = None
        print("[WARN] scipy is missing. Using np.interp fallback instead of CubicSpline.")

    cv2 = cv2_module
    np = np_module
    YOLO = yolo_class
    CubicSpline = cubic_spline_class
    SRC_MAT_BASE = np.float32([[235, 316], [462, 313], [552, 476], [144, 476]])
    STEERING_GAIN = float(args.steering_gain)
    STEERING_MAX_DELTA_PER_FRAME = max(0.1, float(args.steering_max_delta))
    NORMAL_SPEED = int(args.normal_speed)
    LOW_SPEED = int(args.low_speed)
    LANE_MISS_SPEED = int(args.lane_miss_speed)
    LANE_WIDTH = int(args.lane_width)
    CAR_CENTER_POINT = (int(args.car_center_x), int(args.car_center_y))

    if YOLO is None:
        raise RuntimeError("ultralytics is missing. Install with: pip install ultralytics")
    if not os.path.exists(args.model):
        raise RuntimeError(f"Model not found: {args.model}")

    print("[INFO] ROS-like params")
    print(f"  SRC_MAT={SRC_MAT_BASE.tolist()}, src_y_offset={args.src_y_offset}")
    print(f"  CUTTING_IDX={CUTTING_IDX}, CAR_CENTER_POINT={CAR_CENTER_POINT}")
    print(f"  steering = round(slope_deg * {STEERING_GAIN}), clamp +/-{MAX_STEERING_STEP}")

    print(f"[TIME] before model load: {time.time() - boot_time:.2f}s")
    model = YOLO(args.model)
    print(f"[TIME] after model load: {time.time() - boot_time:.2f}s")
    print("[INFO] model names:", model.names)

    print(f"[TIME] before camera open: {time.time() - boot_time:.2f}s")
    cap = open_capture(args)
    print(f"[TIME] after camera open: {time.time() - boot_time:.2f}s")
    print(f"[TIME] before serial open: {time.time() - boot_time:.2f}s")
    arduino = ArduinoSerial(
        args.port,
        args.baudrate,
        no_serial=args.no_serial,
        interval=args.serial_interval,
        warmup=args.serial_warmup,
    )
    print(f"[TIME] after serial open: {time.time() - boot_time:.2f}s")

    target_lane = args.target_lane
    last_lane_change_time = 0.0
    last_result = None
    frame_count = 0
    last_steering = 0.0
    smoothed_steering = 0.0
    missed_lane_frames = 0
    last_frame_save_time = 0.0

    if args.save_camera_frames:
        os.makedirs(args.frame_save_dir, exist_ok=True)
        print(f"[CAPTURE] saving camera frames to {args.frame_save_dir}")

    try:
        if args.wait_start:
            print(f"[READY] setup complete at {time.time() - boot_time:.2f}s")
            print("[READY] press Enter to start driving")
            arduino.stop()
            input()
            print(f"[START] drive command released at {time.time() - boot_time:.2f}s")

        if args.start_delay > 0:
            print(f"[START] sending stop for {args.start_delay:.1f}s")
            until = time.time() + args.start_delay
            while time.time() < until:
                arduino.stop()
                time.sleep(0.1)

        if args.launch_burst > 0:
            launch_speed = NORMAL_SPEED
            print(f"[START] launch burst s0l{launch_speed}r{launch_speed} for {args.launch_burst:.1f}s")
            until = time.time() + float(args.launch_burst)
            while time.time() < until:
                arduino.send(MotionCommand(0, launch_speed, launch_speed), force=True)
                time.sleep(0.1)
        print(f"[TIME] entering drive loop: {time.time() - boot_time:.2f}s")

        while True:
            ok, frame = cap.read()
            if not ok:
                if args.video:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                print("[WARN] frame read failed")
                arduino.stop()
                time.sleep(0.1)
                continue

            frame = cv2.resize(frame, (WIDTH, HEIGHT))
            frame_count += 1
            now = time.time()

            if args.save_camera_frames and now - last_frame_save_time >= args.frame_save_interval:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"camera_{timestamp}_{frame_count:06d}.jpg"
                cv2.imwrite(os.path.join(args.frame_save_dir, filename), frame)
                last_frame_save_time = now

            if last_result is None or frame_count % max(1, args.infer_every) == 0:
                last_result = model.predict(
                    frame,
                    conf=args.conf,
                    device=args.device,
                    imgsz=args.imgsz,
                    verbose=False,
                )[0].cpu()

            result = last_result
            if args.enable_lane_change:
                target_lane, last_lane_change_time, changed = maybe_switch_lane(
                    result, target_lane, last_lane_change_time, args.lane_change_cooldown
                )
                if changed:
                    print(f"[LANE] changed to {target_lane}")

            lane_outputs = {}
            for lane_name in ("lane1", "lane2"):
                edge = yolo_result_to_edge(frame.shape, result, lane_name)
                _, roi, grad, target_points = process_lane_edge(edge, src_y_offset=args.src_y_offset)
                x_path, y_path = plan_path(target_points)
                lane_outputs[lane_name] = {
                    "edge": edge,
                    "roi": roi,
                    "grad": grad,
                    "target_points": target_points,
                    "x_path": x_path,
                    "y_path": y_path,
                }

            target = lane_outputs[target_lane]
            slope = 0.0
            raw_float = 0.0
            if len(target["x_path"]) >= 10:
                slope, raw_float, raw_steering = calculate_steering_detail_from_path(
                    target["x_path"],
                    target["y_path"],
                )
                steering = limit_steering_change(raw_steering, smoothed_steering)
                smoothed_steering = steering
                last_steering = steering
                missed_lane_frames = 0
            else:
                missed_lane_frames += 1
                steering = last_steering
                smoothed_steering = steering

            if missed_lane_frames >= LANE_MISS_STOP_FRAMES:
                speed = 0
            elif missed_lane_frames > LANE_MISS_SLOWDOWN_FRAMES:
                speed = LANE_MISS_SPEED
            else:
                speed = choose_speed(last_lane_change_time)
                speed = apply_curve_slowdown(speed, steering, args)
            command_steering = quantize_steering(steering, args.steering_command_quantum)
            cmd = MotionCommand(command_steering, speed * args.left_speed_sign, speed * args.right_speed_sign)
            arduino.send(cmd)

            print(
                f"lane={target_lane} steering={steering:+.2f} cmd_steer={command_steering:+.2f} speed={speed} "
                f"raw={raw_float:+.2f} slope={slope:+.1f} "
                f"cmd_l={cmd.left_speed} cmd_r={cmd.right_speed} "
                f"missed={missed_lane_frames} points={len(target['target_points'])}"
            )

            if not args.no_preview:
                debug = draw_debug(frame, lane_outputs, target_lane, steering, speed, args.src_y_offset)
                cv2.imshow("ROS-like autonomous driver", debug)
                roi = target["roi"]
                if roi is not None:
                    cv2.imshow(f"{target_lane}_roi", roi)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

    except KeyboardInterrupt:
        print("\n[STOP] interrupted")
    finally:
        arduino.stop()
        arduino.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
