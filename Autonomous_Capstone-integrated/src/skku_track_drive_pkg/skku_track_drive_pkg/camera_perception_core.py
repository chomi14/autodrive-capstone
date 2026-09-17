import cv2
import math
import numpy as np
from typing import Tuple
from collections import namedtuple

# 차선 중심 추정 결과.
#   center       : 추정한 차선 중심 x (정수)
#   edges        : 실제로 검출된 차선 경계 x 목록(1~2개) — 시각화용(초록)
#   virtual_x    : 경계에서 잘려 가상으로 가정한 차선 x (없으면 None) — 시각화용(주황)
#   reconstructed: 가상 차선폭으로 복원했는지 여부
LaneCenter = namedtuple("LaneCenter", ["center", "edges", "virtual_x", "reconstructed"])


def _weighted_median(values, weights):
    """길이(weight)로 가중한 중앙값. 평균보다 이상치(잘못 검출된 선)에 강하다."""
    pairs = sorted(zip(values, weights))
    total = sum(w for _, w in pairs)
    if total <= 0:
        return 0.0
    acc, half = 0.0, total / 2.0
    for v, w in pairs:
        acc += w
        if acc >= half:
            return v
    return pairs[-1][0]


def dominant_gradient(image, theta_limit):
    """
    ROI 이진 영상에서 차선의 지배적 기울기(도)를 추정한다.

    개선점:
      - HoughLines -> HoughLinesP(확률적): 선분 끝점을 직접 얻어 각도 계산이 안정적이고,
        곡선 차선을 짧은 직선 세그먼트들로 잘게 잡아낸다.
      - 선분 길이로 가중한 가중 중앙값: 짧은 노이즈 세그먼트의 영향을 줄이고 이상치에 강하다.
      - 차선 픽셀 수에 따른 적응형 임계값.
    반환 각도 부호는 기존과 호환: 세로선=0도, 오른쪽으로 기울면 +, 왼쪽이면 -.
    """
    if image.dtype != np.uint8:
        image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype("uint8")

    try:
        h, w = image.shape[:2]
        if int(cv2.countNonZero(image)) < 20:
            return 0.0

        # 적응형 파라미터: 곡선 차선은 짧은 세그먼트로 쪼개지므로 minLineLength를 과하게 잡지 않는다.
        threshold = max(12, int(w * 20 / 640))
        min_len = max(10, int(h * 0.12))
        max_gap = max(5, int(h * 0.15))

        lines = cv2.HoughLinesP(image, 1, np.pi / 180, threshold,
                                minLineLength=min_len, maxLineGap=max_gap)
        if lines is None:
            return 0.0

        angles, weights = [], []
        for x1, y1, x2, y2 in lines[:, 0]:
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length = math.hypot(dx, dy)
            if length < 1.0 or abs(dy) < 1e-6:
                continue
            # 세로축 기준 각도(도). 끝점 순서가 바뀌어도 부호 불변(atan(dx / -dy)).
            angle = math.degrees(math.atan(dx / -dy))
            if abs(angle) > theta_limit:
                continue  # 너무 수평인 선은 차선이 아니므로 제외
            angles.append(angle)
            weights.append(length)

        if not angles:
            return 0.0
        return float(_weighted_median(angles, weights))
    except Exception as e:
        print(f"[Lane] gradient detection error: {e}")
        return 0.0


def bird_convert(img, srcmat, dstmat):
    h, w = img.shape[:2]
    transform_matrix = cv2.getPerspectiveTransform(np.float32(srcmat), np.float32(dstmat))
    return cv2.warpPerspective(img, transform_matrix, (w, h))


def roi_rectangle_below(img, cutting_idx):
    return img[cutting_idx:]


def draw_edge(cv_image: np.ndarray, detection, color: Tuple[int]) -> np.ndarray:
    mask_msg = detection.mask
    if not mask_msg.data:
        return cv_image
    mask_array = np.array([[int(ele.x), int(ele.y)] for ele in mask_msg.data])
    return cv2.polylines(cv_image, [mask_array], isClosed=True, color=color, thickness=1, lineType=cv2.LINE_AA)


def draw_edges(detection_msg, cls_name: str, color=255):
    # lane segmentation mask가 없으면 빈 이미지 반환
    first_mask = None
    for det in detection_msg.detections:
        if det.mask.height > 0 and det.mask.width > 0:
            first_mask = det.mask
            break
    if first_mask is None:
        return np.zeros((480, 640), dtype=np.uint8)

    cv_image = np.zeros((first_mask.height, first_mask.width), dtype=np.uint8)
    for detection in detection_msg.detections:
        if detection.class_name == cls_name:
            cv_image = draw_edge(cv_image, detection, color=color)
    return cv_image


def get_lane_center(cv_image: np.ndarray, detection_height: int, detection_thickness: int,
                    road_gradient: float, lane_width: int,
                    virtual_lane_width: int = None, boundary_margin: int = 8) -> LaneCenter:
    """
    한 수평 밴드에서 차선 중심 x를 추정한다.

    코너링 시 한쪽 차선의 상단부가 BEV 창 경계에서 잘리면, 그 잘린 쪽 차선을 보이는
    반대쪽 차선으로부터 'virtual_lane_width'(가상 차선 폭, px)만큼 떨어진 곳에 있다고
    가정해 중심을 복원한다. 이렇게 하면 기울기 부호 추정이 흔들려도 중심이 튀지 않는다.

    반환: LaneCenter(center, edges, virtual_x, reconstructed)
    """
    if virtual_lane_width is None or virtual_lane_width <= 0:
        virtual_lane_width = lane_width
    half_vw = virtual_lane_width / 2.0

    h, w = cv_image.shape[:2]
    upper = max(0, detection_height - int(detection_thickness / 2))
    lower = min(h, detection_height + int(detection_thickness / 2))
    xs = np.sort(np.where(cv_image[upper:lower, :] != 0)[1])

    if xs.shape[0] < 5:
        # 정보 없음 -> 영상 중앙
        return LaneCenter(int(w // 2), [], None, False)

    cut = xs[1:-1] if xs.shape[0] > 2 else xs
    diff = np.diff(cut) if cut.shape[0] > 1 else np.array([0])
    gi = int(np.argmax(diff)) if diff.size else 0
    left_val = int(cut[gi])
    right_val = int(cut[min(gi + 1, cut.shape[0] - 1)])

    left_touch = int(xs[0]) <= boundary_margin
    right_touch = int(xs[-1]) >= (w - 1 - boundary_margin)
    two_clusters = (right_val - left_val) >= (virtual_lane_width / 3.0)

    def clamp(v):
        # 가상 차선/중심이 창 밖으로 뻗을 수 있도록 넓은 범위 허용 → 가상선의 길이를 반영
        return int(max(-w, min(2 * w - 1, v)))

    # 1) 양쪽 차선이 명확히 보이고 경계에 안 닿음 -> 실제 중앙
    if two_clusters and not left_touch and not right_touch:
        return LaneCenter(clamp((left_val + right_val) / 2.0), [left_val, right_val], None, False)

    # 2) 한쪽이 경계에 닿아 잘림 -> 보이는 반대쪽 차선 기준으로 가상 차선 가정
    if two_clusters and left_touch and not right_touch:
        # 왼쪽이 잘림: 오른쪽(right_val)이 실제 차선, 왼쪽을 가상으로
        return LaneCenter(clamp(right_val - half_vw), [right_val], clamp(right_val - virtual_lane_width), True)
    if two_clusters and right_touch and not left_touch:
        return LaneCenter(clamp(left_val + half_vw), [left_val], clamp(left_val + virtual_lane_width), True)

    # 3) 단일 차선만 보임 -> 누락 방향 결정 후 가상 차선 가정
    visible = int(cut[cut.shape[0] // 2])
    if left_touch and not right_touch:
        missing_left = True
    elif right_touch and not left_touch:
        missing_left = False
    else:
        # 경계 접촉이 없으면 기울기 부호로 결정(기존 관례: gradient<=0 -> 왼쪽 누락)
        missing_left = (road_gradient <= 0)

    if missing_left:
        return LaneCenter(clamp(visible - half_vw), [visible], clamp(visible - virtual_lane_width), True)
    return LaneCenter(clamp(visible + half_vw), [visible], clamp(visible + virtual_lane_width), True)


def get_traffic_light_color(cv_image: np.ndarray, bbox, hsv_ranges: dict) -> str:
    x_min = max(0, int(bbox.center.position.x - bbox.size.x / 2))
    x_max = min(cv_image.shape[1], int(bbox.center.position.x + bbox.size.x / 2))
    y_min = max(0, int(bbox.center.position.y - bbox.size.y / 2))
    y_max = min(cv_image.shape[0], int(bbox.center.position.y + bbox.size.y / 2))
    roi = cv_image[y_min:y_max, x_min:x_max]
    if roi.size == 0:
        return "Unknown"

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red_mask1 = cv2.inRange(hsv_roi, hsv_ranges["red1"][0], hsv_ranges["red1"][1])
    red_mask2 = cv2.inRange(hsv_roi, hsv_ranges["red2"][0], hsv_ranges["red2"][1])
    yellow_mask = cv2.inRange(hsv_roi, hsv_ranges["yellow"][0], hsv_ranges["yellow"][1])
    green_mask = cv2.inRange(hsv_roi, hsv_ranges["green"][0], hsv_ranges["green"][1])

    denom = roi.size / 3
    ratios = {
        "Red": cv2.countNonZero(red_mask1 + red_mask2) / denom,
        "Yellow": cv2.countNonZero(yellow_mask) / denom,
        "Green": cv2.countNonZero(green_mask) / denom,
    }
    color, ratio = max(ratios.items(), key=lambda kv: kv[1])
    return color if ratio > 0.01 else "Unknown"
