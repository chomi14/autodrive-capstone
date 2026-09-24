import cv2
import numpy as np
from .messages import LaneInfo, TargetPoint
from . import camera_perception_core as CPFL


class LaneInfoExtractor:
    """
    lane2 마스크 -> Bird-Eye View -> ROI -> 차선 중심점 추출.

    실시간 튜닝을 위해 아래 값들을 인스턴스 속성으로 노출한다(드라이버가 매 프레임 갱신 가능):
      - bev_top_shift : BEV 원근 변환 상단 기준선의 수직 이동량(px).
                        음수면 더 먼 곳(지평선 쪽)을 포함 → 더 위에서 내려다보는 느낌.
      - roi_cut       : BEV 영상에서 잘라낼 상단 경계(px). 클수록 더 아래(가까운 곳)만 본다.
      - look_shift    : 차선 중심 샘플링 높이 범위의 이동량(px). 음수=더 위(먼 곳), 양수=더 아래(가까운 곳).
      - virtual_lane_width : 코너링 시 한쪽 차선이 창 경계에서 잘릴 때, 보이는 차선으로부터
                        이 폭(px)만큼 떨어진 곳에 반대쪽 차선이 있다고 가정해 중심을 복원한다.
    """

    # 원본 영상 기준으로 보정된 BEV 소스 사다리꼴(기본값)
    BASE_SRC = [[238, 316], [402, 313], [501, 476], [155, 476]]

    def __init__(self, show_image: bool = True, bev_top_shift: int = 0,
                 roi_cut: int = 300, look_shift: int = 0, slope_ema_alpha: float = 0.3,
                 virtual_lane_width: int = 300, bev_pad: int = 0,
                 capture_debug: bool = False, center_ema_alpha: float = 0.35,
                 max_center_jump_px: float = 80.0, max_missed_frames: int = 12,
                 min_component_area: int = 250):
        self.show_image = show_image
        self.capture_debug = capture_debug
        self.last_debug = None
        self.bev_top_shift = bev_top_shift
        self.roi_cut = roi_cut
        self.look_shift = look_shift
        # 현재 관측의 가중치. 작을수록 평활화가 강하고 1이면 즉시 반영한다.
        self.slope_ema_alpha = slope_ema_alpha
        self.smoothed_slope = None
        # 코너링 시 잘린 차선을 복원할 때 쓰는 가상 차선 폭(px). 실시간 조절 가능.
        self.virtual_lane_width = virtual_lane_width
        # BEV 캔버스를 좌우로 넓히는 패딩(px, 한쪽). 코너에서 창 밖으로 나가 잘리던 차선을
        # 되살리고, 가상 차선이 보일 공간을 만든다. 좌표가 +bev_pad 이동하므로 차량 중심도
        # 같이 이동해야 한다(드라이버가 motion.car_center_x = 원래중심+bev_pad 로 설정).
        self.bev_pad = bev_pad
        self.center_ema_alpha = center_ema_alpha
        self.max_center_jump_px = max_center_jump_px
        self.max_missed_frames = max(0, int(max_missed_frames))
        self.min_component_area = max(1, int(min_component_area))
        self.smoothed_targets = None
        self.smoothed_target_ys = None
        self.smoothed_fit = None
        self.missed_frames = 0
        self.last_confidence = 0.0

    def _src_mat(self):
        s = [list(p) for p in self.BASE_SRC]
        # 상단 두 점(인덱스 0,1)의 y를 bev_top_shift만큼 이동 → 내려다보는 정도 조절
        s[0][1] += self.bev_top_shift
        s[1][1] += self.bev_top_shift
        return s

    def process(self, detections, frame=None) -> LaneInfo:
        if self.capture_debug:
            self.last_debug = None
        if len(detections.detections) == 0:
            return self._held_lane_info()

        lane2_mask = CPFL.draw_edges(detections, cls_name="lane2", color=255)
        h, w = lane2_mask.shape[:2]

        # 계산은 원래 640 캔버스에서 수행한다. 그래야 경계(x=0, x=w-1) 접촉 판정이
        # 원래대로 동작해 잘린 차선의 가상 복원이 정확히 작동한다.
        # (bev_pad는 시각화 캔버스를 좌우로 넓히는 데에만 쓴다.)
        dst_mat = np.float32([[round(w * 0.3), 0], [round(w * 0.7), 0],
                              [round(w * 0.7), h], [round(w * 0.3), h]])
        src_mat = np.float32(self._src_mat())
        M = cv2.getPerspectiveTransform(src_mat, dst_mat)

        # Pixels outside the calibrated source trapezoid can be projected into
        # the driving ROI as large false regions. Mask them before the warp and
        # carry a separate valid-region mask through the same homography.
        source_valid = np.zeros((h, w), dtype=np.uint8)
        cv2.fillConvexPoly(source_valid, np.rint(src_mat).astype(np.int32), 255)
        lane2_mask = cv2.bitwise_and(lane2_mask, source_valid)
        lane2_bird_image = cv2.warpPerspective(
            lane2_mask, M, (w, h), flags=cv2.INTER_NEAREST
        )
        valid_bird = cv2.warpPerspective(
            source_valid, M, (w, h), flags=cv2.INTER_NEAREST
        )

        roi_cut = int(max(0, min(h - 10, self.roi_cut)))
        valid_roi = cv2.erode(
            valid_bird[roi_cut:], np.ones((3, 3), dtype=np.uint8), iterations=1
        )
        roi_image = CPFL.clean_lane_mask(
            lane2_bird_image[roi_cut:],
            min_component_area=self.min_component_area,
        )
        roi_image[valid_roi == 0] = 0

        # BEV 원근 사다리꼴의 상/하단 폭. 윗변(먼 곳)은 좁고 아랫변(가까운 곳)은 넓다.
        # 직사각형으로 잡아당기면 위로 갈수록 더 크게 확대되므로, 가상 차선 폭도 그만큼 늘린다.
        src = self._src_mat()
        top_w = abs(src[1][0] - src[0][0])      # 사다리꼴 윗변 폭(좁음)
        bottom_w = abs(src[2][0] - src[3][0])   # 사다리꼴 아랫변 폭(넓음) — 기준
        if top_w <= 0:
            top_w = bottom_w

        roi_h = roi_image.shape[0]
        row_observations = []
        for target_y in range(3, max(4, roi_h - 2), 6):
            previous_center = self._previous_center_at(target_y)
            virtual_width = self._virtual_width(
                target_y, roi_cut, h, top_w, bottom_w
            )
            observation = self._row_center(
                roi_image,
                target_y,
                virtual_width,
                previous_center,
                valid_roi,
            )
            if observation.valid:
                row_observations.append((target_y, observation))

        fit, fit_confidence = self._fit_centerline(row_observations, roi_h)
        if fit is None:
            self._capture_debug(
                lane2_bird_image, roi_image, roi_cut, dst_mat, src_mat, []
            )
            return self._held_lane_info()

        target_ys = [
            int(max(0, min(roi_h - 1, base_y + self.look_shift)))
            for base_y in range(5, 155, 50)
        ]
        raw_targets = [float(np.polyval(fit, target_y)) for target_y in target_ys]
        smoothed_targets = self._smooth_targets(raw_targets, target_ys)
        target_points = [
            TargetPoint(target_x=int(round(target_x)), target_y=target_y)
            for target_x, target_y in zip(smoothed_targets, target_ys)
        ]

        raw_slope = self._slope_from_fit(fit, target_ys[-1])
        if self.smoothed_slope is None:
            self.smoothed_slope = raw_slope
        else:
            a = min(1.0, max(0.0, float(self.slope_ema_alpha)))
            self.smoothed_slope = a * raw_slope + (1.0 - a) * self.smoothed_slope
        grad = self.smoothed_slope

        observation_by_y = {target_y: item for target_y, item in row_observations}
        samples = []
        for target_x, target_y in zip(smoothed_targets, target_ys):
            nearest_y = min(
                observation_by_y,
                key=lambda row_y: abs(row_y - target_y),
            )
            observed = observation_by_y[nearest_y]
            samples.append((
                target_y,
                CPFL.LaneCenter(
                    center=int(round(target_x)),
                    edges=observed.edges,
                    virtual_x=observed.virtual_x,
                    reconstructed=observed.reconstructed,
                    valid=True,
                    source=observed.source,
                ),
            ))

        self.missed_frames = 0
        self.last_confidence = fit_confidence

        self._capture_debug(
            lane2_bird_image, roi_image, roi_cut, dst_mat, src_mat, samples,
            confidence=fit_confidence,
            source="observed",
        )

        if self.show_image:
            pad = int(max(0, self.bev_pad))
            if frame is not None:
                # 원본 컬러 프레임을 같은 변환으로 BEV로 편 뒤, 좌우로 pad만큼 넓힌 캔버스에 얹어
                # 창 밖으로 뻗는 가상 차선까지 보여준다(시각화 전용 — 계산은 640에서 끝남).
                color_roi = cv2.warpPerspective(frame, M, (w, h))[roi_cut:]
                wide = cv2.copyMakeBorder(color_roi, 0, 0, pad, pad, cv2.BORDER_CONSTANT, value=(0, 0, 0))
                cv2.imshow("lane_bev_color", self._annotate(wide, samples, pad, w))
            else:
                roi_bgr = cv2.cvtColor(roi_image, cv2.COLOR_GRAY2BGR)
                wide = cv2.copyMakeBorder(roi_bgr, 0, 0, pad, pad, cv2.BORDER_CONSTANT, value=(0, 0, 0))
                cv2.imshow("lane_detect", self._annotate(wide, samples, pad, w))
            cv2.waitKey(1)

        return LaneInfo(
            slope=grad,
            target_points=target_points,
            valid=True,
            confidence=fit_confidence,
            source="observed",
        )

    def _virtual_width(self, target_y, roi_cut, height, top_width, bottom_width):
        bev_row = roi_cut + target_y
        fraction = min(1.0, max(0.0, bev_row / float(height)))
        source_width = top_width + (bottom_width - top_width) * fraction
        return int(round(
            self.virtual_lane_width * (bottom_width / max(1.0, source_width))
        ))

    @staticmethod
    def _contiguous_runs(xs):
        if len(xs) == 0:
            return []
        split_indices = np.flatnonzero(np.diff(xs) > 1) + 1
        return [part for part in np.split(xs, split_indices) if len(part) > 0]

    def _row_center(
        self, mask, target_y, virtual_width, previous_center, valid_mask=None
    ):
        h, w = mask.shape[:2]
        upper = max(0, int(target_y) - 2)
        lower = min(h, int(target_y) + 3)
        xs = np.flatnonzero(np.any(mask[upper:lower] > 0, axis=0))
        runs = self._contiguous_runs(xs)
        if not runs:
            return CPFL.LaneCenter(
                center=w // 2, edges=[], valid=False, source="invalid"
            )

        valid_left = 0
        valid_right = w - 1
        if valid_mask is not None and valid_mask.size > 0:
            valid_xs = np.flatnonzero(
                np.any(valid_mask[upper:lower] > 0, axis=0)
            )
            if len(valid_xs) == 0:
                return CPFL.LaneCenter(
                    center=w // 2, edges=[], valid=False, source="invalid"
                )
            valid_left = int(valid_xs[0])
            valid_right = int(valid_xs[-1])

        candidates = []
        for run in runs:
            left = int(run[0])
            right = int(run[-1])
            run_width = right - left + 1
            left_touch = left <= valid_left + 3
            right_touch = right >= valid_right - 3
            half_width = virtual_width / 2.0

            if left_touch and not right_touch:
                center = right - half_width
                result = CPFL.LaneCenter(
                    center=int(round(center)), edges=[right],
                    virtual_x=int(round(right - virtual_width)),
                    reconstructed=True, source="right_edge",
                )
            elif right_touch and not left_touch:
                center = left + half_width
                result = CPFL.LaneCenter(
                    center=int(round(center)), edges=[left],
                    virtual_x=int(round(left + virtual_width)),
                    reconstructed=True, source="left_edge",
                )
            elif run_width >= max(12.0, virtual_width * 0.25):
                center = (left + right) / 2.0
                result = CPFL.LaneCenter(
                    center=int(round(center)), edges=[left, right],
                    reconstructed=False, source="two_edges",
                )
            elif previous_center is not None:
                visible = (left + right) / 2.0
                candidate_centers = (
                    (visible - half_width, "right_edge"),
                    (visible + half_width, "left_edge"),
                )
                center, source = min(
                    candidate_centers,
                    key=lambda item: abs(item[0] - previous_center),
                )
                virtual_x = (
                    visible - virtual_width
                    if source == "right_edge"
                    else visible + virtual_width
                )
                result = CPFL.LaneCenter(
                    center=int(round(center)), edges=[int(round(visible))],
                    virtual_x=int(round(virtual_x)), reconstructed=True,
                    source=source,
                )
            else:
                continue

            distance = (
                abs(result.center - previous_center)
                if previous_center is not None
                else 0.0
            )
            candidates.append((distance, -run_width, result))

        if not candidates:
            return CPFL.LaneCenter(
                center=w // 2, edges=[], valid=False, source="ambiguous"
            )
        return min(candidates, key=lambda item: (item[0], item[1]))[2]

    @staticmethod
    def _fit_centerline(row_observations, roi_height):
        if len(row_observations) < 8:
            return None, 0.0
        ys = np.asarray([row_y for row_y, _ in row_observations], dtype=np.float64)
        xs = np.asarray(
            [observation.center for _, observation in row_observations],
            dtype=np.float64,
        )
        span_ratio = float((ys.max() - ys.min()) / max(1.0, roi_height - 1.0))
        if span_ratio < 0.25:
            return None, 0.0
        try:
            fit = np.polyfit(ys, xs, deg=2)
            residuals = np.abs(np.polyval(fit, ys) - xs)
            keep = residuals <= 30.0
            if int(np.count_nonzero(keep)) >= 8:
                fit = np.polyfit(ys[keep], xs[keep], deg=2)
                residuals = np.abs(np.polyval(fit, ys[keep]) - xs[keep])
            median_residual = float(np.median(residuals)) if len(residuals) else 30.0
        except np.linalg.LinAlgError:
            return None, 0.0

        row_ratio = min(1.0, len(row_observations) / max(1.0, roi_height / 6.0))
        residual_score = max(0.0, 1.0 - median_residual / 30.0)
        confidence = min(
            1.0,
            0.45 * min(1.0, span_ratio / 0.70)
            + 0.35 * row_ratio
            + 0.20 * residual_score,
        )
        return fit, float(confidence)

    def _smooth_targets(self, raw_targets, target_ys):
        if (
            self.smoothed_targets is None
            or self.smoothed_target_ys != list(target_ys)
            or len(self.smoothed_targets) != len(raw_targets)
        ):
            smoothed = list(raw_targets)
        else:
            alpha = min(1.0, max(0.0, float(self.center_ema_alpha)))
            max_jump = max(1.0, float(self.max_center_jump_px))
            smoothed = []
            for previous, current in zip(self.smoothed_targets, raw_targets):
                innovation = max(-max_jump, min(max_jump, current - previous))
                smoothed.append(previous + alpha * innovation)

        self.smoothed_targets = smoothed
        self.smoothed_target_ys = list(target_ys)
        degree = min(2, len(target_ys) - 1)
        self.smoothed_fit = np.polyfit(target_ys, smoothed, deg=degree)
        return smoothed

    def _previous_center_at(self, target_y):
        if self.smoothed_fit is None:
            return None
        return float(np.polyval(self.smoothed_fit, target_y))

    @staticmethod
    def _slope_from_fit(fit, target_y):
        derivative = float(np.polyval(np.polyder(fit), target_y))
        return float(np.degrees(np.arctan(-derivative)))

    def _held_lane_info(self):
        self.missed_frames += 1
        if (
            self.smoothed_targets is None
            or self.smoothed_target_ys is None
            or self.missed_frames > self.max_missed_frames
        ):
            return LaneInfo()
        decay = max(0.0, 1.0 - self.missed_frames / (self.max_missed_frames + 1.0))
        targets = [
            TargetPoint(target_x=int(round(x)), target_y=int(y))
            for x, y in zip(self.smoothed_targets, self.smoothed_target_ys)
        ]
        return LaneInfo(
            slope=float(self.smoothed_slope or 0.0),
            target_points=targets,
            valid=True,
            confidence=self.last_confidence * decay,
            source="held",
        )

    def _capture_debug(
        self,
        bird_mask,
        cleaned_roi,
        roi_cut,
        dst_mat,
        src_mat,
        samples,
        confidence=0.0,
        source="invalid",
    ):
        if not self.capture_debug:
            return
        cleaned_full = np.zeros_like(bird_mask)
        cleaned_full[roi_cut:roi_cut + cleaned_roi.shape[0]] = cleaned_roi
        self.last_debug = {
            'bev_mask': cleaned_full,
            'inverse_transform': cv2.getPerspectiveTransform(dst_mat, src_mat),
            'roi_cut': roi_cut,
            'samples': samples,
            'confidence': confidence,
            'source': source,
        }

    def _annotate(self, vis, samples, pad, orig_w):
        """넓힌 시각화 캔버스에 복원 결과를 그린다. 좌표는 640 기준이라 +pad만큼 이동해 그린다."""
        ph = vis.shape[0]
        # 원래 카메라 창(640) 경계 표시 → 이 바깥으로 뻗는 가상 차선이 보인다
        cv2.line(vis, (pad, 0), (pad, ph), (120, 120, 120), 1)
        cv2.line(vis, (pad + orig_w, 0), (pad + orig_w, ph), (120, 120, 120), 1)
        for y, res in samples:
            if not res.valid:
                continue
            for ex in res.edges:
                cv2.circle(vis, (int(ex) + pad, int(y)), 5, (0, 255, 0), -1)   # 실제 검출 차선(초록)
            if res.virtual_x is not None:
                cv2.circle(vis, (int(res.virtual_x) + pad, int(y)), 5, (0, 165, 255), -1)  # 가상 차선(주황)
                if res.edges:
                    cv2.line(vis, (int(res.edges[0]) + pad, int(y)), (int(res.virtual_x) + pad, int(y)), (0, 165, 255), 1)
            cv2.circle(vis, (int(res.center) + pad, int(y)), 4, (0, 0, 255), -1)  # 중심(빨강)
        cv2.putText(vis, f"bev_pad={pad} virt={self.virtual_lane_width}  green=real orange=virtual red=center",
                    (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        return vis
