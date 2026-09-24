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
                 capture_debug: bool = False):
        self.show_image = show_image
        self.capture_debug = capture_debug
        self.last_debug = None
        self.bev_top_shift = bev_top_shift
        self.roi_cut = roi_cut
        self.look_shift = look_shift
        # 프레임 간 기울기 떨림을 줄이는 지수이동평균(EMA) 계수. 0=평활화 안 함, 1=매 프레임 즉시 반영.
        self.slope_ema_alpha = slope_ema_alpha
        self.smoothed_slope = None
        # 코너링 시 잘린 차선을 복원할 때 쓰는 가상 차선 폭(px). 실시간 조절 가능.
        self.virtual_lane_width = virtual_lane_width
        # BEV 캔버스를 좌우로 넓히는 패딩(px, 한쪽). 코너에서 창 밖으로 나가 잘리던 차선을
        # 되살리고, 가상 차선이 보일 공간을 만든다. 좌표가 +bev_pad 이동하므로 차량 중심도
        # 같이 이동해야 한다(드라이버가 motion.car_center_x = 원래중심+bev_pad 로 설정).
        self.bev_pad = bev_pad

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
            return LaneInfo()

        lane2_edge_image = CPFL.draw_edges(detections, cls_name="lane2", color=255)
        h, w = lane2_edge_image.shape[:2]

        # 계산은 원래 640 캔버스에서 수행한다. 그래야 경계(x=0, x=w-1) 접촉 판정이
        # 원래대로 동작해 잘린 차선의 가상 복원이 정확히 작동한다.
        # (bev_pad는 시각화 캔버스를 좌우로 넓히는 데에만 쓴다.)
        dst_mat = np.float32([[round(w * 0.3), 0], [round(w * 0.7), 0],
                              [round(w * 0.7), h], [round(w * 0.3), h]])
        src_mat = np.float32(self._src_mat())
        M = cv2.getPerspectiveTransform(src_mat, dst_mat)
        lane2_bird_image = cv2.warpPerspective(lane2_edge_image, M, (w, h))

        roi_cut = int(max(0, min(h - 10, self.roi_cut)))
        roi_image = cv2.convertScaleAbs(lane2_bird_image[roi_cut:])

        grad = CPFL.dominant_gradient(roi_image, theta_limit=70)

        # 시간적 평활화(EMA): 허프 추정 기울기의 프레임 간 떨림을 완화
        if self.smoothed_slope is None:
            self.smoothed_slope = grad
        else:
            a = self.slope_ema_alpha
            self.smoothed_slope = a * grad + (1.0 - a) * self.smoothed_slope
        grad = self.smoothed_slope

        # BEV 원근 사다리꼴의 상/하단 폭. 윗변(먼 곳)은 좁고 아랫변(가까운 곳)은 넓다.
        # 직사각형으로 잡아당기면 위로 갈수록 더 크게 확대되므로, 가상 차선 폭도 그만큼 늘린다.
        src = self._src_mat()
        top_w = abs(src[1][0] - src[0][0])      # 사다리꼴 윗변 폭(좁음)
        bottom_w = abs(src[2][0] - src[3][0])   # 사다리꼴 아랫변 폭(넓음) — 기준
        if top_w <= 0:
            top_w = bottom_w

        roi_h = roi_image.shape[0]
        target_points = []
        samples = []  # (y, LaneCenter) 시각화용
        for base_y in range(5, 155, 50):
            target_point_y = base_y + self.look_shift
            # 샘플링 높이가 ROI 밖으로 나가지 않도록 클램프
            target_point_y = int(max(0, min(roi_h - 1, target_point_y)))
            # 이 ROI 높이에 대응하는 BEV row의 원근 확대율로 가상 차선 폭을 보정.
            # 아랫변(가까운 곳)을 기준(virtual_lane_width)으로 통일하고, 위로 갈수록 더 늘림.
            bev_row = roi_cut + target_point_y
            frac = min(1.0, max(0.0, bev_row / float(h)))
            src_w = top_w + (bottom_w - top_w) * frac
            vw = int(round(self.virtual_lane_width * (bottom_w / max(1.0, src_w))))
            res = CPFL.get_lane_center(
                roi_image,
                detection_height=target_point_y,
                detection_thickness=10,
                road_gradient=grad,
                lane_width=300,
                virtual_lane_width=vw,
            )
            target_points.append(TargetPoint(target_x=round(res.center), target_y=round(target_point_y)))
            samples.append((target_point_y, res))

        if self.capture_debug:
            self.last_debug = {
                'bev_mask': lane2_bird_image,
                'inverse_transform': cv2.getPerspectiveTransform(dst_mat, src_mat),
                'roi_cut': roi_cut,
                'samples': samples,
            }

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

        return LaneInfo(slope=grad, target_points=target_points)

    def _annotate(self, vis, samples, pad, orig_w):
        """넓힌 시각화 캔버스에 복원 결과를 그린다. 좌표는 640 기준이라 +pad만큼 이동해 그린다."""
        ph = vis.shape[0]
        # 원래 카메라 창(640) 경계 표시 → 이 바깥으로 뻗는 가상 차선이 보인다
        cv2.line(vis, (pad, 0), (pad, ph), (120, 120, 120), 1)
        cv2.line(vis, (pad + orig_w, 0), (pad + orig_w, ph), (120, 120, 120), 1)
        for y, res in samples:
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
