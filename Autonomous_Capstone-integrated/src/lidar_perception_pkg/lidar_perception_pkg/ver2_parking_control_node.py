import time
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import LaserScan
from interfaces_pkg.msg import MotionCommand

# 기존 lidar_obstacle_detector_node 와 동일한 라이브러리 사용 -> 각도 규칙 100% 일치
from .lib import lidar_perception_func_lib as LPFL


# =========================== Variable Setting ===========================
# 구독/발행 토픽 이름
SUB_LIDAR_TOPIC_NAME   = 'lidar_processed'        # lidar_processor_node 출력 (없으면 'lidar_raw'로 변경)
PUB_CONTROL_TOPIC_NAME = 'topic_control_signal'   # serial_sender_node 입력

# -------------------- 속도 / 조향 값 --------------------
FORWARD_SPEED = 85      # 전진 속도
REVERSE_SPEED = -60     # 후진 속도
#  ※ 아두이노 펌웨어가 '음수 속도 = 후진'으로 해석한다는 가정. 다르면 맞게 수정.
STOP_SPEED = 0

STEER_STRAIGHT  = 0
STEER_LEFT_MAX  = -7     # 제일 왼쪽
STEER_RIGHT_MAX = 7      # 제일 오른쪽

# -------------------- 본 시퀀스 구간 시간 [초] --------------------
#  ver2: 우측 평행이동 슬라롬 없이 바로 S1(-7 전진)으로 진입.
#  최초 우측 거리 4개 구간별로 아래 파라미터 묶음을 각각 실차에서 따로 튜닝하세요.
@dataclass(frozen=True)
class ParkingTiming:
    fwd_left: float
    rev_straight_a: float
    rev_right: float
    fwd_straight_c: float
    fwd_right: float
    finish: float


# 1구간: 우측 거리 1.0m 이하
TIMING_LE_1_0M = ParkingTiming(
    fwd_left=10.0,        # (S1) -7 조향 전진
    rev_straight_a=2.5,  # (S3)  0 조향 후진
    rev_right=2.0,       # (S4) +7 조향 후진
    fwd_straight_c=1.5,  # (S7)  0 조향 전진
    fwd_right=12.0,      # (S8) +7 조향 전진
    finish=30.0,         # (S9)  0 조향 전진 후 도착선 통과
)

# 2구간: 우측 거리 1.0m 초과 ~ 1.5m 이하
TIMING_1_0_TO_1_5M = ParkingTiming(
    fwd_left=9.0,        # (S1) -7 조향 전진
    rev_straight_a=2.0,  # (S3)  0 조향 후진
    rev_right=3.0,       # (S4) +7 조향 후진
    fwd_straight_c=1.5,  # (S7)  0 조향 전진
    fwd_right=12.0,      # (S8) +7 조향 전진
    finish=30.0,         # (S9)  0 조향 전진 후 도착선 통과
)

# 3구간: 우측 거리 1.5m 초과 ~ 2.0m 이하
TIMING_1_5_TO_2_0M = ParkingTiming(
    fwd_left=9.0,        # (S1) -7 조향 전진
    rev_straight_a=2.0,  # (S3)  0 조향 후진
    rev_right=3.0,       # (S4) +7 조향 후진
    fwd_straight_c=1.5,  # (S7)  0 조향 전진
    fwd_right=12.0,      # (S8) +7 조향 전진
    finish=30.0,         # (S9)  0 조향 전진 후 도착선 통과
)

# 4구간: 우측 거리 2.0m 초과 ~ 2.5m 이하
TIMING_2_0_TO_2_5M = ParkingTiming(
    fwd_left=8.0,        # (S1) -7 조향 전진
    rev_straight_a=2.0,  # (S3)  0 조향 후진
    rev_right=2.5,       # (S4) +7 조향 후진
    fwd_straight_c=1.5,  # (S7)  0 조향 전진
    fwd_right=12.0,      # (S8) +7 조향 전진
    finish=30.0,         # (S9)  0 조향 전진 후 도착선 통과
)

T_PAUSE  = 5.0   # (S6) 측면 장애물 감지 시 정지 시간
T_SETTLE = 0.5   # 전진<->후진 전환 시 모터 보호용 짧은 정지

# ====================================================================
#  [S0] 최초 우측 수직거리 기반 출발위치 판별
# ====================================================================
#  우측 직각 방향(차 옆)을 보는 좁은 섹터. 한 점이 아니라 섹터 '최솟값'을 대표값으로 사용.
#  ※ 라이다 장착/processor 보정 후 '차 오른쪽 정직각'이 몇 도인지 실측해서 맞추세요.
RIGHT_PERP_SECTOR = (250, 270)

#  우측 거리가 이 값 이하로 들어왔을 때만 출발위치를 판별하고 다음 상태로 전이.
R_START_DETECT_MAX = 2.0 # [m]

#  조건 만족 시 측정한 우측 거리로 구간 판별:
#    d <= 1.0       -> 1구간
#    1.0 < d <= 1.5 -> 2구간
#    1.5 < d <= 2.0 -> 3구간
#    2.0 < d <= 2.5 -> 4구간

# -------------------- [S5] 후진 중 측면 장애물 감지 섹터/거리 --------------------
RIGHT_SECTOR = (270, 290)     # 우측 (넓게)
LEFT_SECTOR  = (70, 90)       # 좌측 (넓게)
D_BACK_MIN   = 0.05
D_BACK_MAX   = 1.5            # 후진 중 좌/우 장애물 감지 거리

CONSEC_COUNT = 3              # 연속 N회 감지되어야 인정 (노이즈 방지)

# 제어 루프 주기 [sec] = 20Hz
CONTROL_PERIOD = 0.05

# -------------------- 안전 타임아웃 [초] --------------------
TIMEOUT_FWD_STRAIGHT = 60.0
TIMEOUT_REV_STRAIGHT = 30.0
# ========================================================================


# ----------------------------- 상태 정의 -----------------------------
ST_FWD_STRAIGHT   = 0    # 전진하며 우측 수직거리 측정, 2.5m 이하 -> 4구간 판별
ST_FWD_LEFT       = 1    # (S1) -7 전진  <-- 본 시퀀스 시작
ST_SETTLE_1       = 2    #      정지 (전진->후진)
ST_REV_STRAIGHT_A = 3    # (S3)  0 후진
ST_REV_RIGHT      = 4    # (S4) +7 후진
ST_REV_STRAIGHT_B = 5    # (S5)  0 후진, 좌/우 장애물 감지 대기
ST_PAUSE          = 6    # (S6)  5초 정지
ST_FWD_STRAIGHT_C = 7    # (S7)  0 전진
ST_FWD_RIGHT      = 8    # (S8) +7 전진 (수직 복귀)
ST_FINISH         = 9    # (S9)  0 전진 후 도착선 통과
ST_DONE           = 10   # 정차 완료

STATE_NAME = {
    ST_FWD_STRAIGHT:   'FWD_STRAIGHT (전진/우측거리 대기/판별)',
    ST_FWD_LEFT:       'S1 FWD_LEFT (-7 전진)',
    ST_SETTLE_1:       'SETTLE_1 (정지)',
    ST_REV_STRAIGHT_A: 'S3 REV_STRAIGHT_A (0 후진)',
    ST_REV_RIGHT:      'S4 REV_RIGHT (+7 후진)',
    ST_REV_STRAIGHT_B: 'S5 REV_STRAIGHT_B (0 후진/측면감지)',
    ST_PAUSE:          'S6 PAUSE (5초 정지)',
    ST_FWD_STRAIGHT_C: 'S7 FWD_STRAIGHT_C (0 전진)',
    ST_FWD_RIGHT:      'S8 FWD_RIGHT (+7 전진)',
    ST_FINISH:         'S9 FINISH (도착선 통과)',
    ST_DONE:           'DONE (정차 완료)',
}


class Ver2ParkingControlNode(Node):
    def __init__(self):
        super().__init__('ver2_parking_control_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.scan = None
        self.subscriber = self.create_subscription(
            LaserScan, SUB_LIDAR_TOPIC_NAME, self.scan_callback, self.qos_profile)
        self.publisher = self.create_publisher(
            MotionCommand, PUB_CONTROL_TOPIC_NAME, self.qos_profile)

        self.back_detector = LPFL.StabilityDetector(consec_count=CONSEC_COUNT)
        self.distance_case = None
        self.timing = TIMING_LE_1_0M

        self._done_logged = False

        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control_loop)   # 20Hz
        self.enter_state(ST_FWD_STRAIGHT)   # 실행 즉시 전진부터 시작

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    # ------------------------------------------------------------------
    def enter_state(self, new_state):
        self.state = new_state
        self.state_start = time.monotonic()
        self.back_detector = LPFL.StabilityDetector(consec_count=CONSEC_COUNT)
        self.get_logger().info(f'[STATE] -> {STATE_NAME.get(new_state, new_state)}')

    # ------------------------------------------------------------------
    def select_distance_case(self, distance):
        if distance <= 1.0:
            self.distance_case = '1구간(d <= 1.0m)'
            self.timing = TIMING_LE_1_0M
        elif distance <= 1.5:
            self.distance_case = '2구간(1.0m < d <= 1.5m)'
            self.timing = TIMING_1_0_TO_1_5M
        elif distance <= 1.75:
            self.distance_case = '3구간(1.5m < d <= 1.75m)'
            self.timing = TIMING_1_5_TO_2_0M
        else:
            self.distance_case = '4구간(2.0m < d <= 2.5m)'
            self.timing = TIMING_2_0_TO_2_5M

        self.get_logger().info(
            f'-> {self.distance_case}. 슬라롬 없이 S1 진입. timing={self.timing}'
        )

    # ------------------------------------------------------------------
    def drive(self, steering, speed):
        cmd = MotionCommand()
        cmd.steering = int(steering)
        cmd.left_speed = int(speed)
        cmd.right_speed = int(speed)
        self.publisher.publish(cmd)

    # ------------------------------------------------------------------
    def sector_min(self, sector):
        """섹터 내 유효 거리들의 최솟값(가장 가까운 벽). 없으면 None.
        index = 각도(도) 규칙 (detect_object 와 동일)."""
        if self.scan is None or len(self.scan.ranges) == 0:
            return None
        ranges = self.scan.ranges
        n = len(ranges)
        s, e = sector[0] % 360, sector[1] % 360
        idxs = range(s, e + 1) if s <= e else list(range(s, 360)) + list(range(0, e + 1))
        vals = []
        for i in idxs:
            r = ranges[i % n]
            if r is None or r <= 0.0 or math.isinf(r) or math.isnan(r):
                continue
            vals.append(r)
        return min(vals) if vals else None

    # ------------------------------------------------------------------
    def detect_side(self, sector, dmin, dmax):
        if self.scan is None or len(self.scan.ranges) == 0:
            return False
        return LPFL.detect_object(ranges=self.scan.ranges,
                                  start_angle=sector[0], end_angle=sector[1],
                                  range_min=dmin, range_max=dmax)

    # ------------------------------------------------------------------
    def control_loop(self):
        elapsed = time.monotonic() - self.state_start
        state = self.state
        timing = self.timing

        # ---------- S0: 전진하며 우측 수직거리 대기 후 출발위치 판별 ----------
        if state == ST_FWD_STRAIGHT:
            self.drive(STEER_STRAIGHT, FORWARD_SPEED)
            d = self.sector_min(RIGHT_PERP_SECTOR)
            if d is not None and d <= R_START_DETECT_MAX:
                self.get_logger().info(f'우측거리 감지 = {d:.2f}m (기준 {R_START_DETECT_MAX:.2f}m 이하)')
                self.select_distance_case(d)
                self.enter_state(ST_FWD_LEFT)
            elif elapsed > TIMEOUT_FWD_STRAIGHT:
                self.get_logger().warn('타임아웃: 유효 우측거리 미측정. 안전 정지.')
                self.enter_state(ST_DONE)

        # ======================= 본 시퀀스 (거리 구간별 시간만 다름) =======================
        # ---------- S1: -7 전진 ----------
        elif state == ST_FWD_LEFT:
            self.drive(STEER_LEFT_MAX, FORWARD_SPEED)
            if elapsed >= timing.fwd_left:
                self.enter_state(ST_SETTLE_1)

        # ---------- S2: 정지 (전진->후진) ----------
        elif state == ST_SETTLE_1:
            self.drive(STEER_STRAIGHT, STOP_SPEED)
            if elapsed >= T_SETTLE:
                self.enter_state(ST_REV_STRAIGHT_A)

        # ---------- S3: 0 후진 ----------
        elif state == ST_REV_STRAIGHT_A:
            self.drive(STEER_STRAIGHT, REVERSE_SPEED)
            if elapsed >= timing.rev_straight_a:
                self.enter_state(ST_REV_RIGHT)

        # ---------- S4: +7 후진 (주차칸 평행 정렬) ----------
        elif state == ST_REV_RIGHT:
            self.drive(STEER_RIGHT_MAX, REVERSE_SPEED)
            if elapsed >= timing.rev_right:
                self.enter_state(ST_REV_STRAIGHT_B)

        # ---------- S5: 0 후진, 좌/우 장애물 감지 대기 ----------
        elif state == ST_REV_STRAIGHT_B:
            self.drive(STEER_STRAIGHT, REVERSE_SPEED)
            hit_l = self.detect_side(LEFT_SECTOR,  D_BACK_MIN, D_BACK_MAX)
            hit_r = self.detect_side(RIGHT_SECTOR, D_BACK_MIN, D_BACK_MAX)
            if self.back_detector.check_consecutive_detections(hit_l or hit_r):
                self.get_logger().info('측면 장애물 감지! -> 5초 정지')
                self.enter_state(ST_PAUSE)
            elif elapsed > TIMEOUT_REV_STRAIGHT:
                self.get_logger().warn('타임아웃: 측면 장애물 미감지. 안전 정지.')
                self.enter_state(ST_DONE)

        # ---------- S6: 5초 정지 ----------
        elif state == ST_PAUSE:
            self.drive(STEER_STRAIGHT, STOP_SPEED)
            if elapsed >= T_PAUSE:
                self.enter_state(ST_FWD_STRAIGHT_C)

        # ---------- S7: 0 전진 ----------
        elif state == ST_FWD_STRAIGHT_C:
            self.drive(STEER_STRAIGHT, FORWARD_SPEED)
            if elapsed >= timing.fwd_straight_c:
                self.enter_state(ST_FWD_RIGHT)

        # ---------- S8: +7 전진 (수직 복귀) ----------
        elif state == ST_FWD_RIGHT:
            self.drive(STEER_RIGHT_MAX, FORWARD_SPEED)
            if elapsed >= timing.fwd_right:
                self.enter_state(ST_FINISH)

        # ---------- S9: 0 전진 후 도착선 통과 ----------
        elif state == ST_FINISH:
            self.drive(STEER_STRAIGHT, FORWARD_SPEED)
            if elapsed >= timing.finish:
                self.enter_state(ST_DONE)

        # ---------- 정차 완료 ----------
        elif state == ST_DONE:
            self.drive(STEER_STRAIGHT, STOP_SPEED)
            if not self._done_logged:
                self.get_logger().info('주차 미션 완료. 정차합니다.')
                self._done_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = Ver2ParkingControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        try:
            node.drive(STEER_STRAIGHT, STOP_SPEED)   # 종료 시 안전 정지
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
