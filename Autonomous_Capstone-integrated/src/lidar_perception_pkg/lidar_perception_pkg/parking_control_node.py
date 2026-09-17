import time
import math

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
#  인코더가 없으므로 거리는 '속도 x 시간'으로 제어. 2번(오른쪽 출발)으로 실차 튜닝.
T_FWD_LEFT       = 9.0 # (S1)  -7 조향 전진
T_REV_STRAIGHT_A = 2.0   # (S3)   0 조향 후진
T_REV_RIGHT      = 3.0   # (S4)  +7 조향 후진
T_PAUSE          = 5.0   # (S6)  측면 장애물 감지 시 정지 시간 (요청: 5초)
T_FWD_STRAIGHT_C = 1.5   # (S7)   0 조향 전진
T_FWD_RIGHT      = 12.0   # (S8)  +7 조향 전진 (주차칸과 수직 복귀)
T_FINISH         = 30.0   # (S9)   0 조향 전진 후 도착선 통과
T_SETTLE         = 0.5   # 전진<->후진 전환 시 모터 보호용 짧은 정지

# ====================================================================
#  [S0] 최초 우측 수직거리 기반 출발위치 판별
# ====================================================================
#  우측 직각 방향(차 옆)을 보는 좁은 섹터. 한 점이 아니라 섹터 '최솟값'을 대표값으로 사용.
#  ※ 라이다 장착/processor 보정 후 '차 오른쪽 정직각'이 몇 도인지 실측해서 맞추세요.
RIGHT_PERP_SECTOR = (250, 280)

#  우측 거리가 이 값 이하로 들어왔을 때만 출발위치를 판별하고 다음 상태로 전이.
R_START_DETECT_MAX = 2.0 # [m]

#  조건 만족 시 측정한 우측 거리로 출발위치 판별:
#    측정값 <= R_DECIDE  -> 2번(오른쪽 출발). 곧장 S1 진입
#    측정값 >  R_DECIDE  -> 1번(왼쪽 출발).  슬라롬으로 우측 30cm 평행이동 후 S1 진입
R_DECIDE  = 1.4 # [실측 후 조정] 1/2번 판별 경계 [m]
#  참고(코드엔 직접 안 쓰임): 2번 기준≈0.50m, 1번 기준≈0.80m 예상.
#  -> 2번 실주행으로 'S1 진입 순간 우측 거리'를 측정한 뒤, 그 값 기준으로
#     R_DECIDE 를 채워 넣으세요.

# -------------------- (1번 전용) 우측 30cm 평행이동 슬라롬 [초] --------------------
#  +7 전진(우측으로 흘림) -> -7 전진(기수 복귀) -> 0 후진(종방향 되감기)
#  T_SLALOM_R 와 T_SLALOM_L 은 대칭에 가깝게 둬야 이동 후 heading 이 0으로 복귀함.
#  전부 0 이면 슬라롬 없이 바로 S1 (즉 기능 끄기). 실차로 튜닝.
T_SLALOM_R    = 3.0
T_SLALOM_L    = 3.0
T_SLALOM_BACK = 5.5

# -------------------- [S5] 후진 중 측면 장애물 감지 섹터/거리 --------------------
RIGHT_SECTOR = (270,290)     # 우측 (넓게)
LEFT_SECTOR  = (70, 90)    # 좌측 (넓게)
D_BACK_MIN   = 0.05
D_BACK_MAX   = 1.5          # 후진 중 좌/우 장애물 감지 거리 (요청: 약 30~50cm)

CONSEC_COUNT = 3             # 연속 N회 감지되어야 인정 (노이즈 방지)

# 제어 루프 주기 [sec] = 20Hz
CONTROL_PERIOD = 0.05

# -------------------- 안전 타임아웃 [초] --------------------
TIMEOUT_FWD_STRAIGHT = 60.0
TIMEOUT_REV_STRAIGHT = 30.0
# ========================================================================


# ----------------------------- 상태 정의 -----------------------------
ST_FWD_STRAIGHT   = 0    # 전진하며 우측 수직거리 측정, 3m 이하 -> 1/2번 판별
ST_SLALOM_R       = 1    # (1번) +7 전진
ST_SLALOM_L       = 2    # (1번) -7 전진
ST_SLALOM_BACK    = 3    # (1번)  0 후진
ST_FWD_LEFT       = 4    # (S1)  -7 전진  <-- 본 시퀀스 시작
ST_SETTLE_1       = 5    #        정지 (전진->후진)
ST_REV_STRAIGHT_A = 6    # (S3)   0 후진
ST_REV_RIGHT      = 7    # (S4)  +7 후진
ST_REV_STRAIGHT_B = 8    # (S5)   0 후진, 좌/우 장애물 감지 대기
ST_PAUSE          = 9    # (S6)  5초 정지
ST_FWD_STRAIGHT_C = 10   # (S7)   0 전진
ST_FWD_RIGHT      = 11   # (S8)  +7 전진 (수직 복귀)
ST_FINISH         = 12   # (S9)   0 전진 후 도착선 통과
ST_DONE           = 13   # 정차 완료

STATE_NAME = {
    ST_FWD_STRAIGHT:   'FWD_STRAIGHT (전진/우측거리 대기/판별)',
    ST_SLALOM_R:       'SLALOM_R (+7 전진, 우측이동)',
    ST_SLALOM_L:       'SLALOM_L (-7 전진, 기수복귀)',
    ST_SLALOM_BACK:    'SLALOM_BACK (0 후진, 종방향복귀)',
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


class ParkingControlNode(Node):
    def __init__(self):
        super().__init__('parking_control_node')

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

        self.back_detector    = LPFL.StabilityDetector(consec_count=CONSEC_COUNT)

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
        self.back_detector    = LPFL.StabilityDetector(consec_count=CONSEC_COUNT)
        self.get_logger().info(f'[STATE] -> {STATE_NAME.get(new_state, new_state)}')

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

        # ---------- S0: 전진하며 우측 수직거리 대기 후 출발위치 판별 ----------
        if state == ST_FWD_STRAIGHT:
            self.drive(STEER_STRAIGHT, FORWARD_SPEED)
            d = self.sector_min(RIGHT_PERP_SECTOR)
            if d is not None and d <= R_START_DETECT_MAX:
                self.get_logger().info(f'우측거리 감지 = {d:.2f}m (기준 {R_START_DETECT_MAX:.2f}m 이하)')
                if d <= R_DECIDE:
                    self.get_logger().info('-> 2번(오른쪽 출발). 바로 S1 진입')
                    self.enter_state(ST_FWD_LEFT)
                else:
                    self.get_logger().info('-> 1번(왼쪽 출발). 우측 30cm 평행이동(슬라롬) 후 S1')
                    self.enter_state(ST_SLALOM_R)
            elif elapsed > TIMEOUT_FWD_STRAIGHT:
                self.get_logger().warn('타임아웃: 유효 우측거리 미측정. 안전 정지.')
                self.enter_state(ST_DONE)

        # ---------- (1번) 우측 평행이동 슬라롬 ----------
        elif state == ST_SLALOM_R:
            self.drive(STEER_RIGHT_MAX, FORWARD_SPEED)
            if elapsed >= T_SLALOM_R:
                self.enter_state(ST_SLALOM_L)

        elif state == ST_SLALOM_L:
            self.drive(STEER_LEFT_MAX, FORWARD_SPEED)
            if elapsed >= T_SLALOM_L:
                self.enter_state(ST_SLALOM_BACK)

        elif state == ST_SLALOM_BACK:
            self.drive(STEER_STRAIGHT, REVERSE_SPEED)
            if elapsed >= T_SLALOM_BACK:
                self.enter_state(ST_FWD_LEFT)

        # ======================= 본 시퀀스 (1/2번 공통) =======================
        # ---------- S1: -7 전진 ----------
        elif state == ST_FWD_LEFT:
            self.drive(STEER_LEFT_MAX, FORWARD_SPEED)
            if elapsed >= T_FWD_LEFT:
                self.enter_state(ST_SETTLE_1)

        # ---------- S2: 정지 (전진->후진) ----------
        elif state == ST_SETTLE_1:
            self.drive(STEER_STRAIGHT, STOP_SPEED)
            if elapsed >= T_SETTLE:
                self.enter_state(ST_REV_STRAIGHT_A)

        # ---------- S3: 0 후진 ----------
        elif state == ST_REV_STRAIGHT_A:
            self.drive(STEER_STRAIGHT, REVERSE_SPEED)
            if elapsed >= T_REV_STRAIGHT_A:
                self.enter_state(ST_REV_RIGHT)

        # ---------- S4: +7 후진 (주차칸 평행 정렬) ----------
        elif state == ST_REV_RIGHT:
            self.drive(STEER_RIGHT_MAX, REVERSE_SPEED)
            if elapsed >= T_REV_RIGHT:
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
            if elapsed >= T_FWD_STRAIGHT_C:
                self.enter_state(ST_FWD_RIGHT)

        # ---------- S8: +7 전진 (수직 복귀) ----------
        elif state == ST_FWD_RIGHT:
            self.drive(STEER_RIGHT_MAX, FORWARD_SPEED)
            if elapsed >= T_FWD_RIGHT:
                self.enter_state(ST_FINISH)

        # ---------- S9: 0 전진 후 도착선 통과 ----------
        elif state == ST_FINISH:
            self.drive(STEER_STRAIGHT, FORWARD_SPEED)
            if elapsed >= T_FINISH:
                self.enter_state(ST_DONE)

        # ---------- 정차 완료 ----------
        elif state == ST_DONE:
            self.drive(STEER_STRAIGHT, STOP_SPEED)
            if not self._done_logged:
                self.get_logger().info('주차 미션 완료. 정차합니다.')
                self._done_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = ParkingControlNode()
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
