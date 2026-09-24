import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, LaneInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        # 파라미터 선언 및 기본값
        self.declare_parameter('camera_topic', 'image_raw')
        self.declare_parameter('detection_topic', 'detections')
        self.declare_parameter('persp_src', [0.1,1.0,0.9,1.0,0.6,0.4,0.4,0.4])
        self.declare_parameter('persp_dst', [0.3,1.0,0.7,1.0,0.7,0.0,0.3,0.0])
        self.declare_parameter('publish_debug', False)
        self.declare_parameter('publish_viz', False)
        self.declare_parameter('lost_lane_grace_frames', 5)

        cam_topic = self.get_parameter('camera_topic').value
        det_topic = self.get_parameter('detection_topic').value
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        result_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # CvBridge 및 토픽 설정
        self.bridge = CvBridge()
        img_sub = Subscriber(self, Image, cam_topic, qos_profile=sensor_qos)
        det_sub = Subscriber(self, DetectionArray, det_topic, qos_profile=sensor_qos)
        self.ts = ApproximateTimeSynchronizer([img_sub, det_sub], queue_size=3, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

        self.viz_pub = self.create_publisher(Image, 'lane_viz', sensor_qos)
        self.debug_pub = self.create_publisher(Image, 'lane_debug', sensor_qos)
        # LaneInfo 퍼블리셔
        self.lane_info_pub = self.create_publisher(LaneInfo, 'lane_info', result_qos)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.publish_viz = bool(self.get_parameter('publish_viz').value)
        self.lost_lane_grace_frames = max(
            0, int(self.get_parameter('lost_lane_grace_frames').value)
        )

        # 외삽 및 중앙선 보정 조건
        self.MIN_POINTS_FOR_CURVE = 5
        self.MIN_Y_SPAN_FOR_CURVE = 20
        self.LANE_WIDTH_PIXELS = 350
        self.EDGE_MARGIN = 1

        # 이전 차선 저장
        self.last_lane = None
        self.last_valid_info = None
        self.lost_lane_frames = 0

        self.get_logger().info(f"LaneDetector initialized: {cam_topic}, {det_topic}")

    def extrapolate_poly(self, pts, y_min, y_max, degree=2):
        if len(pts) < degree+1:
            return []
        ys = np.array([y for x,y in pts])
        xs = np.array([x for x,y in pts])
        coeff = np.polyfit(ys, xs, degree)
        poly = np.poly1d(coeff)
        y_extra = np.arange(y_min+1, y_max+1)
        return [(int(poly(y)), int(y)) for y in y_extra]

    def pick_longest_segment(self, pts):
        if not pts:
            return []
        pts_sorted = sorted(pts, key=lambda p: p[1])
        segments = []
        curr = [pts_sorted[0]]
        for p in pts_sorted[1:]:
            if p[1] - curr[-1][1] <= 1:
                curr.append(p)
            else:
                segments.append(curr)
                curr = [p]
        segments.append(curr)
        return max(segments, key=lambda s: s[-1][1] - s[0][1])

    def sync_callback(self, img_msg, det_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        h, w = frame.shape[:2]

        # 1. Masks
        mask1 = np.zeros((h,w), np.uint8)
        mask2 = np.zeros((h,w), np.uint8)
        for det in det_msg.detections:
            pts = np.array([[int(p.x), int(p.y)] for p in det.mask.data], np.int32)
            if pts.shape[0] < 3: continue
            if det.class_name == 'lane1': cv2.fillPoly(mask1, [pts], 255)
            elif det.class_name == 'lane2': cv2.fillPoly(mask2, [pts], 255)

        # 2. BEV 변환
        src = self.get_parameter('persp_src').value
        dst = self.get_parameter('persp_dst').value
        src_pts = np.float32([[src[i]*w, src[i+1]*h] for i in range(0,8,2)])
        dst_pts = np.float32([[dst[i]*w, dst[i+1]*h] for i in range(0,8,2)])
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        bw1 = cv2.warpPerspective(mask1, M, (w,h), flags=cv2.INTER_LINEAR)
        bw2 = cv2.warpPerspective(mask2, M, (w,h), flags=cv2.INTER_LINEAR)

        # 3. Morphology
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        proc1 = cv2.morphologyEx(bw1, cv2.MORPH_CLOSE, kern)
        proc1 = cv2.morphologyEx(proc1, cv2.MORPH_OPEN, kern)
        proc2 = cv2.morphologyEx(bw2, cv2.MORPH_CLOSE, kern)
        proc2 = cv2.morphologyEx(proc2, cv2.MORPH_OPEN, kern)
        dbg = cv2.merge([proc1, np.zeros_like(proc1), proc2])
        # The old implementation published three full-size debug images for
        # every input frame.  Publish only the final mask and only on request.
        if self.publish_debug:
            debug_msg = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
            debug_msg.header = img_msg.header
            self.debug_pub.publish(debug_msg)

        # 4. Valid & erosion
        ones = np.ones((h,w), np.uint8)*255
        valid = cv2.warpPerspective(ones, M, (w,h), flags=cv2.INTER_NEAREST)>0
        eroded = cv2.erode(valid.astype(np.uint8), np.ones((3,3), np.uint8), iterations=1).astype(bool)
        rows = np.where(np.any(valid, axis=1))[0]
        y_end = rows.max() if rows.size else h
        y0 = h//2

        bev = dbg.copy()
        c_colors = [('b1l', (0,0,255)),('b1r', (0,165,255)),('b2l', (255,0,255)),('b2r', (255,255,0))]
        c_cent = [(0,255,0),(0,255,255)]
        bound_c = (42,42,165)

        # outlines
        inv = (~valid).astype(np.uint8)*255
        cnts,_ = cv2.findContours(inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            if cnt[:,:,1].max()<y0: continue
            cv2.drawContours(bev, [cnt], -1, bound_c, 2)
        cv2.rectangle(bev, (0,0),(w-1,h-1), bound_c,2)

        # 5. extract raw boundaries
        raw = {}
        for key, proc in [('b1l',proc1),('b1r',proc1),('b2l',proc2),('b2r',proc2)]:
            raw[key] = [(int(xs.min()), y) if 'l' in key else (int(xs.max()),y) \
                        for y in range(y0,h) if (xs:=np.where(proc[y]>0)[0]).size]

        # 6. filter & extrapolate
        b = {k:[] for k in raw}
        for key in raw:
            filt = [(x,y) for x,y in raw[key] if eroded[y,x] and y<y_end]
            seg = self.pick_longest_segment(filt)
            if any(x<=self.EDGE_MARGIN or x>=w-self.EDGE_MARGIN for x,y in seg):
                continue
            ext=[]
            if len(seg)>=self.MIN_POINTS_FOR_CURVE and max(y for x,y in seg)-min(y for x,y in seg)>=self.MIN_Y_SPAN_FOR_CURVE:
                ext=self.extrapolate_poly(seg, y_min=max(y for x,y in seg), y_max=y_end)
            b[key]=seg+ext

        # 7. draw boundaries
        for key,color in c_colors:
            pts=b[key]
            if len(pts)>1:
                cv2.polylines(bev,[np.array(pts,np.int32)],False,color,2)

        # 8. compute lane infos
        infos=[]
        for idx,(lkey,rkey) in enumerate([('b1l','b1r'),('b2l','b2r')], start=1):
            left, right = b[lkey], b[rkey]
            raw_c=[]
            if left and right:
                lm={y:x for x,y in left}; rm={y:x for x,y in right}
                common=sorted(set(lm)&set(rm))
                raw_c=[((lm[y]+rm[y])//2,y) for y in common]
            elif left:
                raw_c=[(x+self.LANE_WIDTH_PIXELS//2,y) for x,y in left]
            elif right:
                raw_c=[(x-self.LANE_WIDTH_PIXELS//2,y) for x,y in right]
            seg = self.pick_longest_segment(raw_c)
            ext=[]
            if len(seg)>=self.MIN_POINTS_FOR_CURVE and max(y for x,y in seg)-min(y for x,y in seg)>=self.MIN_Y_SPAN_FOR_CURVE:
                ext=self.extrapolate_poly(seg, y_min=max(y for x,y in seg), y_max=y_end)
            c_full = seg+ext

            # regression slope
            n=len(c_full)
            if n>=3:
                if idx == 1:  # lane1 
                    i20 = max(0, min(n-1, n-1 - int(n * 0.0)))
                    i40 = max(0, min(n-1, n-1 - int(n * 0.4)))
                else:  # lane2 
                    i20 = max(0, min(n-1, n-1 - int(n * 0.2)))
                    i40 = max(0, min(n-1, n-1 - int(n * 0.4)))

                segpts = c_full[i40:i20+1] if i40 < i20 else c_full[i20:i40+1]
                ys = np.array([y for x, y in segpts])
                xs = np.array([x for x, y in segpts])
                a, _ = np.polyfit(ys, xs, 1)
                ang_rad = np.arctan2(-a, 1.0)
                ang_deg = int(np.degrees(ang_rad))
            else:
                ang_deg = 0
            # vehicle_x based minimal abs
            if c_full:
                bx=c_full[-1][0]; mid=w//2
                vx=mid-bx
            else:
                vx=9999  # 차선 미검출 시
            infos.append((ang_deg,idx,vx))
            # draw center line
            if len(c_full)>1:
                cv2.polylines(bev,[np.array(c_full,np.int32)],False,c_cent[idx-1],3)

        # Both lanes not detected
        if  not infos or all(vx==9999 for _,_,vx in infos):
            self.lost_lane_frames += 1
            if (self.last_valid_info is not None and
                    self.lost_lane_frames <= self.lost_lane_grace_frames):
                angle, lane_num, vx = self.last_valid_info
            else:
                # 9999 is the existing invalid sentinel used by this package.
                # The motion node converts it to a STOP command.
                lane_num = self.last_lane if self.last_lane is not None else 1
                angle = 0
                vx = 9999
            msg=LaneInfo()
            msg.steering_angle = angle
            msg.lane_num = lane_num
            msg.vehicle_position_x = vx
            self.lane_info_pub.publish(msg)
            self.last_lane = lane_num
            if self.lost_lane_frames in (1, self.lost_lane_grace_frames + 1):
                self.get_logger().warning(
                    f'No lane mask: miss={self.lost_lane_frames}, '
                    f'grace={self.lost_lane_grace_frames}, output_x={vx}'
                )
            return

        # select minimal abs vehicle_x
        best=min(infos, key=lambda x:abs(x[2]))
        angle, lane_num, vx = best
        # publish
        msg=LaneInfo()
        msg.steering_angle=angle; msg.lane_num=lane_num; msg.vehicle_position_x=vx
        self.lane_info_pub.publish(msg)
        self.last_lane = lane_num  # update last known lane
        self.last_valid_info = (angle, lane_num, vx)
        self.lost_lane_frames = 0

        # 9. visualization publish
        if self.publish_viz:
            viz_msg = self.bridge.cv2_to_imgmsg(bev, 'bgr8')
            viz_msg.header = img_msg.header
            self.viz_pub.publish(viz_msg)


def main(args=None):
    rclpy.init()
    node=LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()
