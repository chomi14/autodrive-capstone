"""Standalone horizontal camera/LiDAR association; no ROS dependency."""
import json
import math
from pathlib import Path

import numpy as np

# 사용자 조정값: 거리 m, 각도 degree, 픽셀은 640x480 영상 기준.
SETTINGS = {
    'yaw_deg': 0.0,              # driver 회전 적용 후 남은 LiDAR 방향 보정
    'angle_sign': 1,             # scan 좌우 반전: +1 또는 -1
    'camera_fx_px': 554.0,       # 수평 초점거리: 반드시 실제 카메라로 보정
    'camera_cx_px': 320.0,       # 카메라 영상 중심 x
    'camera_yaw_deg': 0.0,       # 차량 전방 대비 카메라 광축 방향(+좌측)
    'lidar_x_m': 0.0,            # 카메라 원점 대비 LiDAR 전방 위치
    'lidar_y_m': 0.0,            # 카메라 원점 대비 LiDAR 좌측 위치
    'range_min_m': 0.15,         # 너무 가까운/차체 반사 제외
    'range_max_m': 6.0,          # 화면 및 매칭 최대 거리
    'near_m': 1.5,               # 이 거리 이내 경로 장애물은 기존 회피 수행
    'stop_m': 0.45,              # 이 거리 이내 경로 장애물은 즉시 정지
    'min_points': 3,             # bbox에 연결되어야 하는 최소 scan 점 수
    'margin_px': 8.0,            # bbox 수평 매칭 여유
    'max_cluster_gap_m': 0.20,   # 서로 다른 거리의 반사점 분리 기준
    'scan_timeout_s': 0.40,      # scan 미수신 정지 기준
    'image_timeout_s': 0.75,     # 카메라 미수신 정지 기준(새 모드 전용)
    'max_pair_delta_s': 0.25,    # 영상과 scan 취득 timestamp 최대 차이
    'confirmed': 0,              # 실제 물체로 정렬 확인 후 GUI에서 1로 설정
}
CALIBRATION_PATH = str(Path.home() / '.config/autodrive/lidar_camera_calibration.json')
SCAN_TOPIC = '/fusion/scan'
CALIBRATION_TOPIC = '/fusion/calibration'
WINDOW_SIZE = 800                  # LiDAR top-view 화면 크기
GUI_PERIOD_S = 0.05                # GUI 갱신 간격
WATCHDOG_PERIOD_S = 0.05           # scan/image 정지 검사 간격


def validate(values):
    p = {**SETTINGS, **values}
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in p.values()):
        raise ValueError('Calibration must contain finite numeric values')
    if p['angle_sign'] not in (-1, 1) or p['confirmed'] not in (0, 1):
        raise ValueError('angle_sign must be +/-1; confirmed must be 0/1')
    if not 0 < p['range_min_m'] < p['stop_m'] <= p['near_m'] <= p['range_max_m']:
        raise ValueError('Required: 0 < range_min < stop <= near <= range_max')
    if p['camera_fx_px'] <= 0 or p['min_points'] < 1 or int(p['min_points']) != p['min_points']:
        raise ValueError('Positive fx and integer min_points required')
    if min(p[k] for k in ('scan_timeout_s', 'image_timeout_s', 'max_pair_delta_s', 'max_cluster_gap_m')) <= 0 or p['margin_px'] < 0:
        raise ValueError('Timeouts/cluster gap must be positive; margin nonnegative')
    return p


def load_settings(path=CALIBRATION_PATH):
    file = Path(path)
    return validate(json.loads(file.read_text(encoding='utf-8')) if file.exists() else {})


def save_settings(values, path=CALIBRATION_PATH):
    file = Path(path)
    file.parent.mkdir(parents=True, exist_ok=True)
    temporary = file.with_suffix('.tmp')
    temporary.write_text(json.dumps(validate(values), indent=2), encoding='utf-8')
    temporary.replace(file)


def points(scan, p):
    r = np.asarray(scan.ranges, dtype=float)
    a = (scan.angle_min + np.arange(len(r)) * scan.angle_increment) * p['angle_sign'] + math.radians(p['yaw_deg'])
    ok = np.isfinite(r) & (r >= max(scan.range_min, p['range_min_m'])) & (r <= min(scan.range_max, p['range_max_m']))
    return np.column_stack((r[ok] * np.cos(a[ok]) + p['lidar_x_m'], r[ok] * np.sin(a[ok]) + p['lidar_y_m']))


def associate(detection, xy, p, width=640):
    """Horizontal bearing only: not a full calibrated 3D projection.

    Multiple depth clusters are ambiguous; do not guess which object is in bbox.
    A 2D scan cannot distinguish vertically stacked objects.
    """
    yaw = math.radians(p['camera_yaw_deg'])
    forward = xy[:, 0] * math.cos(yaw) + xy[:, 1] * math.sin(yaw)
    left = -xy[:, 0] * math.sin(yaw) + xy[:, 1] * math.cos(yaw)
    ok = forward > 0.01
    u = (p['camera_cx_px'] - p['camera_fx_px'] * left[ok] / forward[ok]) * width / 640.0
    box = detection.bbox
    margin = p['margin_px'] * width / 640.0
    selected = xy[ok][(u >= box.center.position.x - box.size.x/2 - margin) &
                       (u <= box.center.position.x + box.size.x/2 + margin)]
    distances = np.sort(np.linalg.norm(selected, axis=1))
    groups = np.split(distances, np.where(np.diff(distances) > p['max_cluster_gap_m'])[0] + 1)
    groups = [g for g in groups if len(g) >= p['min_points']]
    if len(groups) != 1:
        return {'distance_m': None, 'points': len(selected), 'reason': 'AMBIGUOUS_DEPTH' if groups else 'NO_MATCH'}
    return {'distance_m': float(np.quantile(groups[0], 0.20)), 'points': len(groups[0]), 'reason': 'MATCHED'}
