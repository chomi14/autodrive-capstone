#!/usr/bin/env python3
"""Compare BEV/ROI extraction offline, using synthetic and saved camera masks.

Imports controller rendering only; never constructs a ROS node, loads a model,
opens a device, or publishes a command. Steering values are in-memory results.
"""
import argparse
import json
from pathlib import Path
import subprocess
from types import ModuleType, SimpleNamespace

import cv2
import numpy as np
import yaml

from skku_track_drive_pkg.lane_processing import LaneInfoExtractor
from skku_track_drive_pkg.messages import Detection, DetectionArray, Mask
from skku_track_drive_pkg.motion_planner import MotionPlanner
from skku_track_drive_pkg.path_planner import PathPlanner
from skku_track_drive_pkg.track_controller_node import TrackControllerNode


ROOT = Path(__file__).resolve().parents[1]
LANE_MODULE = 'src/skku_track_drive_pkg/skku_track_drive_pkg/lane_processing.py'


def baseline_class(revision):
    source = subprocess.check_output(['git', 'show', f'{revision}:./{LANE_MODULE}'], cwd=ROOT, text=True)
    module = ModuleType('skku_track_drive_pkg._baseline_lane_processing')
    module.__package__ = 'skku_track_drive_pkg'
    exec(compile(source, f'{revision}:{LANE_MODULE}', 'exec'), module.__dict__)
    return module.LaneInfoExtractor


def extractor(cls, tuning):
    return cls(show_image=False, capture_debug=True,
               bev_source_points=[[tuning[f'bev_src_{c}_x'], tuning[f'bev_src_{c}_y']]
                                  for c in ('tl', 'tr', 'br', 'bl')],
               bev_top_shift=tuning['bev_top_shift'], roi_cut=tuning['roi_cut'],
               look_shift=tuning['look_shift'], slope_ema_alpha=tuning['ema_alpha'],
               virtual_lane_width=tuning['virtual_lane_width'],
               center_ema_alpha=tuning['center_ema_alpha'],
               max_center_jump_px=tuning['max_center_jump_px'],
               max_missed_frames=tuning['max_missed_frames'],
               min_component_area=tuning['min_component_area'])


def run_case(cls, bitmap, frame, tuning):
    lane = extractor(cls, tuning)
    detections = DetectionArray([Detection(
        class_name='lane2', mask=Mask(bitmap=bitmap, width=640, height=480),
    )])
    info = lane.process(detections)
    path = PathPlanner().plan(info)
    vehicle_y = tuning['car_center_y']
    if cls is LaneInfoExtractor:
        vehicle_y += tuning.get('car_center_reference_roi_cut', tuning['roi_cut']) - tuning['roi_cut']
    motion = MotionPlanner(stanley_gain=tuning['stanley_gain'], heading_gain=tuning['heading_gain'],
                           lookahead_steps=tuning['lookahead_index'],
                           car_center_x=tuning['car_center_x'], car_center_y=vehicle_y,
                           max_steering=tuning['max_steering'], heading_step=tuning['heading_step'],
                           stanley_softening=tuning['stanley_softening'], steering_sign=tuning['steering_sign'],
                           default_left_speed=tuning['speed'], default_right_speed=tuning['speed'],
                           max_steering_angle_deg=tuning['max_steering_angle'], stop_on_red=False)
    command = motion.plan(detections, path)
    record = {'valid': info.valid, 'source': info.source,
              'targets': [[p.target_x, p.target_y] for p in info.target_points],
              'steering': command.steering, 'cte_px': motion.last_cte,
              'heading_deg': motion.last_heading_deg,
              'path': list(map(list, zip(path.x_points, path.y_points))),
              'row_counts': (lane.last_debug or {}).get('row_counts', {}),
              'virtual_samples': sum(int(s.reconstructed) for _, s in (lane.last_debug or {}).get('samples', []))}
    node = object.__new__(TrackControllerNode)
    node.lane, node.motion = lane, motion
    node._initialize_vehicle_reference(motion.car_center_y, lane.roi_cut)
    node.enabled, node.publish_bev_debug = False, True
    rendered = node._make_debug(frame, detections, info, path, command, 0., info.valid, bool(path.x_points))
    return record, rendered, lane.last_debug


def synthetic_masks(tuning):
    reference = extractor(LaneInfoExtractor, tuning)
    destination = np.float32([[192, 0], [448, 0], [448, 480], [192, 480]])
    inverse = cv2.getPerspectiveTransform(destination, np.float32(reference._src_mat()))
    for name, left, right in (
        ('straight', 250, 390), ('cross_calibration', 150, 300),
        ('outside_calibration', 80, 185), ('clip_left', 0, 200), ('clip_right', 440, 640),
        ('missing', 0, 0), ('false_region_below_roi', 470, 580),
    ):
        bird = np.zeros((480, 640), np.uint8)
        bird[:, left:right] = 255
        if name == 'false_region_below_roi':
            bird[:tuning['roi_cut']] = 0
        bitmap = cv2.warpPerspective(bird, inverse, (640, 480), flags=cv2.INTER_NEAREST)
        yield name, bitmap, np.zeros((480, 640, 3), np.uint8)
    bird = np.zeros((480, 640), np.uint8)
    for y in range(480):
        center = round(320 + .001 * (y - 300) ** 2)
        bird[y, center - 60:center + 60] = 255
    yield 'corner', cv2.warpPerspective(bird, inverse, (640, 480), flags=cv2.INTER_NEAREST), np.zeros((480, 640, 3), np.uint8)
    yield 'both_clipped', np.full((480, 640), 255, np.uint8), np.zeros((480, 640, 3), np.uint8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline-ref', default='HEAD')
    parser.add_argument('--tuning', type=Path, default=ROOT/'src/vehicle_bringup_pkg/config/track_tuning.yaml')
    parser.add_argument('--mask-dir', type=Path, default=ROOT/'reports/tuning_analysis_2026-10-05/model_comparison')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    tuning = {'max_steering': 7., 'heading_step': 3, 'car_center_y': 179.,
              'stanley_softening': .001, 'steering_sign': 1,
              **yaml.safe_load(args.tuning.read_text())['track_controller_node']['ros__parameters']}
    before_cls = baseline_class(args.baseline_ref)
    cases = list(synthetic_masks(tuning))
    for mask_path in sorted(args.mask_dir.glob('*_lane2.png')):
        frame = cv2.imread(str(mask_path.with_name(mask_path.name.split('_')[0] + '_input.jpg')))
        bitmap = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        cases.append((mask_path.stem, bitmap, frame if frame is not None else np.zeros((480, 640, 3), np.uint8)))
    report = {'baseline_revision': subprocess.check_output(['git', 'rev-parse', args.baseline_ref], cwd=ROOT, text=True).strip(),
              'tuning': tuning, 'hardware_started': False, 'canvas': [640, 480],
              'method': 'Each saved mask is evaluated independently; no ground-truth accuracy claim.',
              'cases': []}
    for name, bitmap, frame in cases:
        old, old_panel, old_debug = run_case(before_cls, bitmap, frame, tuning)
        new, new_panel, new_debug = run_case(LaneInfoExtractor, bitmap, frame, tuning)
        same_transform = np.array_equal(old_debug['inverse_transform'], new_debug['inverse_transform'])
        same_path = old['path'] == new['path']
        delta = max((abs(a[0] - b[0]) for a, b in zip(old['path'], new['path'])), default=None)
        if name in ('straight', 'corner'):
            assert same_path and old['steering'] == new['steering'], name
        assert same_transform, name
        report['cases'].append({'name': name, 'same_transform': same_transform, 'same_path': same_path,
                                'max_path_delta_px': delta, 'steering_delta': new['steering'] - old['steering'],
                                'before': old, 'after': new})
        panels = []
        for title, panel in (('BEFORE: calibration trapezoid crop', old_panel),
                             ('AFTER: full camera coverage + horizontal ROI', new_panel)):
            banner = cv2.copyMakeBorder(panel, 26, 0, 0, 0, cv2.BORDER_CONSTANT)
            cv2.putText(banner, f'{name} | {title}', (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            panels.append(banner)
        cv2.imwrite(str(args.output/f'{name}.png'), np.vstack(panels))
    (args.output/'results.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'cases': len(cases), 'same_transform': all(c['same_transform'] for c in report['cases']),
                      'changed_paths': [c['name'] for c in report['cases'] if not c['same_path']],
                      'output': str(args.output)}, indent=2))


if __name__ == '__main__':
    main()
