#!/usr/bin/env python3
"""Compare mission checkpoints through the actual detector at runtime settings."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/skku_track_drive_pkg'))
from skku_track_drive_pkg.yolo_perception import YoloDetector

NAMES = ['lane1', 'lane2', 'obstacle', 'traffic_light']
COLORS = [(255, 180, 0), (0, 200, 255), (0, 255, 0), (220, 0, 220)]


def iou(a, b):
    overlap = np.maximum(np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2]), 0).prod()
    return float(overlap / max(np.prod(a[2:] - a[:2]) + np.prod(b[2:] - b[:2]) - overlap, 1e-9))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work', type=Path, default=ROOT / 'datasets/mission/autodrive_26_2_v1_20261113')
    parser.add_argument('--candidate', type=Path, default=ROOT / 'src/skku_track_drive_pkg/models/mission/best_m_v2.pt')
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/mission_training_2026-11-13')
    parser.add_argument('--split', choices=['val', 'test'], default='test')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(4)
    models = {'baseline': args.work / 'original_best_m.pt', 'candidate': args.candidate}
    detectors = {name: YoloDetector(str(path), device='cuda:0', conf=0.5,
                                    use_openvino=False, imgsz=640) for name, path in models.items()}
    counts = {name: {cls: Counter() for cls in NAMES} for name in models}
    near_counts = {name: Counter() for name in models}
    light_counts = {name: Counter() for name in models}
    latencies = {name: [] for name in models}
    mask_scores = {name: {cls: [] for cls in NAMES[:2]} for name in models}
    images = [Path(line) for line in (args.work / f'{args.split}.txt').read_text().splitlines()]
    images.sort(key=lambda image: int(image.stem.split('_')[1]))
    video = cv2.VideoWriter(str(args.output / 'comparison.mp4'),
                            cv2.VideoWriter_fourcc(*'mp4v'), 4.0, (1280, 480))
    failures = []
    false_positives = []
    try:
        for image_path in images:
            frame = cv2.resize(cv2.imread(str(image_path)), (640, 480))
            label_path = image_path.parent.parent / 'labels' / (image_path.stem + '.txt')
            truth = []
            for line in label_path.read_text().splitlines():
                values = line.split()
                polygon = np.asarray(values[1:], dtype=float).reshape(-1, 2) * [640, 480]
                truth.append((int(values[0]), np.r_[polygon.min(0), polygon.max(0)], polygon))
            panels = []
            for name, detector in detectors.items():
                torch.cuda.synchronize()
                started = time.perf_counter()
                predictions = detector.detect(frame).detections
                torch.cuda.synchronize()
                latencies[name].append(1000 * (time.perf_counter() - started))
                used = set()
                eligible_truth = {index for index, (cls, box, _) in enumerate(truth)
                                  if cls == 3 and box[2]-box[0] >= 20 and box[3] <= 60}
                used_lights = set()
                painted = frame.copy()
                class_masks = [np.zeros(frame.shape[:2], np.uint8) for _ in NAMES]
                for prediction in sorted(predictions, key=lambda detection: -detection.score):
                    cls = prediction.class_id
                    if NAMES[cls] != prediction.class_name:
                        raise AssertionError('Runtime class mapping changed')
                    if prediction.mask.bitmap is None or prediction.mask.bitmap.shape != frame.shape[:2]:
                        raise AssertionError('Runtime mask is missing or has incorrect coordinates')
                    center = prediction.bbox.center.position
                    width, height = prediction.bbox.size.x, prediction.bbox.size.y
                    box = np.array([center.x-width/2, center.y-height/2, center.x+width/2, center.y+height/2])
                    options = [(iou(box, gt), index) for index, (cid, gt, _) in enumerate(truth)
                               if cid == cls and index not in used]
                    best_iou, match = max(options, default=(0, -1))
                    if cls == 3 and width >= 20 and box[3] <= 60:
                        if best_iou >= 0.5 and match in eligible_truth:
                            light_counts[name]['tp'] += 1
                            used_lights.add(match)
                        else:
                            light_counts[name]['fp'] += 1
                    if best_iou >= 0.5:
                        used.add(match)
                        counts[name][NAMES[cls]]['tp'] += 1
                        if cls == 2 and truth[match][1][3] >= 300:
                            near_counts[name]['tp'] += 1
                    else:
                        counts[name][NAMES[cls]]['fp'] += 1
                        if cls in [2, 3]:
                            false_positives.append(dict(model=name, image=image_path.name,
                                class_name=NAMES[cls], bbox=box.tolist(), confidence=prediction.score))
                        if cls == 2 and box[3] >= 300:
                            near_counts[name]['fp'] += 1
                    class_masks[cls] |= (prediction.mask.bitmap > 0).astype(np.uint8)
                    x1, y1, x2, y2 = box.astype(int)
                    cv2.rectangle(painted, (x1, y1), (x2, y2), COLORS[cls], 2)
                    cv2.putText(painted, f'{NAMES[cls]} {prediction.score:.2f}', (x1, max(44, y1-4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLORS[cls], 1, cv2.LINE_AA)
                for index, (cls, gt_box, _) in enumerate(truth):
                    if index not in used:
                        counts[name][NAMES[cls]]['fn'] += 1
                        if cls == 2 and gt_box[3] >= 300:
                            near_counts[name]['fn'] += 1
                        if cls in [2, 3]:
                            failures.append(dict(model=name, image=image_path.name,
                                                 class_name=NAMES[cls], bbox=gt_box.tolist()))
                light_counts[name]['fn'] += len(eligible_truth - used_lights)
                for cls in range(2):
                    gt_mask = np.zeros(frame.shape[:2], np.uint8)
                    for cid, _, polygon in truth:
                        if cid == cls:
                            cv2.fillPoly(gt_mask, [polygon.astype(np.int32)], 1)
                    union = np.count_nonzero(gt_mask | class_masks[cls])
                    if union:
                        mask_scores[name][NAMES[cls]].append(float(np.count_nonzero(gt_mask & class_masks[cls]) / union))
                cv2.rectangle(painted, (0, 0), (640, 28), (0, 0, 0), -1)
                cv2.putText(painted, f'{name} | conf=0.5 | 640x480', (8, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
                panels.append(painted)
            video.write(np.hstack(panels))
    finally:
        video.release()
    report = dict(split=args.split, images=len(images), confidence=0.5, match_bbox_iou=0.5,
                  input_resolution=[640, 480], inference_imgsz=640,
                  mask_metric='Mean per-frame union-mask IoU; absent GT with prediction counts as zero',
                  models={})
    for name in models:
        classes = {}
        for cls, counter in counts[name].items():
            tp, fp, fn = (counter[k] for k in ['tp', 'fp', 'fn'])
            classes[cls] = dict(tp=tp, fp=fp, fn=fn,
                                precision=tp/max(1, tp+fp), recall=tp/max(1, tp+fn))
        report['models'][name] = dict(classes=classes,
            near_obstacle_bbox_bottom_at_least_300={key: near_counts[name][key] for key in ['tp', 'fp', 'fn']},
            traffic_light_controller_region_width_20_bottom_60={key: light_counts[name][key] for key in ['tp', 'fp', 'fn']},
            lane_mask_iou={cls: float(np.mean(values)) for cls, values in mask_scores[name].items()},
            detector_latency_ms=dict(median=float(np.median(latencies[name])),
                                     p95=float(np.percentile(latencies[name], 95))))
    report['missed_objects'] = failures
    report['false_positive_objects'] = false_positives
    (args.output / 'runtime_comparison.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({name: value for name, value in report.items()
                      if name not in ['missed_objects', 'false_positive_objects']}, indent=2))


if __name__ == '__main__':
    main()
