#!/usr/bin/env python3
"""Run deployed weights through the real parking adapter and image geometry.

Uses already oriented held-out dataset images, never opens cameras or serial.
This verifies deployment and latency on dataset frames, not vehicle parking.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/skku_track_drive_pkg'))

from skku_track_drive_pkg.modes.perpendicular_camera.configuration import load_config
from skku_track_drive_pkg.modes.perpendicular_camera.model import SegmentModel, model_path
from skku_track_drive_pkg.modes.perpendicular_camera.perception import extract


def validate(role, config, work, report, count):
    view = config[role]
    adapter = SegmentModel(view)
    images = [Path(p) for p in (work/role/'test.txt').read_text().splitlines()]
    chosen = np.linspace(0, len(images)-1, min(count, len(images)), dtype=int)
    detections, geometry = Counter(), Counter()
    timings, panels, examples = [], [], []
    first_inference_ms = None
    for index in chosen:
        path = images[index]
        frame = cv2.imread(str(path))
        if frame is None:
            raise ValueError(f'Cannot read {path}')
        started = time.perf_counter()
        # These dataset frames already match the model orientation; the live
        # node rotates RAW rear camera frames before passing them to this adapter.
        polygons = adapter.predict(frame)
        obs, segmented, bev = extract(frame, polygons, view, role, int(index)+1, path.name)
        elapsed_ms = (time.perf_counter()-started)*1000
        if first_inference_ms is None:
            first_inference_ms = elapsed_ms
        else:
            timings.append(elapsed_ms)
        assert obs.valid
        h, w = frame.shape[:2]
        for name, polys in polygons.items():
            detections[name] += bool(polys)
            for poly in polys:
                points = np.asarray(poly)
                if not np.isfinite(points).all() or (points < 0).any() or (points > [w, h]).any():
                    raise ValueError(f'{role}: inference polygon outside source pixels')
        for name in ('line', 'space', 'end', 'outline'):
            line = getattr(obs, name)
            geometry[name] += line.found
            if line.found:
                assert all(np.isfinite([line.x, line.y, line.yaw]))
                assert 0 <= line.x <= 1 and 0 <= line.y <= 1
        interesting = obs.line.found if role == 'front' else obs.space.found or obs.end.found
        if interesting and len(panels) < 6:
            examples.append(str(path))
            panel = np.hstack([cv2.resize(segmented, (480, 320)), cv2.resize(bev, (480, 320))])
            cv2.putText(panel, f'{role} source={path.name[:55]}', (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
            panels.append(panel)
    if any(detections[key] == 0 for key in view['classes']):
        raise ValueError(f'{role}: no detections for a required class: {detections}')
    required_geometry = ('line',) if role == 'front' else ('space', 'end')
    if any(geometry[key] == 0 for key in required_geometry):
        raise ValueError(f'{role}: deployed model produced no usable required geometry: {geometry}')
    result = dict(role=role, model_path=model_path(view['model_path']),
                  model_names=adapter.names, used_class_mapping=view['classes'],
                  device=view['model_device'], confidence=view['confidence'],
                  inference_images=len(chosen), input_orientation='Dataset orientation; not rotated twice',
                  detected_frames=dict(detections), geometry_found_frames=dict(geometry),
                  example_source_images=examples,
                  first_pipeline_ms=first_inference_ms,
                  pipeline_ms=dict(mean=float(np.mean(timings)), p50=float(np.percentile(timings, 50)),
                                   p95=float(np.percentile(timings, 95)), max=float(np.max(timings))),
                  samples_over_observation_timeout=sum(t > config['control']['observation_timeout_s']*1000 for t in timings),
                  scope='Sequential offline inference+geometry; no USB capture/DDS/vehicle actuation. '
                        'Geometry counts are availability, not steering or localization accuracy.')
    report.mkdir(parents=True, exist_ok=True)
    (report/'runtime_validation.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    if panels:
        cv2.imwrite(str(report/'inference_geometry.jpg'), np.vstack(panels))
    print(json.dumps(result, indent=2), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=('front', 'rear', 'both'), default='both')
    parser.add_argument('--config', type=Path, default=ROOT/'src/vehicle_bringup_pkg/config/perpendicular_camera.yaml')
    parser.add_argument('--work', type=Path, default=ROOT/'datasets/parking/20261113')
    parser.add_argument('--report', type=Path, default=ROOT/'reports/parking_training_2026-11-13')
    parser.add_argument('--count', type=int, default=80)
    args = parser.parse_args()
    if args.count < 2:
        parser.error('--count must be at least 2 for warmup-excluded timing')
    cv2.setNumThreads(1)
    data = load_config(args.config)
    for role in ('front', 'rear') if args.role == 'both' else (args.role,):
        validate(role, data, args.work, args.report/role, args.count)


if __name__ == '__main__':
    main()
