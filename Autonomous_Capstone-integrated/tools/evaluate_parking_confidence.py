#!/usr/bin/env python3
"""Report class-specific mask precision/recall versus confidence on VALIDATION.

Confidence is a detection score, not dataset mAP or a parking-success rate.
Threshold analysis uses validation only; test remains for final reporting.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=('front', 'rear', 'both'), default='both')
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    torch.set_num_threads(2)
    for role in ('front', 'rear') if args.role == 'both' else (args.role,):
        work = ROOT/'datasets/parking/20261113'/role
        model = YOLO(str(ROOT/f'src/skku_track_drive_pkg/models/parking/parking_{role}.pt'))
        result = model.val(data=str(work/'data.yaml'), split='val', imgsz=640,
                           batch=4, workers=2, device=args.device, plots=False,
                           project=str(work/'evaluation'), name='confidence', exist_ok=True,
                           verbose=False)
        seg = result.seg
        report = dict(split='validation', matching='mask IoU >= 0.5', classes={})
        for row, class_id in enumerate(seg.ap_class_index):
            name = model.names[int(class_id)]
            best = int(np.argmax(seg.f1_curve[row]))
            values = dict(best_f1_confidence=float(seg.px[best]),
                          best_f1=float(seg.f1_curve[row][best]),
                          mask_mAP50=float(seg.ap50[row]),
                          mask_mAP50_95=float(seg.ap[row]), thresholds={})
            for threshold in (.20, .25, .30, .35, .40, .50, .60, .70, .80, .90):
                values['thresholds'][str(threshold)] = {
                    'precision': float(np.interp(threshold, seg.px, seg.p_curve[row])),
                    'recall': float(np.interp(threshold, seg.px, seg.r_curve[row])),
                    'f1': float(np.interp(threshold, seg.px, seg.f1_curve[row])),
                }
            report['classes'][name] = values
        target = ROOT/'reports/parking_training_2026-11-13'/role/'confidence_analysis.json'
        target.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
