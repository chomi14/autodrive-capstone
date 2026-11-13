#!/usr/bin/env python3
"""Audit and fine-tune the four-class mission segmentation model locally.

Run from any directory. Raw data, checkpoints and optimizer states stay under
datasets/; the comparison and provenance are saved under reports/.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import os
from pathlib import Path
import shutil
from zipfile import ZipFile

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
CLASSES = ['lane1', 'lane2', 'obstacle', 'traffic_light']
DEFAULT_WORK = ROOT / 'datasets/mission/autodrive_26_2_v1_20261113'
REPORT = ROOT / 'reports/mission_training_2026-11-13'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def prepare(archive, work):
    original = work / 'original'
    with ZipFile(archive) as source:
        for entry in source.infolist():
            if not (original / entry.filename).resolve().is_relative_to(original.resolve()):
                raise ValueError(f'Unsafe ZIP entry: {entry.filename}')
        source.extractall(original)
    metadata = yaml.safe_load((original / 'data.yaml').read_text())
    if metadata['names'] != CLASSES:
        raise ValueError(f'Unexpected classes: {metadata["names"]}')
    records, hashes = [], defaultdict(list)
    for image in sorted(original.glob('*/images/*.jpg')):
        label = image.parent.parent / 'labels' / f'{image.stem}.txt'
        rows = [line.split() for line in label.read_text().splitlines() if line.strip()]
        counts = Counter()
        for row in rows:
            cls, points = int(row[0]), np.asarray(row[1:], dtype=float)
            if (cls not in range(4) or len(points) < 6 or len(points) % 2
                    or not np.isfinite(points).all() or points.min() < 0 or points.max() > 1):
                raise ValueError(f'Invalid polygon: {label}')
            counts[cls] += 1
        timestamp = int(image.stem.split('_')[1])
        records.append(dict(image=image, label=label, timestamp=timestamp, counts=counts))
        hashes[sha256(image)].append(image)
    if len(records) != 950 or any(len(paths) > 1 for paths in hashes.values()):
        raise ValueError('Unexpected image count or exact duplicate images')

    # Gaps >5 s identify ten continuous captures. Hold out complete later
    # captures, including one traffic-light encounter in each evaluation split.
    # Splitting random adjacent frames would overstate generalization.
    records.sort(key=lambda item: item['timestamp'])
    groups = []
    for record in records:
        if not groups or record['timestamp'] - groups[-1][-1]['timestamp'] > 5_000_000_000:
            groups.append([])
        groups[-1].append(record)
    if len(groups) != 10 or [len(group) for group in groups] != [8, 131, 37, 54, 98, 124, 126, 4, 189, 179]:
        raise ValueError('Capture grouping changed; review held-out groups')
    audit = dict(archive=str(archive), archive_sha256=sha256(archive),
                 classes=CLASSES, source_images=len(records), exact_duplicates=0,
                 original_splits={s: len(list((original / s / 'images').glob('*.jpg')))
                                  for s in ['train', 'valid', 'test']},
                 grouping='Complete captures separated by >5 seconds; group 8 validation, group 9 test',
                 source_preprocessing='Roboflow stretched images to 432x432; original aspect ratio unavailable',
                 splits={}, manifest=[])
    partitions = defaultdict(list)
    for group_id, group in enumerate(groups):
        split = 'val' if group_id == 8 else 'test' if group_id == 9 else 'train'
        for record in group:
            partitions[split].append(record)
            audit['manifest'].append(dict(image=record['image'].name, split=split,
                                          group=group_id, timestamp=record['timestamp']))
    for split, entries in partitions.items():
        directory = work / 'grouped' / split
        image_paths = []
        objects, images_per_class = Counter(), Counter()
        for record in entries:
            # Four exposures per epoch for scarce traffic-light training frames;
            # each exposure gets independent online augmentation. Never duplicate
            # evaluation images or import held-out frames into training.
            copies = 4 if split == 'train' and record['counts'][3] else 1
            for repeat in range(copies):
                name = record['image'].stem + (f'_repeat{repeat}' if repeat else '')
                for kind, source in [('images', record['image']), ('labels', record['label'])]:
                    target = directory / kind / (name + source.suffix)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    if not target.exists():
                        os.link(source, target)
                image_paths.append(str(directory / 'images' / (name + record['image'].suffix)))
            objects.update(record['counts'])
            images_per_class.update(record['counts'].keys())
        if set(objects) != set(range(4)):
            raise ValueError(f'Missing evaluation/training classes in {split}')
        (work / f'{split}.txt').write_text('\n'.join(image_paths) + '\n')
        audit['splits'][split] = dict(unique_images=len(entries), exposures=len(image_paths),
                                     objects={CLASSES[k]: v for k, v in objects.items()},
                                     images_per_class={CLASSES[k]: v for k, v in images_per_class.items()})
    data = dict(path=str(work), train=str(work / 'train.txt'),
                val=str(work / 'val.txt'), test=str(work / 'test.txt'), names=CLASSES, nc=4)
    data_path = work / 'grouped.yaml'
    data_path.write_text(yaml.safe_dump(data, sort_keys=False))
    write_json(REPORT / 'dataset_audit.json', audit)
    return data_path


def evaluate(model_path, data, work, name, device):
    from ultralytics import YOLO
    result = {}
    for split in ['val', 'test']:
        metrics = YOLO(str(model_path)).val(
            data=str(data), split=split, imgsz=640, batch=8, device=device,
            workers=2, plots=True, project=str(work / 'evaluation'),
            name=f'{name}_{split}', exist_ok=True, verbose=False)
        result[split] = dict(aggregate={key: float(value) for key, value in metrics.results_dict.items()},
                             per_class=metrics.summary(), speed=metrics.speed)
    write_json(REPORT / f'{name}_metrics.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, default=Path('/home/autolab/Downloads/autodrive_26-2.v1i.yolov8.zip'))
    parser.add_argument('--work', type=Path, default=DEFAULT_WORK)
    parser.add_argument('--device', default='0')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    work = args.work.resolve()
    REPORT.mkdir(parents=True, exist_ok=True)
    data = prepare(args.archive.resolve(), work)
    print('DATA_AUDIT', (REPORT / 'dataset_audit.json').read_text().split('"manifest"')[0], flush=True)
    if args.prepare_only:
        return
    import torch
    from ultralytics import YOLO, __version__
    if args.device != 'cpu' and not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable. Run outside the device-restricted sandbox.')
    torch.set_num_threads(4)
    baseline = work / 'original_best_m.pt'
    if not baseline.exists():
        shutil.copy2(ROOT / 'src/skku_track_drive_pkg/models/mission/best_m.pt', baseline)
    existing = YOLO(str(baseline))
    if existing.task != 'segment' or list(existing.names.values()) != CLASSES:
        raise ValueError('Mission model task/classes do not match dataset')
    provenance = dict(torch=torch.__version__, ultralytics=__version__,
                      baseline_sha256=sha256(baseline), baseline=str(baseline),
                      gpu=torch.cuda.get_device_name(0) if args.device != 'cpu' else 'cpu')
    write_json(REPORT / 'provenance.json', provenance)
    if not (REPORT / 'baseline_metrics.json').exists():
        evaluate(baseline, data, work, 'baseline', args.device)
    config = dict(data=str(data), epochs=args.epochs, patience=25, imgsz=640,
                  batch=8, device=args.device, workers=2, cache=False,
                  project=str(work), name='training', exist_ok=False,
                  optimizer='AdamW', lr0=0.0003, lrf=0.05, cos_lr=True,
                  weight_decay=0.0005, warmup_epochs=3.0, warmup_bias_lr=0.0,
                  seed=42, deterministic=True, amp=False, freeze=None,
                  hsv_h=0.0, hsv_s=0.20, hsv_v=0.30,
                  degrees=2.0, translate=0.06, scale=0.20,
                  shear=0.0, perspective=0.0, flipud=0.0, fliplr=0.0,
                  bgr=0.0, mosaic=0.0, mixup=0.0, cutmix=0.0, copy_paste=0.0,
                  close_mosaic=0, mask_ratio=4, overlap_mask=True,
                  val=True, plots=True, save=True, save_period=10)
    write_json(REPORT / 'training_config.json', config)
    if args.resume:
        model = YOLO(str(work / 'training/weights/last.pt'))
        model.train(resume=True, device=args.device)
    else:
        model = YOLO(str(baseline))
        model.train(**config)
    best = work / 'training/weights/best.pt'
    candidate = ROOT / 'src/skku_track_drive_pkg/models/mission/best_m_v2.pt'
    evaluate(best, data, work, 'candidate', args.device)
    shutil.copy2(best, candidate)
    shutil.copy2(work / 'training/results.csv', REPORT / 'results.csv')
    for filename in ['results.png', 'train_batch0.jpg', 'val_batch0_pred.jpg']:
        source = work / 'training' / filename
        if source.exists():
            shutil.copy2(source, REPORT / filename)
    provenance.update(candidate=str(candidate), candidate_sha256=sha256(candidate))
    write_json(REPORT / 'provenance.json', provenance)
    print(f'TRAINING_COMPLETE candidate={candidate}', flush=True)


if __name__ == '__main__':
    main()
