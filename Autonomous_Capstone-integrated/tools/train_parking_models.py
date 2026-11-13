#!/usr/bin/env python3
"""Audit, train and evaluate front/rear parking segmentation models.

Raw datasets, caches and optimizer checkpoints stay in ignored datasets/.
Only deployment weights and reproducible reports are copied into source control.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
from zipfile import ZipFile

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
CLASSES = {
    'front': ['out_line', 'parking_lot', 'parking_space'],
    'rear': ['end_line', 'parking_lot', 'parking_space'],
}
VERSIONS = {'front': 'v2', 'rear': 'v5'}
WORK = ROOT / 'datasets/parking/20261113'
REPORT = ROOT / 'reports/parking_training_2026-11-13'
MODEL_DIR = ROOT / 'src/skku_track_drive_pkg/models/parking'


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f'Object of type {type(value).__name__} is not JSON serializable')


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False,
                               default=json_default) + '\n')


def prepare(role, archive, work):
    original = work / 'original'
    original.mkdir(parents=True, exist_ok=True)
    with ZipFile(archive) as source:
        for entry in source.infolist():
            target = (original / entry.filename).resolve()
            if not target.is_relative_to(original.resolve()):
                raise ValueError(f'Unsafe ZIP entry: {entry.filename}')
        source.extractall(original)
    metadata = yaml.safe_load((original / 'data.yaml').read_text())
    if metadata['names'] != CLASSES[role] or metadata['nc'] != len(CLASSES[role]):
        raise ValueError(f'{role}: unexpected dataset class order: {metadata}')
    entries = []
    source_splits = defaultdict(set)
    hash_splits = defaultdict(set)
    sizes = Counter()
    for split in ('train', 'valid', 'test'):
        for image in sorted((original / split / 'images').iterdir()):
            if image.suffix.lower() not in ('.jpg', '.jpeg', '.png'):
                continue
            label = original / split / 'labels' / (image.stem + '.txt')
            objects = Counter()
            for line_number, line in enumerate(label.read_text().splitlines(), 1):
                row = line.split()
                if not row:
                    continue
                cls, points = int(row[0]), list(map(float, row[1:]))
                if cls not in range(3) or len(points) < 6 or len(points) % 2 or any(
                        not math.isfinite(x) or not 0 <= x <= 1 for x in points):
                    raise ValueError(f'Invalid segmentation polygon {label}:{line_number}')
                objects[cls] += 1
            with Image.open(image) as im:
                im.verify()
            with Image.open(image) as im:
                sizes[f'{im.width}x{im.height}'] += 1
            # Roboflow's .rf suffix identifies augmentations of one source.
            source = image.stem.split('.rf.')[0]
            sha = digest(image)
            source_splits[source].add(split)
            hash_splits[sha].add(split)
            entries.append(dict(image=image, label=label, split=split,
                                source=source, sha256=sha, objects=objects))
    # Preserve test first, then validation. Remove train augmentations of any
    # held-out source, as well as validation augmentations of test sources.
    priority = {'train': 0, 'valid': 1, 'test': 2}
    def owner(splits):
        return max(splits, key=priority.__getitem__)
    audit = dict(role=role, archive=archive.name, archive_sha256=digest(archive),
                 classes=CLASSES[role], image_sizes=dict(sizes),
                 dataset_metadata=metadata,
                 policy='Preserve test > valid > train; remove lower-priority source/hash overlaps. '
                        'Native random splits are not independent capture sessions.',
                 source_overlap_groups=sum(len(s) > 1 for s in source_splits.values()),
                 exact_overlap_groups=sum(len(s) > 1 for s in hash_splits.values()),
                 original_images=dict(Counter(e['split'] for e in entries)),
                 splits={}, excluded=[], manifest=[])
    for split in ('train', 'valid', 'test'):
        selected, objects = [], Counter()
        for e in entries:
            if e['split'] != split:
                continue
            if (owner(source_splits[e['source']]) != split or
                    owner(hash_splits[e['sha256']]) != split):
                audit['excluded'].append(dict(image=e['image'].name, split=split, source=e['source']))
                continue
            selected.append(e)
            objects.update(e['objects'])
            audit['manifest'].append(dict(image=e['image'].name, split=split,
                                          source=e['source'], sha256=e['sha256']))
        if set(objects) != set(range(3)):
            raise ValueError(f'{role}/{split}: missing classes')
        (work / f'{split}.txt').write_text('\n'.join(str(e['image']) for e in selected) + '\n')
        audit['splits'][split] = dict(images=len(selected),
                                     source_images=len({e['source'] for e in selected}),
                                     backgrounds=sum(not e['objects'] for e in selected),
                                     objects={CLASSES[role][i]: objects[i] for i in range(3)})
    data = dict(path=str(work), train=str(work/'train.txt'), val=str(work/'valid.txt'),
                test=str(work/'test.txt'), names=CLASSES[role], nc=3)
    data_path = work / 'data.yaml'
    data_path.write_text(yaml.safe_dump(data, sort_keys=False))
    write_json(REPORT/role/'dataset_audit.json', audit)
    print('DATA_AUDIT', role, json.dumps({k: audit[k] for k in (
        'original_images', 'splits', 'source_overlap_groups', 'exact_overlap_groups', 'image_sizes')}), flush=True)
    return data_path


def evaluate(model_path, data, work, role, device):
    from ultralytics import YOLO
    results = {}
    for split in ('val', 'test'):
        model = YOLO(str(model_path))
        if model.task != 'segment' or list(model.names.values()) != CLASSES[role]:
            raise ValueError(f'{role}: trained model task or labels do not match')
        metrics = model.val(data=str(data), split=split, imgsz=640, batch=12,
                            device=device, workers=4, plots=True, verbose=False,
                            project=str(work/'evaluation'), name=split, exist_ok=True)
        results[split] = dict(aggregate={k: float(v) for k, v in metrics.results_dict.items()},
                             per_class=metrics.summary(), speed=metrics.speed)
    write_json(REPORT/role/'metrics.json', results)
    return results


def train(role, args):
    import torch
    from ultralytics import YOLO, __version__
    work = args.work / role
    work.mkdir(parents=True, exist_ok=True)
    data = prepare(role, args.downloads / f'parking_{role}.{VERSIONS[role]}i.yolov8.zip', work)
    if args.prepare_only:
        return
    if args.device != 'cpu' and not torch.cuda.is_available():
        raise RuntimeError('CUDA is required for the requested GPU training')
    torch.set_num_threads(4)
    seed = work / 'pretrained_yolo11n-seg.pt'
    if not seed.exists():
        shutil.copy2(args.base, seed)
    if YOLO(str(seed)).task != 'segment':
        raise ValueError('Pretrained model must be a segmentation model')
    config = dict(data=str(data), epochs=args.epochs, patience=15, imgsz=640,
                  batch=args.batch, device=args.device, workers=4, cache=False, rect=True,
                  project=str(work), name='training', exist_ok=False,
                  optimizer='AdamW', lr0=0.001, lrf=0.05, cos_lr=True,
                  weight_decay=0.0005, warmup_epochs=3.0, warmup_bias_lr=0.0,
                  seed=42, deterministic=True, amp=True,
                  hsv_h=0.01, hsv_s=0.25, hsv_v=0.35,
                  degrees=5.0, translate=0.06, scale=0.25, shear=2.0,
                  perspective=0.0, flipud=0.0, fliplr=0.0, bgr=0.0,
                  mosaic=0.0, mixup=0.0, cutmix=0.0, copy_paste=0.0, close_mosaic=0,
                  mask_ratio=2, overlap_mask=True, val=True, plots=True,
                  save=True, save_period=10)
    provenance = dict(started_utc=datetime.now(timezone.utc).isoformat(),
                      torch=torch.__version__, ultralytics=__version__,
                      gpu=torch.cuda.get_device_name(0) if args.device != 'cpu' else 'cpu',
                      base_sha256=digest(seed), base_architecture='YOLO11n-seg',
                      training_config=config, notes=[
                          'Existing Roboflow rotation/shear/noise augmentation is retained.',
                          'Additional mild photometric/geometric augmentation applies only to training.',
                          'No flipping/mosaic/mixup: preserve parking-side orientation and thin continuous lines.',
                          'No vehicle calibration or independent-site performance is implied.'])
    write_json(REPORT/role/'provenance.json', provenance)
    if args.resume and (work/'training/weights/last.pt').exists():
        previous = YOLO(str(work/'training/weights/last.pt'))
        # Ultralytics strips optimizer state and writes epoch=-1 on completion.
        # A two-role resume must also handle a finished front + unfinished rear.
        if previous.ckpt.get('epoch', -1) >= 0:
            previous.train(resume=True, device=args.device, batch=args.batch)
        else:
            print('TRAINING_ALREADY_FINISHED', role, flush=True)
    else:
        YOLO(str(seed)).train(**config)
    best = work/'training/weights/best.pt'
    evaluate(best, data, work, role, args.device)
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    target = MODEL_DIR / f'parking_{role}.pt'
    shutil.copy2(best, target)
    for name in ('results.csv', 'results.png', 'train_batch0.jpg', 'val_batch0_labels.jpg', 'val_batch0_pred.jpg'):
        source = work/'training'/name
        if source.exists():
            shutil.copy2(source, REPORT/role/name)
    provenance.update(completed_utc=datetime.now(timezone.utc).isoformat(),
                      deployed_model=str(target.relative_to(ROOT)), deployed_sha256=digest(target))
    write_json(REPORT/role/'provenance.json', provenance)
    print('TRAINING_COMPLETE', role, target, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=('front', 'rear', 'both'), default='both')
    parser.add_argument('--downloads', type=Path, default=Path('/home/autolab/Downloads'))
    parser.add_argument('--work', type=Path, default=WORK)
    parser.add_argument('--base', type=Path, required=True, help='COCO pretrained YOLO11n-seg .pt')
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--batch', type=int, default=32)
    parser.add_argument('--device', default='0')
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    args.work = args.work.resolve()
    for role in ('front', 'rear') if args.role == 'both' else (args.role,):
        train(role, args)


if __name__ == '__main__':
    main()
