# 전·후방 카메라 수직주차 모델

- `parking_front.pt`: 전방 out_line / parking_lot / parking_space segmentation.
- `parking_rear.pt`: 후방 end_line / parking_lot / parking_space segmentation.
- architecture: YOLO11n-seg, pretrained COCO backbone, 640-pixel inference.
- dataset: 사용자가 제공한 Roboflow front v2 / rear v5, CC BY 4.0.
  [전방 출처](https://universe.roboflow.com/hmobility-fvu9x/parking_front/dataset/2),
  [후방 출처](https://universe.roboflow.com/hmobility-fvu9x/parking_rear/dataset/5).
- labels retain their original IDs. Runtime maps by class name; rear parking_lot
  is trained but is not consumed by the rear parking geometry.
- default paths: `package://skku_track_drive_pkg/models/parking/parking_front.pt`,
  `package://skku_track_drive_pkg/models/parking/parking_rear.pt`.
- training/evaluation provenance, hashes and per-class metrics:
  `reports/parking_training_2026-11-13/{front,rear}/` in the workspace root.

Reproduce from the workspace root with the two dataset ZIPs in Downloads and a
COCO pretrained `yolo11n-seg.pt` seed (not an existing driving/mission model):

```bash
python3 tools/train_parking_models.py --base /absolute/path/to/yolo11n-seg.pt
```

`--role front|rear|both`, `--epochs`, `--batch`, `--device`, `--downloads`, `--work`,
`--prepare-only` and `--resume` are supported. Defaults are 60 epochs, patience 15,
CUDA device 0, batch 32. Optimizer checkpoints and raw images stay under ignored datasets/.
The seed checksum and complete effective training configuration are recorded.

Test sources take priority over validation, which takes priority over training.
Cross-split Roboflow source/augmentation and exact-image overlaps are excluded
from the lower-priority split. Original sequential frames can still be correlated;
the reported split metrics do not establish independent-site performance.

The exports already include rotation, shear and salt-and-pepper augmentation.
Online training adds mild brightness/saturation/hue, ±5° rotation, translation,
scale and ±2° shear. No flipping, mosaic, mixup or copy-paste is used, to preserve
camera direction and continuous thin parking/stop lines. Validation/test images
receive only deterministic inference resizing/letterboxing.

Native datasets are front 1280×720, rear 720×1280. Camera configuration captures
1280×720 and rotates the rear image 90° clockwise before inference. Already
oriented dataset images must not be rotated again during offline evaluation.
BEV, steering sign and stop position still require vehicle calibration.
