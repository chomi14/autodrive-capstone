"""Ultralytics segment adapter; actual class IDs are resolved from names."""
from pathlib import Path


def model_path(value):
    path = Path(value).expanduser()
    if not value or not path.is_file():
        raise FileNotFoundError('Parking segmentation model missing; set front/rear.model_path in camera YAML')
    if path.suffix.lower() not in ('.pt', '.onnx', '.engine'):
        raise ValueError('Use an Ultralytics segment .pt/.onnx/.engine export')
    return str(path.resolve())


class SegmentModel:
    def __init__(self, cfg):
        path = model_path(cfg['model_path'])
        from ultralytics import YOLO
        self.model = YOLO(path, task='segment')
        self.cfg = cfg
        # Accessing names may initialize an exported backend. It deliberately
        # fails early if runtime dependencies or metadata are unavailable.
        names = self.model.names
        self.names = dict(enumerate(names)) if isinstance(names, list) else names
        missing = set(cfg['classes'].values()) - set(self.names.values())
        if missing:
            raise ValueError(f'Model missing required segment classes {sorted(missing)}; actual={self.names}')
        self.canonical = {actual: canonical for canonical, actual in cfg['classes'].items()}

    def predict(self, frame):
        result = self.model.predict(frame, device=self.cfg['model_device'],
                                    conf=self.cfg['confidence'], verbose=False)[0]
        polygons = {name: [] for name in self.cfg['classes']}
        if result.boxes is None or len(result.boxes) == 0:
            return polygons
        if result.masks is None:
            raise RuntimeError('Model returned boxes without segmentation masks')
        if len(result.masks.xy) != len(result.boxes):
            raise RuntimeError('Mask/box count mismatch')
        for class_id, poly in zip(result.boxes.cls.cpu().tolist(), result.masks.xy):
            name = self.canonical.get(self.names[int(class_id)])
            if name is not None:
                polygons[name].append(poly)
        return polygons
