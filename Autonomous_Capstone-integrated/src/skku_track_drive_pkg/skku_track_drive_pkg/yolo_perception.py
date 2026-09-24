"""YOLO 추론 래퍼.

PyTorch(.pt) 그대로 쓰거나, OpenVINO IR로 변환해 추론할 수 있다.
OpenVINO는 Intel CPU/iGPU/NPU에서 PyTorch CPU 추론보다 보통 2~4배 빠르므로,
GPU(CUDA)가 없는 노트북/미니PC에서 주행 프레임레이트를 올리는 데 쓴다.
(CUDA GPU가 실제로 동작하는 환경이라면 그대로 device="cuda:0" + use_openvino=False가 더 빠르다.)

OpenVINO 사용 시:
  - 최초 1회 <모델>.pt 옆에 <모델>_openvino_model/ 폴더를 자동 생성(export)하고,
    이후 실행부터는 그 폴더를 그대로 불러 쓴다(변환 재실행 없음).
  - openvino 패키지가 없거나 변환에 실패하면 경고만 출력하고 원래 .pt로 폴백한다.
    (설치: pip install openvino>=2024.0.0)
"""
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
from ultralytics import YOLO
from .messages import DetectionArray, Detection, BoundingBox2D, Pose2D, Point2D, Vector2, Mask


def openvino_model_dir(model_path: str) -> Path:
    """<모델>.pt -> <모델>_openvino_model/ 경로."""
    p = Path(model_path)
    return p.with_name(p.stem + "_openvino_model")


def _read_ov_metadata(ov_dir: Path):
    """OpenVINO IR 폴더의 metadata.yaml에서 (task, names)를 읽는다. 없으면 (None, None)."""
    meta = Path(ov_dir) / "metadata.yaml"
    if not meta.is_file():
        return None, None
    try:
        import yaml
        with open(meta, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        task = data.get("task")
        names = data.get("names")
        # yaml은 int 키를 그대로 준다({0: 'car', ...}). ultralytics도 int 키를 기대한다.
        if isinstance(names, dict):
            names = {int(k): v for k, v in names.items()}
        return task, names
    except Exception as e:
        print(f"[YOLO] metadata.yaml 읽기 실패({e})", flush=True)
        return None, None


def ensure_openvino_model(model_path: str, imgsz: int = 640, half: bool = True) -> Optional[Path]:
    """OpenVINO IR 폴더를 준비한다(없으면 변환). 실패하면 None."""
    out_dir = openvino_model_dir(model_path)
    if out_dir.is_dir() and any(out_dir.glob("*.xml")):
        return out_dir
    try:
        print(f"[YOLO] OpenVINO 변환 시작(최초 1회): {model_path} -> {out_dir.name} "
              f"(imgsz={imgsz}, half={half})", flush=True)
        exported = YOLO(model_path).export(format="openvino", imgsz=imgsz, half=half)
        path = Path(exported) if exported else out_dir
        if path.is_file():
            path = path.parent
        if path.is_dir() and any(path.glob("*.xml")):
            print(f"[YOLO] OpenVINO 변환 완료: {path}", flush=True)
            return path
        print(f"[YOLO] OpenVINO 변환 결과를 찾지 못했습니다: {exported}", flush=True)
        return None
    except Exception as e:
        print(f"[YOLO] OpenVINO 변환 실패({e}) -> 원래 .pt 모델로 계속합니다. "
              f"(필요 시: pip install openvino>=2024.0.0)", flush=True)
        return None


class YoloDetector:
    def __init__(self, model_path: str, device: str = "cpu", conf: float = 0.5,
                 use_openvino: bool = False, openvino_device: str = "intel:cpu",
                 openvino_half: bool = True, imgsz: Optional[int] = None):
        self.model_path = model_path
        self.device = device
        self.conf = conf
        self.imgsz = imgsz
        self.backend = "torch"

        loaded = None
        if use_openvino:
            ov_dir = ensure_openvino_model(model_path, imgsz=int(imgsz or 640), half=openvino_half)
            if ov_dir is not None:
                try:
                    # OpenVINO IR을 그냥 YOLO(dir)로 열면 ultralytics가 task를 자동 추정하다
                    # segment 모델을 detect로 오인해(출력 (1,39,8400)을 4+35class로 잘못 파싱)
                    # 클래스 인덱스/마스크가 깨진다. metadata.yaml의 task/names를 읽어 명시한다.
                    task, names = _read_ov_metadata(ov_dir)
                    loaded = YOLO(str(ov_dir), task=task) if task else YOLO(str(ov_dir))
                    # task를 명시하면 OpenVINO 백엔드가 metadata.yaml의 names(car/lane1/lane2)를
                    # 자동 적용한다. 혹시 비어 있으면 metadata에서 읽은 값으로 보강한다.
                    if names and not getattr(loaded, "names", None):
                        try:
                            loaded.model.names = names
                        except Exception:
                            pass
                    # "intel:cpu" / "intel:gpu" / "intel:npu" 형태면 그 장치를, 아니면 AUTO를 쓴다
                    # (ultralytics OpenVINOBackend가 device 문자열로 판단한다).
                    self.device = openvino_device
                    self.backend = "openvino"
                    # OpenVINO IR은 고정 입력 크기로 변환되므로 추론 imgsz를 맞춰준다.
                    self.imgsz = int(imgsz or 640)
                except Exception as e:
                    print(f"[YOLO] OpenVINO 모델 로드 실패({e}) -> .pt로 폴백", flush=True)
                    loaded = None

        if loaded is None:
            loaded = YOLO(model_path)
            try:
                loaded.fuse()
            except Exception:
                pass
            # 모델을 지정 device에 미리 상주시킨다(첫 프레임 지연/재이동 방지).
            if device and device != "cpu":
                try:
                    loaded.to(device)
                except Exception as e:
                    print(f"[YOLO] .to({device}) failed: {e}")

        self.yolo = loaded

        # 워밍업: 첫 프레임에서 컴파일/메모리 할당 지연이 크게 튀는 것을 미리 흡수한다.
        try:
            import numpy as _np
            side = int(self.imgsz or 640)
            _dummy = _np.zeros((side, side, 3), dtype=_np.uint8)
            self._predict(_dummy)
        except Exception:
            pass

        print(f"[YOLO] loaded: {model_path}, backend={self.backend}, "
              f"device={self.device}, imgsz={self.imgsz or 'auto'}, conf={conf}")

    def _predict(self, frame):
        kwargs = dict(
            source=frame,
            verbose=False,
            stream=False,
            conf=self.conf,
            device=self.device,
            # Preserve masks in original-image coordinates instead of the
            # letterboxed inference canvas.
            retina_masks=True,
        )
        if self.imgsz:
            kwargs["imgsz"] = self.imgsz
        return self.yolo.predict(**kwargs)[0].cpu()

    def detect(self, frame) -> DetectionArray:
        results = self._predict(frame)

        detections = DetectionArray()

        # box / class / score
        boxes = []
        if results.boxes is not None:
            for box_data in results.boxes:
                box = box_data.xywh[0]
                bbox = BoundingBox2D(
                    center=Pose2D(position=Point2D(float(box[0]), float(box[1]))),
                    size=Vector2(float(box[2]), float(box[3])),
                )
                det = Detection(
                    class_id=int(box_data.cls),
                    class_name=self.yolo.names[int(box_data.cls)],
                    score=float(box_data.conf),
                    bbox=bbox,
                )
                boxes.append(det)

        # mask가 있을 경우, box와 같은 순서라고 가정하여 넣음
        if results.masks is not None and len(boxes) > 0:
            mask_bitmaps = results.masks.data.numpy()
            orig_height = int(results.orig_img.shape[0])
            orig_width = int(results.orig_img.shape[1])
            for i, mask in enumerate(results.masks):
                if i >= len(boxes):
                    break
                polygons = mask.xy
                points = (
                    [Point2D(float(x), float(y)) for x, y in polygons[0].tolist()]
                    if polygons
                    else []
                )
                bitmap = mask_bitmaps[i]
                if bitmap.shape[:2] != (orig_height, orig_width):
                    bitmap = cv2.resize(
                        bitmap.astype(np.float32),
                        (orig_width, orig_height),
                        interpolation=cv2.INTER_NEAREST,
                    )
                bitmap = np.where(bitmap > 0.5, 255, 0).astype(np.uint8)
                boxes[i].mask = Mask(
                    data=points,
                    height=orig_height,
                    width=orig_width,
                    bitmap=bitmap,
                )

        detections.detections = boxes
        return detections
