# 트랙 모델 v2

`src/skku_track_drive_pkg/models/best_v2.pt`는 같은 폴더의 기존 `best.pt`를
초기 가중치로 사용해 추가 학습한 YOLOv8s 세그멘테이션 모델이다.
클래스는 기존과 동일한 `0: lane2`, `1: traffic_light`다. 기존 `best.pt`는
변경하지 않았고 원본 데이터와 백업의 SHA-256 보존 검사를 통과했다.

현재 `track_controller_node.py`의 기본 모델은 `best_v2.pt`다.
새 모델 파일을 설치 경로에 등록하려면 다음 명령을 실행한다.

```bash
colcon build --symlink-install --packages-select skku_track_drive_pkg
source install/setup.bash
```

## 데이터와 학습

- 입력: `datasets/lanedetector/segdata_sm/` 아래 COCO 세그멘테이션 폴더 두 개.
- COCO `2: lane2`를 YOLO `0: lane2`, COCO `4: traffic_light`를 YOLO
  `1: traffic_light`로 변환했다. lane1/obstacles/상위 분류는 제외했다.
- 신호등이 obstacles로 표시된
  `frame_20260629_175232_299_jpg.rf.cSIqpxE5HFH03BzuNrDg.jpg`는 전체 제외했다.
- 시간 구간별 학습/검증/테스트는 263/52/51장이다. 학습 프레임이 검증/테스트
  프레임과 2초 이상 떨어지도록 경계 구간 15장은 사용하지 않았다.
- 세 분할 모두 lane2와 traffic_light를 포함한다.
- RTX 4050에서 AdamW, 학습률 0.0001, batch 4, imgsz 640, freeze 10으로
  학습했다. 좌우/상하 반전, mosaic, mixup은 사용하지 않았다.
- 최대 50 epoch, patience 12로 실행하여 34 epoch에서 조기 종료했다.
  검증 fitness 기준 최적 epoch 22의 체크포인트를 v2로 복사했다.
- 학습 당시 Ultralytics 8.4.6, PyTorch 2.5.1+cu124를 사용했다.

## 평가

아래 값은 마스크 mAP50-95이다.

| 분할 | 클래스 | 원본 | v2 |
|---|---|---:|---:|
| 검증 | lane2 | 99.40% | 99.40% |
| 검증 | traffic_light | 77.27% | 79.67% |
| 테스트 | lane2 | 95.18% | 98.05% |
| 테스트 | traffic_light | 85.53% | 87.95% |

신호등 테스트 객체는 16개이며, 보고된 마스크 재현율은 93.37%에서 87.50%로
낮아졌다. mAP 향상은 실제 운용 confidence에서 재현율 향상을 보장하지 않는다.
모든 이미지가 한 촬영 세션에서 나왔으므로 다른 조명/카메라/촬영 회차 또는
실차 주행 성능에 대한 독립 검증 결과로 해석하지 않는다.

원본 데이터와 실행 산출물은 저장소 제외 규칙에 따라 커밋하지 않는다.
로컬 상세 기록은 `datasets/lanedetector/v2_20261017/`의 `manifest.json`,
`evaluation.json`, `training/args.yaml`, `training/results.csv`에 있다.

## SHA-256

| 파일 | SHA-256 |
|---|---|
| 원본 `best.pt` | `27d7fedda0946ded2f03f9c7ff47023b3a1f859b08574ee1c323baee9ee25e03` |
| `best_v2.pt` | `d5befac7d90ddc483df74ed8f623273c0b6adb6640c59e456a34e44fdcc37e9f` |
