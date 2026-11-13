# 카메라 수직주차 모델 학습 — 2026-11-13

현재 상태: 학습 진행 중. 완료한 모델과 최종 평가는 아래 역할별 기록에 남긴다.

| 역할 | 원본 학습/검증/시험 이미지 | 중복 원본 제거 후 학습/검증/시험 | 원본 크기 |
| --- | --- | --- | --- |
| 전방 v2 | 3068 / 292 / 146 | 3068 / 292 / 146 | 1280×720 |
| 후방 v5 | 4934 / 472 / 233 | 4280 / 446 / 233 | 720×1280 |

후방의 원본 244개가 여러 split에 걸쳐 있었다. 동일 Roboflow 원본명에서 나온
증강본과 동일 파일 해시를 기준으로 test > valid > train 순서로 보존하고,
학습 654장·검증 26장을 제외했다. 정리 후 원본명/해시 기준 split 간 중복은 0이다.
각 파일의 선택 내역과 ZIP SHA-256은 `front/dataset_audit.json`,
`rear/dataset_audit.json`에 있다. 원본 프레임의 시간적 상관까지 제거한 것은
아니므로 다른 장소·조명·차량에서의 독립적인 성능 평가와는 구분한다.

## 학습과 증강

COCO 사전학습 YOLO11n-seg에서 전·후방을 독립적으로 fine-tuning한다.
640 pixels, AdamW, 최대 60 epochs, patience 15, batch 32, AMP,
mask_ratio 2로 얇은 종료선 마스크 해상도를 확보한다.
원래 라벨 ID는 전방 `[out_line, parking_lot, parking_space]`,
후방 `[end_line, parking_lot, parking_space]`이며 클래스 삭제·재번호 부여는 없다.

원본 export에는 회전 ±15°, shear ±10°, salt-and-pepper 1.01% 증강이 포함되어 있다.
학습에만 hue 0.01/saturation 0.25/value 0.35, 회전 ±5°, 이동 0.06,
scale 0.25, shear ±2°를 추가한다. 주차 방향과 연속 선 구조를 보존하도록
수평·수직 반전, mosaic, mixup, copy-paste는 끈다. 검증·시험에는 확률적 증강을
적용하지 않는다. 전체 실제 설정·라이브러리 버전·GPU·체크섬은
`{front,rear}/provenance.json`에 기록한다.
전방은 15회차까지 batch 12로 진행하고 이후 batch 32로 체크포인트를 재개했다.
재개 이력은 `front/resume_history.json`에 있으며, results.csv의 time 열은 재개 시
다시 시작하므로 전체 누적 학습시간으로 해석하지 않는다.

## 코드 연결

저장 파일은 `src/skku_track_drive_pkg/models/parking/parking_front.pt`,
`parking_rear.pt`이며 ROS package share로 설치한다. YAML 기본값은
`package://skku_track_drive_pkg/models/parking/...`으로 이 가중치를 참조한다.
실제 model.names로 ID를 매핑하고 box-only checkpoint는 거부한다.
전방은 세 클래스, 후방 geometry는 parking_space/end_line만 소비한다.
전방 raw 1280×720, 후방 raw 1280×720 → 시계방향 90° → 720×1280을 기본으로 한다.
실차 BEV·조향 부호·정지 위치 보정은 별도로 필요하다.

`reference.json`에는 원본 main commit을 기록했다. 참고한 코드는
`projects/4th_skku_contest`의 전·후방 인식, motion_parking, parking launch,
image_publisher, yolo_seg 및 주차 메시지 정의다.

## 검증 산출물

- `{front,rear}/metrics.json`: validation/test box·mask mAP 및 클래스별 지표.
- `{front,rear}/results.csv`, `results.png`: 전체 학습 이력.
- `{front,rear}/train_batch0.jpg`: 실제 학습 증강 입력.
- `{front,rear}/val_batch0_pred.jpg`: 검증 segmentation 시각화.
- `{front,rear}/runtime_validation.json`, `inference_geometry.jpg`:
  배포 모델을 실제 SegmentModel → extract 경로로 실행한 결과와 처리시간.
- `ground_truth_geometry.json`은 보정 전 YAML에 데이터셋의 정답 polygon을 넣은
  관측 가능성 점검이며 모델 정확도 지표가 아니다. 미정렬/시야 밖 주차선에서는
  중심선이 없을 수 있으므로 이것을 모든 프레임에서의 주차 성공률로 해석하지 않는다.

재현 명령:

```bash
python3 tools/train_parking_models.py --base /absolute/path/yolo11n-seg.pt
python3 tools/validate_parking_models.py
```

실차 모터 구동·USB 동시 캡처·카메라 BEV 보정·실제 주차·출차 성공률은
이 데이터셋 학습 및 오프라인 검증 범위에 포함되지 않는다.
