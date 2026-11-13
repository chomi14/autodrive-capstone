# 장애물 회피·신호등 모델 추가 학습

학습과 오프라인 검증을 완료했다. 최종 후보는
[`best_m_v3.pt`](../../src/skku_track_drive_pkg/models/mission/best_m_v3.pt)이며,
두 번째 학습의 45 epoch checkpoint다. 첫 학습의 `best_m_v2.pt`도 비교용으로 보관했다.
기존 기본 모델 `best_m.pt`는 원본 해시 그대로 보존했다.

## 최종 결과

아래는 같은 테스트 촬영 구간 179장의 비교다. 전체 AP는 Ultralytics 평가,
고정 신뢰도 결과는 실제 `YoloDetector`를 640×480·conf=0.5로 실행한 수치다.

| 지표 | 기존 모델 | 최종 후보 |
|---|---:|---:|
| 전체 box mAP50–95 | 78.60% | 84.72% |
| 전체 mask mAP50–95 | 78.79% | 86.55% |
| 장애물 검출 / 라벨 77개 | 57개 (74.03%) | 71개 (92.21%) |
| 가까운 장애물 검출 / 라벨 11개 | 11개 | 11개 |
| 신호등 검출 / 라벨 16개 | 16개 | 16개 |
| lane1 검출 / 라벨 122개 | 117개 | 120개 |
| lane2 검출 / 라벨 163개 | 160개 | 160개 |
| lane1 평균 union-mask IoU | 88.98% | 92.93% |
| lane2 평균 union-mask IoU | 92.30% | 93.96% |
| detector GPU 중앙 지연 | 6.72ms | 6.75ms |

**기본 모델 자동 교체는 보류했다.** 전체 신호등은 모두 찾지만,
현행 코드의 bbox 폭 ≥20, 하단 y≤60 필터까지 적용하면 정답 영역의 신호등 검출이
5/5→4/5로 줄었다. bbox 위치 변화로 경계에서 한 건 더 제외되는 사례다.
색상 판정이나 실제 신호 준수 성능이 개선됐다고 주장하지 않는다.
실차 실행 전에 bbox 경계와 오탐 사례를 함께 검토해야 한다.
기존 코드의 신호등 필터나 회피 파라미터는 이번 학습에서 수정하지 않았다.

라벨과 매칭되지 않은 장애물 예측은 1→21개, 신호등 예측은 5→8개다.
장애물 라벨의 실제 누락을 시각 확인했으므로 이 개수가 모두 실제 오탐을 뜻하지는 않는다.
신호등의 영역 필터를 통과한 매칭되지 않은 예측도 2→4개로 늘어 기본 교체를 보류했다.
상세한 모든 누락·매칭되지 않은 예측의 이미지 이름과 bbox는 JSON에 기록했다.

- [최종 테스트 비교 영상](conservative/runtime_test/comparison.mp4): 왼쪽 기존 / 오른쪽 최종 후보, 재생 4 FPS.
- [최종 고정 신뢰도 지표와 실패 사례](conservative/runtime_test/runtime_comparison.json).
- [최종 검증 구간 결과](conservative/runtime_val/runtime_comparison.json).
- [학습 곡선](conservative/results.png), [최종 AP 지표](conservative/candidate_metrics.json).
- [모델 선택 근거](selection.json), [기본 PT 보존·CPU 추론 확인](deployment_status.json).

두 차례 학습은 각각 51 epoch / 60 epoch, 총 약 21분 27초(GPU 학습 시간)가 걸렸다.
첫 번째는 모든 층을 학습하고, 두 번째는 backbone 10개 층을 고정했다.
최종 후보는 검증 구간에서 기존 대비 신호등·lane2 재현율을 유지/개선하며
장애물 재현율과 두 차선 IoU가 개선되는 점을 확인해 선택했다.
테스트 촬영은 첫 모델 진단 후 다시 사용했으므로 새 독립 촬영에 대한 성능 추정은 아니다.

원본: `~/Downloads/autodrive_26-2.v1i.yolov8.zip`.
기존 미션 모델 `src/skku_track_drive_pkg/models/mission/best_m.pt`의
YOLOv8n segmentation 가중치를 시작점으로 네 클래스를 함께 추가 학습한다.
차선만 인식하는 `best_v3.pt`는 이번 학습 대상이 아니다.

## 데이터 확인과 분리

950장 모두 정규화된 세그멘테이션 polygon 라벨이며 손상된 좌표나
이미지 내용의 SHA-256이 같은 중복 파일은 없다.
Roboflow 원본 분리는 729/149/72장이지만, 연속 촬영의 인접 프레임이
서로 다른 분할에 들어가 있어 촬영 구간 단위로 다시 분리했다.
파일명의 촬영 시각이 5초 이상 끊기면 새 구간으로 구분하며,
총 10개 구간 중 0–7번을 학습, 8번을 검증, 9번을 테스트에 사용한다.
분리 기준은 라벨 평가 전에 고정했다.

| 구분 | 고유 이미지 | lane1 객체 | lane2 객체 | 장애물 객체 | 신호등 객체 |
|---|---:|---:|---:|---:|---:|
| 학습 | 582 | 459 | 524 | 368 | 17 |
| 검증 | 189 | 131 | 175 | 67 | 17 |
| 테스트 | 179 | 122 | 163 | 77 | 16 |

학습과 검증의 가장 가까운 프레임 사이 간격은 163.35초,
학습과 테스트는 268.28초, 검증과 테스트는 61.63초다.
신호등 학습 이미지 17장만 총 4회씩 노출해서 epoch당 633장을 학습한다.
반복본마다 온라인 증강을 독립 적용하고 평가 데이터는 반복하지 않는다.
분할 manifest와 원본 ZIP 해시는 `dataset_audit.json`에 기록했다.

## 첫 번째 학습 설정

- 입력 640, batch 8, AdamW, 초기 학습률 0.0003, cosine 감쇠.
- 최대 100 epoch, 검증 fitness가 25 epoch 동안 개선되지 않으면 조기 종료.
- warmup 3 epoch, seed 42, 모든 층 미세조정, FP32 학습.
- 밝기 ±30%, 채도 ±20%, 크기 ±20%, 위치 ±6%, 회전 ±2도.
- 차선 클래스 의미와 신호등 색상을 보존하도록 좌우/상하 반전과 hue 변환을 끈다.
- 여러 장면의 차선과 신호등 위치가 섞이지 않도록 mosaic, mixup,
  cutmix, copy-paste를 끈다.
- 검증 fitness는 box와 mask mAP50–95의 합이며 가장 좋은 checkpoint를 선택한다.

[Ultralytics 증강 문서](https://docs.ultralytics.com/guides/yolo-data-augmentation/)와
설치된 Ultralytics 8.4.6 구현을 확인해 설정했다.
전체 설정은 `training_config.json`, GPU·패키지 버전과 모델 해시는
`provenance.json`에 있다.

첫 학습은 51 epoch에서 조기 종료됐으며 26 epoch가 선택됐다.
테스트 box mAP50–95는 78.60%→83.23%, mask mAP50–95는 78.79%→86.39%다.
640×480·신뢰도 0.5로 실제 detector를 실행하면 장애물 77개 중 검출은 57→68개,
신호등 16개 중 검출은 16→14개다. lane2의 프레임 평균 union-mask IoU는
92.30%→89.89%로 낮아져, 기존 특징을 보존하는 추가 후보를 비교하기로 했다.
누락된 신호등 두 개의 정답 bbox 하단은 각각 y=82.08, 84.23이며,
현재 제어 코드의 `traffic_max_bottom_y=60` 영역 밖에 해당한다.

두 번째 학습은 같은 원본 모델에서 시작해 backbone 앞 10개 층을 고정하고,
학습률을 0.00005로 낮추며 최대 60 epoch·patience 15를 사용한다.
증강과 데이터 분할은 동일하다. 기록은 `conservative/` 하위에 저장한다.
첫 테스트 결과를 진단한 뒤 이 추가 실험을 했으므로, 두 번째 모델의 같은 테스트
수치를 새로운 완전 독립 평가로 해석하면 안 된다. 최종 외부 검증에는 새 촬영이 필요하다.

## 재실행

```bash
python3 -u tools/train_mission_model.py
# 중단 후 optimizer 상태를 포함해 이어서 학습
python3 -u tools/train_mission_model.py --resume
# 완료된 새 모델을 실제 YoloDetector 설정으로 기존 모델과 비교
python3 tools/evaluate_mission_models.py \
  --candidate src/skku_track_drive_pkg/models/mission/best_m_v3.pt \
  --output reports/mission_training_repeat_evaluation
# 두 번째 실험과 같은 설정으로 새 실행 폴더에 학습
python3 -u tools/train_mission_model.py --run-name conservative_repeat \
  --epochs 60 --patience 15 --lr0 0.00005 --freeze 10 \
  --report reports/mission_training_repeat \
  --output-model datasets/mission/autodrive_26_2_v1_20261113/repeat.pt
```

데이터와 학습 checkpoint는 Git에서 제외된
`datasets/mission/autodrive_26_2_v1_20261113/`에 보관한다.
원본 가중치는 그 안의 `original_best_m.pt`에 보존한다.
실행 로그는 `/tmp/mission_training_20261113.log`다.
재학습은 기존 `training/` 폴더와 겹치지 않는 새 `--work` 경로를 지정한다.
또는 같은 work 안에 새 `--run-name`을 지정하면 보존된 원본 가중치를 재사용한다.
새 work에서도 동일한 초기 가중치를 사용하려면 `--initial-model`로
기존 work의 `original_best_m.pt`를 명시한다.

## 해석 범위

평가는 같은 실내 코스를 나중에 통과한 촬영 구간에 대한 비교다.
새 장소·다른 조명·다른 카메라에서의 성능이나 실차 회피 성공률은 측정하지 않았다.
기존 checkpoint의 과거 학습 데이터 전체는 확보하지 못했으므로,
기존 모델에 대해서도 완전히 새로운 데이터라고 보장할 수 없다.
원본 이미지는 이미 432×432로 늘려 저장되어 원본 영상의 세부 정보는 복원할 수 없다.
신호등은 전체 50장, 학습 구간에서는 한 번의 연속 관측에 해당한다.
신호등 bbox를 찾는 모델이며 빨강·노랑·초록 판단은 기존 코드의 별도 색상 로직이 담당한다.

검증 구간의 라벨 누락도 시각 확인했다. 예를 들어 `00000937`–`00000940`
프레임에서는 왼쪽의 가까운 실제 흰색 차량에 장애물 라벨이 없고,
멀리 있는 차량에만 라벨이 있다. `00000920` 부근에도 추가 차량이 라벨에서 빠져 있다.
`validation_label_review.jpg`의 빨간 사각형은 라벨과 매칭되지 않은 새 모델의 예측,
초록 윤곽은 공급된 장애물 라벨이다. 일부 다른 후보는 화면 가장자리라서 판단이 어렵다.
따라서 FP/정밀도 수치는 **공급된 라벨 기준**이며, FP 전체가 실제 오인식인 것은 아니다.
평가 라벨을 모델 출력에 맞춰 수정하지 않았다.
