# 트랙 모델 best_v3.pt 추가 학습 결과

기존 `src/skku_track_drive_pkg/models/best_v2.pt`에서 시작해 추가 학습했다.
클래스는 `0: lane2`, `1: traffic_light`로 유지했다. 원본 v2 모델과 별도 백업,
원본 JY 이미지·라벨·ZIP 및 이전 v2 재학습 데이터의 체크섬을 확인했다.
원본 데이터는 수정하거나 삭제하지 않고, 별도 학습용 복사본을 만들었다.

- 결과: `src/skku_track_drive_pkg/models/best_v3.pt`
- v2 백업: `datasets/lanedetector/v3_20261024/original_best_v2.pt`
- 학습: 50 epoch 완료, 검증 fitness 기준 epoch 38 선택.
- AdamW, lr 0.0001, batch 4, imgsz 640, freeze 10, seed 42, GPU RTX 4050.
- 원본 v2의 설정을 유지하며, 좌우/상하 반전과 mosaic/mixup은 사용하지 않았다.

## 데이터 구성

새 JY 데이터는 UUID 촬영 묶음 두 개와 기타 원본 파일 계열을 각각 학습,
테스트, 검증으로 나누었다. 같은 촬영 묶음을 서로 다른 분할에 섞지 않았다.
UUID 생성 시각을 실제 촬영 시각이나 FPS로 해석하지 않았다.
이전 SM 데이터는 기존 학습 분할만 재학습에 넣었고, 검증/테스트는 평가에만
사용했다. 기존 SM 분할의 2초 이상 경계 간격도 유지했다.

| 데이터 | 학습 | 검증 | 테스트 |
|---|---:|---:|---:|
| 새 JY | 374 | 72 | 123 |
| 이전 SM | 263 | 52 | 51 |
| 합계 | 637 | 124 | 174 |

다음 이미지 네 장은 새 학습·검증·테스트 모두에서 제외했다.

- `193_speed_130_steer_2_e1dcb8c8-d487-11ee-9354-f927ddc9dc6d_png.rf.e7277089f071cd3a920f9cb62a1df756.jpg`
- `202_speed_130_steer_6_e34ae9a4-d487-11ee-9354-f927ddc9dc6d_png.rf.c4dec7f91fab48c2c6a27fcb822830fb.jpg`
- `199_speed_130_steer_6_e34ae9a1-d487-11ee-9354-f927ddc9dc6d_png.rf.32ff35f1569019d0d921bfddb8f7aea8.jpg`
- `sample62_jpg.rf.c1dfedd9afcceb6a28691a74b7e890bc.jpg`

## 성능 비교

동일한 분할에서 평가한 마스크 mAP50-95이다.

| 분할 | 데이터 | 클래스 | v2 | v3 | 변화(%p) |
|---|---|---|---:|---:|---:|
| val | jy | lane2 | 95.62% | 95.57% | -0.05 |
| val | jy | traffic_light | 74.20% | 81.93% | +7.72 |
| val | sm | lane2 | 99.40% | 99.45% | +0.05 |
| val | sm | traffic_light | 79.67% | 80.34% | +0.68 |
| test | jy | lane2 | 99.23% | 99.22% | -0.01 |
| test | jy | traffic_light | 60.69% | 78.22% | +17.53 |
| test | sm | lane2 | 98.05% | 98.79% | +0.74 |
| test | sm | traffic_light | 87.95% | 90.40% | +2.45 |

새 JY 테스트 차선 mAP는 99.23%에서 99.22%로 사실상 유지됐다.
신호등 mAP는 60.69%에서 78.22%로 개선됐고, 보고된 재현율은 68.83%에서
100.00%로 높아졌지만 정밀도는 87.67%에서 85.47%로 낮아졌다.
정밀도·재현율은 평가기가 선택한 기준의 값이며 실제 주행 confidence에서의
고정 임계값 성능과 동일하지 않다. JY 테스트 신호등 객체 수는 31개다.
이전 SM 테스트에서도 차선과 신호등 mAP가 개선됐다.

출처 구분은 파일명에 근거하며 원본 촬영 메타데이터가 완전하지 않다.
장소/시각적 특징이 겹칠 수 있고, 조상 모델 best.pt의 최초 학습 데이터는
없으므로 과거 학습 프레임과의 중복까지 배제할 수 없다. 새로운 촬영 회차와
실차 주행에 대한 독립 검증은 아직 수행하지 않았다.

## 원본 보존 및 실행 산출물

학습 중 별도 워크스페이스 정리로 보조 보호 경로 `models/best.pt`가 없어져
최종 저장 직전 검사가 중단됐다. 실제 학습 원본인 `models/best_v2.pt`와 백업은
동일한 체크섬을 유지했다. 예전 best.pt의 기존 백업도 체크섬을 확인했고,
삭제된 파일은 복원하지 않았다. 학습을 다시 하지 않고 완료된 38 epoch 후보의
전체 평가·원본 v2·원본 데이터 검사를 확인한 뒤 v3 파일을 생성했다.

로컬 상세 산출물은 `datasets/lanedetector/v3_20261024/`에 있다.
`manifest.json`은 제외 목록·그룹·입력 체크섬, `evaluation.json`은 클래스별
평가, `training/args.yaml`과 `training/results.csv`는 학습 조건과 경과다.
학습 기본 모델 경로는 변경하지 않았다.

## SHA-256

- 원본 best_v2.pt 및 백업: `d5befac7d90ddc483df74ed8f623273c0b6adb6640c59e456a34e44fdcc37e9f`
- best_v3.pt: `83ed83beae553f4220fa15de52859934a9769065902d5898f275d22567b8c7b3`

패키지 설치 경로 등록과 실제 YoloDetector를 통한 CPU 샘플 추론을 통과했다. 카메라·시리얼·모터는 열지 않았다.
