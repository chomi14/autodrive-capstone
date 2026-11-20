# 수직주차 후방카메라 모델 — 완료

2026-11-13, 중단된 54/60회차 체크포인트에서 옵티마이저·EMA를 복원해
55~60회차 학습을 완료했다. 전체 학습 중 검증 fitness가 가장 좋았던
`best.pt`를 `src/skku_track_drive_pkg/models/parking/parking_rear.pt`로 저장했다.
YOLO11n-seg, 클래스 순서 `0=end_line`, `1=parking_lot`, `2=parking_space`를 유지한다.
파일 크기는 5,983,325 bytes다.

| 평가 데이터 | 이미지 | Box mAP@50 | Box mAP@50:95 | Mask mAP@50 | Mask mAP@50:95 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 검증 | 446 | 98.97% | 90.15% | 92.11% | 77.08% |
| 시험 | 233 | 98.53% | 90.21% | 92.58% | 78.23% |

평가 설정은 640 pixels, batch 12, GPU 0이다. 데이터 분할은 기존 중복 원본
제외 구성을 유지했다. 재개 이후 results.csv의 time 열은 다시 시작하므로
전체 누적 학습시간으로 해석하지 않는다. 최종 모델은 마지막 회차 가중치가
아니라 Ultralytics가 전체 학습의 검증 성능으로 보존한 best 가중치다.

ROS 패키지 설치를 완료했고 기본 URI
`package://skku_track_drive_pkg/models/parking/parking_rear.pt`가 이 파일을 가리킨다.
설치본·학습 best·최종 PT의 SHA-256은 모두 같다:

```text
72098a15687b8661338a8233420f996ed04de30a89c4b3ec75e93006bd76845a
```

실제 `SegmentModel → extract` 경로로 시험 이미지 80장을 GPU 추론했다.
초기 로딩을 제외한 평균 처리시간은 32.69ms, p95는 47.27ms, 최대는 58.94ms다.
주차영역 검출은 63장, 종료선 검출은 55장이며, BEV·ROI 처리 후
주차 중심선 관측은 24장, 종료선 관측은 41장에서 생성됐다.
이 수치는 정답 대비 정확도나 주차 성공률이 아닌 관측 생성 개수다.
현재 confidence 0.5에서 검증 마스크 precision/recall은 종료선 94.79%/91.30%,
주차영역 98.55%/98.78%다. 기존 confidence 설정을 유지했다.

- `metrics.json`: 검증·시험 집계 지표. per_class의 일반 mAP 열은 box 기준이다.
- `confidence_analysis.json`: 검증 데이터의 클래스별 mask AP와 신뢰도별 precision/recall.
- `runtime_validation.json`, `inference_geometry.jpg`: 실제 인식 코드 실행 결과와 예시.
- `results.csv`, `results.png`: 총 60회 학습 이력.
- `provenance.json`, `resume_history.json`: 최초 설정, 재개 이력, 배포 체크섬.
- `completion_verification.json`: 최종 파일·설치본 확인 및 수행한 점검.

수직주차 관련 기존 테스트는 66 passed, 1 skipped였고,
`colcon build --packages-select skku_track_drive_pkg --symlink-install`도 통과했다.
원본 체크포인트와 재개 전 이력은
`datasets/parking/20261113/rear/recovery_before_resume_epoch54/`에 보존했다.
학습 로그는 `datasets/parking/20261113/rear_resume_20261113.log`다.
학습·평가·파일 저장 후 마지막 콘솔 지표 출력에서 NumPy JSON 직렬화 오류가
발생했으며, 저장된 지표 JSON·체크섬·모델 재로딩·실제 추론을 별도로 확인했다.
해당 오류는 완료된 학습이나 저장 파일에 영향을 주지 않았다.

모델 추론 재확인 명령:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/validate_parking_models.py --role rear --count 80
python3 tools/evaluate_parking_confidence.py --role rear --device 0
```

입력은 이미 회전된 데이터셋 영상이다. 실카메라 raw 1280×720 영상에는
기존 설정대로 시계방향 90° 회전 후 추론해야 한다. 실차 카메라·BEV 보정,
조향·정지 위치 검증과 실제 주차 성공률 평가는 별도로 남아 있다.
