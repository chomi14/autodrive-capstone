# 모델 현황 및 migration 계획

파일을 이동/삭제/로드하지 않았다. pickle 기반 가중치의 내부 내용을 실행하지 않았다.
아래는 실제 파일 길이이며 build/install symlink 자체 길이가 아니다.
"사용"은 소스 기본값에 의한 정적 판단이고 현재 실행 중인 ROS graph/추론을 관측한 결과가 아니다.

| Path | Size (bytes) | SHA-256 | 참조 코드 | 사용 판단 |
|---|---:|---|---|---|
| 0603.pt | 6,779,572 | da6a38e034fa499e6b9372ffa31007059fe827142b8e12a95807f645d4569200 | 소스에 파일명 직접 참조 없음 | 기본 실행 경로에서 사용 확인 못 함 |
| best.pt | 6,779,572 | 377b6a225e05ce3da6209b2dfdf0e129df034e8837d12861511e54267d258f15 | camera_perception_pkg/yolov8_node.py model=best.pt | 미션 상대경로 후보; 작업 디렉터리에 따라 달라짐 |
| best0515.pt | 6,784,116 | bc4f0f183cb324a7b14fe76c7c44a9760f0cf94cee9e89e9524cfb72aff30cae | 소스에 파일명 직접 참조 없음 | 기본 실행 경로에서 사용 확인 못 함 |
| best0529.pt | 6,784,180 | 4c58d0c477f0fa654dcbceeca95e40f691aae39e3fb5eafff5a098f66d02428f | 소스에 파일명 직접 참조 없음 | 기본 실행 경로에서 사용 확인 못 함 |
| build/skku_track_drive_pkg/models/best.pt | 23,857,571 | 27d7fedda0946ded2f03f9c7ff47023b3a1f859b08574ee1c323baee9ee25e03 | colcon symlink → src/skku_track_drive_pkg/models/best.pt | 패키지 모델의 동일 파일 참조 |
| install/skku_track_drive_pkg/share/skku_track_drive_pkg/models/best.pt | 23,857,571 | 27d7fedda0946ded2f03f9c7ff47023b3a1f859b08574ee1c323baee9ee25e03 | colcon symlink → src/skku_track_drive_pkg/models/best.pt | 패키지 모델의 동일 파일 참조 |
| src/skku_track_drive_pkg/models/best.pt | 23,857,571 | 27d7fedda0946ded2f03f9c7ff47023b3a1f859b08574ee1c323baee9ee25e03 | track_controller_node.py default_model; setup.py 설치 | 트랙 launch 기본 선택 (실제 추론 미실행) |

`models/track`, `models/traffic_light`, `models/obstacle` 및 `models/model_manifest.yaml`
구조는 적절하다. 단 현재 미션 신호등 검출기는 별도 모델이 아니라 YOLO detections를
해석하므로 모델 하나를 여러 역할 폴더에 중복 복사하지 않는다. shared detector 역할도 허용한다.
트랙 모델과 루트 best.pt는 이름만 같고 크기/hash가 다르므로 대체해서는 안 된다.

1. 담당자에게 각 가중치의 학습 데이터, class mapping, 목적, 라이선스, 검증 실적을 확인한다.
2. manifest에 id, task(s), version, source, sha256, size_bytes, classes, framework,
   runtime compatibility, validation, canonical path, legacy aliases를 기록한다.
3. 평가된 모델에만 canonical id를 부여한다. 미확인 파일은 pending 상태로 유지한다.
4. feature branch에서 패키지 share 설치와 model_path/model parameter를 함께 변경한다.
   CWD와 무관하게 동일 가중치를 찾는지 확인하고 과거 경로는 이행 기간 유지한다.
5. recorded input 회귀 → 실차 저속 검증 → develop/main 승인 후 별도로 중복 정리 여부를 결정한다.

현재 단계에는 새 모델 디렉터리/manifest를 실제 권위값으로 만들지 않았다.
