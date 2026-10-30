# BEV 보정과 ROI 분리 검증 — 2026-10-16

보정 사다리꼴 바깥의 정상 차선이 버려지지 않도록, 원근 변환 보정과
검출 범위 제한을 분리했다. 비교 기준은 수정 전 커밋
`90ee48c9f339ac7670fb45781414b432ed80bc27`이다.

## 처리 방식

- 네 보정점, `bev_top_shift`, 변환 목적 좌표와 640×480 계산 캔버스를 유지했다.
- 원본 차선 마스크 전체를 BEV로 변환한다. 보정 사다리꼴 밖을 미리 지우지 않는다.
- 기존 `roi_cut` 가로선 위쪽 제외, 중심선 피팅·평활화, CubicSpline,
  `lookahead_index`, Stanley 코드와 튜닝 파일을 유지했다.
- 원본 영상 전체를 같은 행렬로 변환한 **카메라 관측 마스크**로 화각/캔버스 경계를 구한다.
  이는 별도 도로 마스크가 아니다. 도로·하늘 등을 임의로 분류하거나 보정 영역으로 자르지 않는다.
  마스크 정리 과정에서 카메라 관측 밖으로 픽셀이 번지는 것만 제거한다.
- 한쪽 관측 경계에 닿으면 기존 가상 폭으로 복원한다(`clipped_left/right`).
  좁은 내부 검출을 이전 중심으로 추정하는 기존 동작은 유지하고,
  `incomplete_detection`으로 구분한다. 실제 원인을 완벽히 식별하는 분류기는 아니다.
- 완전 누락은 기존 프레임 유지/만료 정책을 사용한다. 양쪽 관측 경계가 모두 잘리면
  화면 중앙을 실제 양쪽 경계의 중심으로 채택하지 않고 기존 유지/만료 정책으로 넘긴다.
- 장애물 회피의 차선 변경 경로 생성과 정지 정책은 수정하지 않았다.

## 디버그 표시

카메라 화면의 주황 사다리꼴은 `BEV calibration only`로 표시한다.
BEV 화면에서는 주황 직사각형이 보정 영역, 회색 테두리가 카메라 관측 범위,
자홍 가로선이 ROI 절단선이다. 카메라 화면에도 해당 절단선을 역투영한다.
초록 점은 실제 경계, 주황 점은 가상 경계, 빨간 점은 평활화 중심이다.
`clip`과 `infer`, 행별 관측 종류 수, `observed/held/invalid` 상태로 복원 근거를 확인한다.
차선 검출이 없어도 관측 범위와 ROI를 표시한다.

## 검증 결과

관련 오프라인 테스트 **91개 통과**, `git diff --check` 통과.
카메라, ROS 노드, serial sender 및 차량 구동을 시작하지 않았다.
노드 객체를 생성하지 않고 렌더링 메서드와 순수 계산만 호출했다.

별도 비교 도구로 합성 9개와 저장된 실제 차선 마스크 12장, 총 21개를 비교했다.
모든 사례에서 이전/이후 BEV 변환행렬이 동일했다. 설정은 저장된 개인 튜닝이 아닌
저장소 `track_tuning.yaml`과 컨트롤러 기본값을 사용했다.

| 사례 | 결과 |
|---|---|
| 보정 영역 내부 직선·곡선 | 기존 경로와 조향 결과 동일 |
| 보정 영역 경계를 가로지르는 차선 | 불필요한 가상 복원 대신 실제 양쪽 경계 사용 |
| 보정 영역 밖의 차선 | 기존에는 경로 없음, 수정 후 정상 관측 경로 생성 |
| 실제 카메라/캔버스 경계의 좌·우 잘림 | 가상 복원 유지, 합성 사례의 기존 경로·조향 동일 |
| 검출 누락 | 새 가상차선을 만들지 않고 이전 중심 유지 후 만료 |
| 관측 영역 양쪽을 가득 채운 마스크 | 가짜 중앙 관측을 거부하고 이전 중심 유지 또는 invalid |
| ROI 위쪽 검출 | 경로 계산에서 제외 |
| ROI 아래의 일관된 큰 오탐 | 정상 차선과 픽셀만으로 구분할 수 없어 경로로 채택될 수 있음 |

저장 마스크 12장 중 11장에서 경로가 달라졌다. 같은 경로 높이의 최대 X 차이는
63px이고 조향 명령 차이는 최대 4단계(-7..+7 체계)였다.
예를 들어 `000000_track_lane2`는 가상 경계 샘플 3개가 실제 경계 샘플로 바뀌며
조향이 +4에서 +3으로 변했다. 좌표계와 게인을 유지해도 사용 관측이 달라지면
제어 출력은 달라진다. 이 비교는 각 저장 마스크를 독립 평가한 것으로,
연속 주행 성능이나 정답 중심선 대비 정확도를 입증하지 않는다.

## 캔버스와 남은 한계

검증한 보정 영역 밖 차선은 기존 캔버스 안에서 처리할 수 있어 자동 확장은 하지 않았다.
기존 차량·경로 좌표를 그대로 유지하며 `bev_pad`는 계속 시각화 전용이다.
BEV 화면 밖으로 투영되는 차선 정보는 여전히 잘린다. 현재 검증으로 모든 코너와
모든 옆 차선이 640px 안에 들어온다고 보장하지 않으며, 확장은 별도 좌표·표시 검증이 필요하다.

요청 범위에 따라 별도 도로/지평선 마스크를 추가하지 않았다.
가로선 아래에 들어오는 큰 오탐이나 잘못된 투영은 모두 제거되지 않는다.
기존 마스크 정리는 가장 큰 연결 성분을 선택하므로 이런 오탐이 정상 차선보다 크면
선택될 수 있다. 한쪽 차선 복원 폭과 내부 검출의 좌우 추정도 기존 휴리스틱을 사용한다.
옆 차선 관측 범위를 넓힌 것만으로 회피 경로 생성·정지 문제가 모두 해결되는 것은 아니다.

설치된 패키지는 이 작업공간 소스에 심볼릭 링크되어 있음을 확인했다.
실행 중인 프로세스는 Python 코드를 다시 읽도록 재시작해야 한다.

## 후속 수정: ROI 변경 시 차량 기준점 고정

기존에는 제어 차량 Y를 잘라낸 ROI의 고정값으로 사용해,
ROI 변경 시 전체 BEV에서 차량 기준점도 함께 이동했다.
이제 제어용 차량 Y는 다음처럼 계산한다.

```text
전체 BEV 차량 Y = car_center_y + car_center_reference_roi_cut
현재 ROI 차량 Y = 전체 BEV 차량 Y - roi_cut
```

기존 튜닝 파일에 `car_center_reference_roi_cut`이 없으면 최초 실행의 `roi_cut`을
기준값으로 사용하므로 그 실행의 초기 차량 위치와 제어 동작을 유지한다.
기본값 `car_center_y=179`, 기준 ROI=300이면 전체 BEV Y=479이며,
ROI를 250으로 바꿨을 때 내부 제어 Y만 229로 보정한다.
카메라 화면과 BEV 화면의 차량 십자가는 같은 위치를 유지한다.

튜너의 P/save는 차량 Y와 기준 ROI도 저장한다. 예를 들어 아래 설정은
재실행 후에도 전체 BEV 차량 Y=479를 유지한다.

```yaml
roi_cut: 250
car_center_y: 179.0
car_center_reference_roi_cut: 300
```

`car_center_y` 파라미터는 저장된 **기준 ROI에서의 값**으로 유지하고,
실제 `MotionPlanner.car_center_y`만 현재 ROI 좌표로 변환한다.
차량 Y를 명시적으로 조절하면 기준 ROI에 대한 값으로 해석한다.
BEV 보정점을 바꾸면 원근 변환 자체가 달라지므로, 이 보정이 카메라 공간의
동일한 물리 위치까지 보장하는 것은 아니다.

추가 오프라인 테스트 7개는 트랙·미션·bbox 모드의 ROI 변경, 동일 물리 경로의
Stanley CTE/heading/조향 불변성, 두 화면 십자가 위치, 잘못된 파라미터 거부,
GUI 저장/설정 재로딩 후 기준점 유지를 검증한다.
ROI 변경으로 관측·샘플링할 경로 범위는 달라질 수 있어 실제 조향이 항상 같다는 뜻은 아니다.
기존 주행·미션 회귀와 launch 조립을 포함한 최종 검사에서는 **116개 테스트가 통과**했다.
기본 속도 테스트는 운영자의 개인 저장 파일에 영향을 받지 않도록 저장소 기본 YAML을
명시하며, 실제 개인 튜닝 파일은 수정하지 않았다.

## 재현

작업공간 루트에서:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_LOG_DIR=/tmp/bev_roi_tests
export PYTHONPATH="$PWD/src/skku_track_drive_pkg:$PWD/src/vehicle_bringup_pkg:$PWD/src/vehicle_io_pkg:$PWD/tools/tests:${PYTHONPATH:-}"
python3 -m pytest -q src/skku_track_drive_pkg/test \
  tools/tests/test_bev_source_tuning.py tools/tests/test_bbox_1009_integration.py \
  tools/tests/test_tuning_revision.py tools/tests/test_tuning_analysis.py
python3 -m pytest -q tools/tests/test_vehicle_roi_reference.py
# 후속 수정의 launch 조립 회귀까지 포함하려면 위 테스트들과 함께 실행:
python3 -m pytest -q tools/tests/test_four_mode_launches.py tools/tests/test_bbox_launch_assembly.py
python3 tools/validate_bev_roi_separation.py \
  --baseline-ref 90ee48c9f339ac7670fb45781414b432ed80bc27 \
  --output /tmp/bev_roi_separation_validation
git diff --check
```

비교 도구는 `results.json`과 사례별 전후 비교 PNG를 출력한다.
현재 환경에서 기존 SciPy/NumPy 버전 조합 경고가 발생하지만 검증은 통과했다.
