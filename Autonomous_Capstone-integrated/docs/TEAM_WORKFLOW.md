# 팀 협업 규칙

Canonical Workspace: `/home/autolab/autodrive_ws/Autonomous_Capstone-integrated`.
현재 Git root는 부모 `/home/autolab/autodrive_ws`이므로 다른 workspace가 staging되지 않도록
workspace 경로를 지정해 diff/add를 검토한다. 이번 작업은 commit/merge하지 않았다.

- main = 실차 검증 완료 기준 코드.
- develop = 통합 중 코드.
- feature/* = 기능 개발 및 최소 단위 변경.
- 친구 workspace 전체를 현재 src로 복붙하지 않는다.
- 친구/과거 코드베이스는 별도 reference/upstream으로 관리한다. 현재 src 밖의 참고자료로 유지한다.
- 필요한 알고리즘만 feature branch에서 이식하고 출처/commit/license를 기록한다.
- sensor input과 vehicle output interface는 canonical workspace의 ROS_INTERFACE.md를 따른다.
- 코드 리뷰, 하드웨어 없는 빌드/회귀시험, 실제 차량 검증 후 develop/main으로 merge한다.
- 테스트 기록에는 firmware/모델 hash, 차량 calibration, 포트 profile, commit, 운영자, 결과를 기록한다.
- 파일 삭제/패키지 rename/알고리즘 교체는 별도 제안과 호환성 계획을 먼저 검토한다.
- build/, install/, log/, __pycache__/, *.pyc, dataset(s)/, Collected_Datasets/는 commit 금지.
- 이미지/동영상 결과물은 외부 dataset 저장소에 둔다. 모든 image 확장자를 ignore해서 소스 이미지까지 숨기지 않는다.
- 모델은 이동 전에 manifest 및 checksum/출처/평가 결과를 기록하고 대용량 저장소 또는 Git LFS 도입을 검토한다.

이번 감사에서 workspace 내 build/install/log/pyc/dataset 결과물은 Git tracked가 아니었다.
5개의 실제 .pt 모델은 tracked 상태다. 중복/용량만으로 지금 삭제하거나 untrack하지 않는다.

검증 명령 (하드웨어를 시작하지 않음):

```bash
cd /home/autolab/autodrive_ws/Autonomous_Capstone-integrated
source /opt/ros/humble/setup.bash
colcon build --symlink-install
g++ -std=c++11 -Wall -Wextra -Werror tools/tests/firmware_safety_test.cpp -o /tmp/firmware_safety_test
/tmp/firmware_safety_test
```

실차 승인 전 이 변경을 검증 완료 main 버전으로 취급하지 않는다.
