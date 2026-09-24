#!/usr/bin/env bash
set -eo pipefail

contest_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(cd -- "${contest_ws}/../.." && pwd)"

source /opt/ros/humble/setup.bash
source "${repo_root}/Autonomous_Capstone-integrated/install/setup.bash"
source "${contest_ws}/install/setup.bash"
set -u

exec ros2 launch camera_pkg contest_drive.launch.py "$@"
