#!/usr/bin/env bash
set -eo pipefail

contest_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(cd -- "${contest_ws}/../.." && pwd)"
canonical_ws="${repo_root}/Autonomous_Capstone-integrated"

source /opt/ros/humble/setup.bash

if [[ ! -f "${canonical_ws}/install/setup.bash" ]]; then
  echo "Canonical workspace is not built: ${canonical_ws}" >&2
  echo "Build it first with: cd ${canonical_ws} && colcon build --symlink-install" >&2
  exit 1
fi

source "${canonical_ws}/install/setup.bash"
set -u
cd "${contest_ws}"

# interfaces_pkg and decision_making_pkg intentionally overlay the canonical
# packages only inside this shell.  The contest LaneInfo definition differs,
# while MotionCommand is byte-for-byte compatible with vehicle_io_pkg.
colcon build \
  --symlink-install \
  --allow-overriding interfaces_pkg decision_making_pkg \
  "$@"
