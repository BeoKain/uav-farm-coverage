#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
seed="${1:-202}"
if [[ $# -gt 0 ]]; then shift; fi
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
[[ -f "$ros_setup" ]] || { echo "ROS setup missing: $ros_setup" >&2; exit 2; }
[[ -f "$ROS2_WS/install/setup.bash" ]] || { echo "Build px4_msgs first; see docs/INSTALL.md. Missing $ROS2_WS/install/setup.bash" >&2; exit 2; }
source "$ros_setup"
source "$ROS2_WS/install/setup.bash"
exec "$PYTHON_BIN" "$REPO_ROOT/tools/run_case.py" --seed "$seed" "$@"
