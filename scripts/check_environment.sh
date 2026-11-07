#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ -f "$ROS2_WS/install/setup.bash" ]]; then source "$ROS2_WS/install/setup.bash"; fi
exec "$PYTHON_BIN" "$REPO_ROOT/tools/check_environment.py"
