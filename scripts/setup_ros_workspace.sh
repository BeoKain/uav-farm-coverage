#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
pin=7f7ebd2a372d0fa93f6c2b6a53c291b4ed4f608e
mkdir -p "$ROS2_WS/src"
if [[ ! -e "$ROS2_WS/src/px4_msgs" ]]; then
  git clone https://github.com/PX4/px4_msgs.git "$ROS2_WS/src/px4_msgs"
  git -C "$ROS2_WS/src/px4_msgs" checkout --detach "$pin"
fi
[[ "$(git -C "$ROS2_WS/src/px4_msgs" rev-parse HEAD)" == "$pin" ]] || { echo "Existing px4_msgs differs from pinned commit; use a separate ROS2_WS." >&2; exit 2; }
[[ -z "$(git -C "$ROS2_WS/src/px4_msgs" status --porcelain)" ]] || { echo "Existing px4_msgs has edits; use a separate ROS2_WS." >&2; exit 2; }
cd "$ROS2_WS"
colcon build --packages-select px4_msgs
