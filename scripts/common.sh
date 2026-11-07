#!/usr/bin/env bash
set -eo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$REPO_ROOT/.env" ]]; then set -a; source "$REPO_ROOT/.env"; set +a; fi
export PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
export ROS2_WS="${ROS2_WS:-$HOME/ros2_px4_ws}"
export NS3_ROOT="${NS3_ROOT:-$HOME/ns-allinone-3.43/ns-3.43}"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$REPO_ROOT/src/uav_farm${PYTHONPATH:+:$PYTHONPATH}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
