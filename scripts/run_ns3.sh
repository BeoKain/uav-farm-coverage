#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
seed="${1:-202}"
if [[ $# -gt 0 ]]; then shift; fi
exec "$PYTHON_BIN" "$REPO_ROOT/tools/replay_ns3.py" --seed "$seed" "$@"
