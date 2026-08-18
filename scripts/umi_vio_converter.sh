#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# umi_vio_converter.sh — TinyNav runtime entrypoint for umi_vio_converter.py.
#
# Sources ROS overlays, checks python-mcap, then execs the Python orchestrator.
#
# Usage (inside container):
#   bash tool/umi/umi_vio_converter.sh <input.mcap> [--pose-sensors ...] [--mode auto|full|backfill|check] ...
# =============================================================================

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TINYNAV_ROOT="${TINYNAV_ROOT:-/tinynav}"
[[ -d "${TINYNAV_ROOT}" ]] || {
  echo "TinyNav root not found: ${TINYNAV_ROOT}" >&2
  exit 1
}

source_devcontainer_setup() {
  set +u
  for f in \
    /opt/ros/humble/setup.bash \
    /3rdparty/ros2_ws/install/local_setup.bash \
    /3rdparty/plotjuggler_ws/install/local_setup.bash \
    /3rdparty/message_filters_ws/install/local_setup.bash; do
    [[ -f "$f" ]] && source "$f"
  done
  set -u
}

require_python_mcap() {
  python3 -c "import mcap" >/dev/null 2>&1 || {
    echo "python-mcap is required; install dependencies in the runtime image before conversion" >&2
    exit 1
  }
}

source_devcontainer_setup
require_python_mcap

cd "${TINYNAV_ROOT}"
exec python3 "${SELF_DIR}/umi_vio_converter.py" "$@"
