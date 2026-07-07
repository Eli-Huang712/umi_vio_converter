#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# umi_vio_converter.sh — devcontainer entrypoint for umi_vio_converter.py.
#
# Sources ROS overlays + ensures python-mcap (mirrors container_all_in_one_parallel.sh),
# then execs the Python orchestrator. Point the batch driver's
# VIO_RUNNER_IN_CONTAINER at this script to get tactile + 3-way auto-dispatch.
#
# Usage (inside container):
#   bash tool/umi/umi_vio_converter.sh <input.mcap> [--pose-sensors ...] [--mode auto|full|backfill|check] ...
# =============================================================================

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SELF_DIR}/../.." && pwd)"

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

ensure_python_mcap() {
  if python3 -c "import mcap" >/dev/null 2>&1; then
    return 0
  fi
  local wheel_dir="/data/vendor_wheels"
  if [[ -d "${wheel_dir}" ]]; then
    echo "Installing python mcap from ${wheel_dir}" >&2
    python3 -m pip install --no-index --find-links "${wheel_dir}" mcap >&2
  else
    echo "python-mcap missing and ${wheel_dir} not found" >&2
    exit 1
  fi
}

source_devcontainer_setup
ensure_python_mcap

cd "${REPO_DIR}"
exec python3 "${SELF_DIR}/umi_vio_converter.py" "$@"
