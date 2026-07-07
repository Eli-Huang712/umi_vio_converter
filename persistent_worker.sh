#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# persistent_worker.sh — devcontainer entrypoint for persistent_worker.py.
#
# Sources ROS overlays + ensures python-mcap, then execs the orchestrator. The
# orchestrator captures this ROS-sourced environment and passes it to the
# perception/buildmap child processes it spawns (they need rclpy/rosbag2_py/
# tinynav ROS msgs), so it MUST run through this wrapper, not bare python.
#
# Usage (inside container):
#   bash tool/umi/persistent_worker.sh <ep1.mcap> [ep2.mcap ...] \
#        --ros-domain-id 90 --gpu 1 --play-rate 20
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
  if python3 -c "import mcap" >/dev/null 2>&1; then return 0; fi
  local wheel_dir="/data/vendor_wheels"
  if [[ -d "${wheel_dir}" ]]; then
    python3 -m pip install --no-index --find-links "${wheel_dir}" mcap >&2
  else
    echo "python-mcap missing and ${wheel_dir} not found" >&2; exit 1
  fi
}

source_devcontainer_setup
ensure_python_mcap
export PYTHONPATH="${REPO_DIR}:${PYTHONPATH:-}"
cd "${REPO_DIR}"
exec python3 "${SELF_DIR}/persistent_worker.py" "$@"
