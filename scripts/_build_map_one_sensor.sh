#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# _build_map_one_sensor.sh — run tinynav perception + build_map for ONE sensor.
#
# Extracted verbatim (behaviour-preserving) from container_all_in_one_parallel.sh
# `run_build_map_for_sensor`, generalized from the two hard-coded wrists to any
# sensor name. Produces <work_dir>/<sensor>_db/poses.npy.
#
# Usage:
#   _build_map_one_sensor.sh <input.mcap> <sensor_name> [ros_domain_id] [play_rate] [repo_dir]
#
# Runs inside the tinynav devcontainer. Sources ROS overlays itself.
# =============================================================================

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <input.mcap> <sensor_name> [ros_domain_id] [play_rate] [repo_dir]" >&2
  exit 2
fi

input_mcap="$1"
sensor_name="$2"
ros_domain_id="${3:-${ROS_DOMAIN_ID:-}}"
play_rate="${4:-${UMI_BUILD_MAP_PLAY_RATE:-20}}"
repo_dir="${5:-${TINYNAV_ROOT:-/tinynav}}"
self_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${input_mcap}" != *.mcap ]]; then
  echo "input must be an .mcap file: ${input_mcap}" >&2
  exit 2
fi

work_dir="${input_mcap%.mcap}"
mkdir -p "${work_dir}"

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
source_devcontainer_setup

shell_quote() { printf %q "$1"; }

if [[ -n "${ros_domain_id}" ]]; then
  ros_domain_export="export ROS_DOMAIN_ID=$(shell_quote "${ros_domain_id}")"
else
  ros_domain_export=":"
fi

# Propagate the GPU pin INTO the tmux command. tmux panes inherit the tmux
# server's environment (set when the server first started), NOT the caller's,
# so under parallel workers CUDA_VISIBLE_DEVICES from `docker exec -e ...` would
# otherwise be lost and every worker would pile onto GPU 0. Re-exporting it here
# guarantees each build_map/perception process pins to its intended card.
if [[ -n "${CUDA_VISIBLE_DEVICES:-}" ]]; then
  cuda_export="export CUDA_VISIBLE_DEVICES=$(shell_quote "${CUDA_VISIBLE_DEVICES}")"
else
  cuda_export=":"
fi

ros_setup="set +u
  export PYTHONUNBUFFERED=1
  ${ros_domain_export}
  ${cuda_export}
  for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
    [ -f \"\$f\" ] && source \"\$f\"
  done
  set -u"

map_dir="${work_dir}/${sensor_name}_db"
status_file="${work_dir}/${sensor_name}_build.status"
perception_log="${work_dir}/${sensor_name}_perception.log"
build_log="${work_dir}/${sensor_name}_build.log"
session_name="umi_${input_mcap##*/}_${sensor_name}"
session_name="${session_name//[^A-Za-z0-9_]/_}"
if [[ -n "${ros_domain_id}" ]]; then
  session_name="${session_name}_d${ros_domain_id}"
fi
done_sig="${session_name}_done"

echo "==> Build map ${sensor_name}"
rm -rf "${map_dir}" "${status_file}"
: > "${perception_log}"
: > "${build_log}"
tmux kill-session -t "${session_name}" 2>/dev/null || true

# ---------------------------------------------------------------------------
# Direct-feed path (opt-in: UMI_DIRECT_FEED=1). One process decodes each
# frame to numpy and feeds perception + build_map in-process/synchronously,
# bypassing the two-process tmux + DDS Image-serialization path. VIO math is
# unchanged. Wrapped in a `timeout` watchdog: there is no perception-subscriber
# handshake to hang on here, but a wedged GPU/decoder must never hold the card
# forever. The stock two-process path stays the default below.
# ---------------------------------------------------------------------------
if [[ "${UMI_DIRECT_FEED:-0}" == "1" ]]; then
  driver="${self_dir}/direct_feed_build_map.py"
  timeout_sec="${UMI_BUILD_MAP_TIMEOUT_SEC:-600}"
  echo "    mode: direct-feed (single process), timeout=${timeout_sec}s" >&2
  echo "    build log: ${build_log}" >&2
  # Isolate DDS: single-process direct feed has no cross-process topic traffic,
  # but rclpy.init() still joins a domain and allocates /dev/shm segments. Pin a
  # unique domain so parallel workers don't share discovery / shm.
  if [[ -n "${ros_domain_id}" ]]; then
    export ROS_DOMAIN_ID="${ros_domain_id}"
  fi
  set +e
  timeout "${timeout_sec}" python3 "${driver}" \
    --bag_file "${input_mcap}" --sensor "${sensor_name}" \
    --map_save_path "${map_dir}" --no_verbose_timer > "${build_log}" 2>&1
  st=$?
  set -e
  if [[ "${st}" == "124" ]]; then
    echo "direct-feed timed out after ${timeout_sec}s for ${sensor_name}" >&2
  fi
  if [[ "${st}" != "0" ]]; then
    echo "direct-feed build_map failed for ${sensor_name} (status=${st})" >&2
    tail -n 60 "${build_log}" >&2 || true
    exit "${st}"
  fi
  echo "    poses: ${map_dir}/poses.npy" >&2
  exit 0
fi

rq="$(shell_quote "${repo_dir}")"
iq="$(shell_quote "${input_mcap}")"
mq="$(shell_quote "${map_dir}")"
sf="$(shell_quote "${status_file}")"
ds="$(shell_quote "${done_sig}")"
pr="$(shell_quote "${play_rate}")"
sn="$(shell_quote "${sensor_name}")"
pl="$(shell_quote "${perception_log}")"
bl="$(shell_quote "${build_log}")"

tmux new-session -d -s "${session_name}" -n "${sensor_name}" \
  "${ros_setup}; cd ${rq} && python3 tinynav/core/perception_node.py > ${pl} 2>&1"

tmux split-window -t "${session_name}:0" -h \
  "${ros_setup}; cd ${rq} && sleep 5; set +e
   python3 tinynav/core/build_map_node.py --bag_file ${iq} --sensor ${sn} --map_save_path ${mq} --play_rate ${pr} --no_verbose_timer > ${bl} 2>&1
   status=\$?; echo \$status > ${sf}
   tmux wait-for -S ${ds}
   exit \$status"
tmux select-layout -t "${session_name}:0" even-horizontal >/dev/null

echo "    domain: ${ros_domain_id:-default}" >&2
echo "    perception log: ${perception_log}" >&2
echo "    build log: ${build_log}" >&2
tmux wait-for "${done_sig}"

st="$(cat "${status_file}")"
tmux kill-session -t "${session_name}" 2>/dev/null || true
if [[ "${st}" != "0" ]]; then
  echo "build_map failed for ${sensor_name} (status=${st})" >&2
  tail -n 60 "${build_log}" >&2 || true
  exit "${st}"
fi
echo "    poses: ${map_dir}/poses.npy" >&2
