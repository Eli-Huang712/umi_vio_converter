#!/usr/bin/env bash
# =============================================================================
# prof_flamegraph_df.sh — "after" flamegraph for the P1 direct-feed path.
#
# The stock "before" (6_flame_build.svg) shows build_map spending ~63% in
# sensor_msgs/_image.py CDR (de)serialization + rclpy executor spin. Direct feed
# is ONE process (direct_feed_build_map.py) with NO ROS Image serialization and
# NO executor spin, so this flamegraph should show that block GONE — time now in
# TRT infer / gtsam / H.264 decode instead.
#
# Launches one direct-feed wrist, grabs the single python3 PID, py-spy records it,
# and also captures per-thread %CPU (pidstat -t). Because direct feed is short
# (~9s), we start sampling immediately and sample the whole run.
#
# Usage: prof_flamegraph_df.sh <episode.mcap> <gpu> [out] [duration_s]
# Run INSIDE the container (py-spy in /opt/venv). Needs SYS_PTRACE. Idle GPU.
# =============================================================================
set +u
EP="${1:?usage: prof_flamegraph_df.sh <episode.mcap> <gpu> [out] [dur]}"
GPU="${2:?need gpu}"
OUT="${3:-/data/p1_scratch/flame_df_out}"
DUR="${4:-14}"
mkdir -p "$OUT"

for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
ONE=/tinynav/tool/umi/_build_map_one_sensor.sh
PYSPY=/opt/venv/bin/py-spy

CUDA_VISIBLE_DEVICES="$GPU" UMI_DIRECT_FEED=1 \
  bash "$ONE" "$EP" left_wrist 175 20 > "$OUT/wrapper.log" 2>&1 &
BUILDER=$!

pick_py() {  # the real interpreter, not the tmux/bash/timeout wrapper
  for p in $(pgrep -f "$1"); do
    [ "$(cat /proc/$p/comm 2>/dev/null)" = "python3" ] && { echo "$p"; return; }
  done
}
echo "waiting for direct_feed_build_map.py python process..."
PID=""
for i in $(seq 1 60); do
  PID=$(pick_py "direct_feed_build_map.py")
  [ -n "$PID" ] && break
  sleep 0.2
done
echo "direct-feed py-pid=$PID (attaching)"

pidstat -t -p "${PID:-1}" 1 "$DUR" > "$OUT/pidstat_df_threads.txt" 2>&1 &
if [ -n "$PID" ]; then
  "$PYSPY" record -p "$PID" -d "$DUR" -r 50 --idle -o "$OUT/flame_df.svg" 2>"$OUT/pyspy_df.err"
fi
echo "=== pyspy stderr ==="; tail -3 "$OUT/pyspy_df.err" 2>/dev/null
ls -la "$OUT"/*.svg 2>/dev/null
wait "$BUILDER" 2>/dev/null
echo "FLAME_DF_DONE out=$OUT"
