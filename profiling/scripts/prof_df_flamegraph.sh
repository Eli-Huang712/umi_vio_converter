#!/usr/bin/env bash
# =============================================================================
# prof_df_flamegraph.sh — single direct-feed process: py-spy flamegraph + per-
# thread %CPU (GIL evidence), to see WHERE df's one process spends time now that
# the ROS Image-serialization (63% of stock build_map) is gone.
#
# Runs ONE df wrist on an idle GPU, grabs the single python3 pid, samples it for
# the whole run, and captures pidstat -t (per-thread). Because df is short (~10-20s)
# we attach fast and sample the entire run.
#
# Usage: prof_df_flamegraph.sh <episode.mcap> <gpu> [out] [dur]
# Run INSIDE the container (py-spy in /opt/venv). Needs SYS_PTRACE. Idle GPU.
# =============================================================================
set +u
EP="${1:?usage: prof_df_flamegraph.sh <episode.mcap> <gpu> [out] [dur]}"
GPU="${2:?need gpu}"
OUT="${3:-/data/p1_pool/df_flame}"
DUR="${4:-25}"
mkdir -p "$OUT"
for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
DRIVER=/tinynav/tool/umi/direct_feed_build_map.py
PYSPY=/opt/venv/bin/py-spy
WORK="${EP%.mcap}"; MAP="$WORK/prof_lw_db"

CUDA_VISIBLE_DEVICES="$GPU" python3 "$DRIVER" --bag_file "$EP" --sensor left_wrist \
  --map_save_path "$MAP" --no_verbose_timer > "$OUT/driver.log" 2>&1 &
DRV=$!

# find the driver python pid (this shell backgrounded python3 directly -> $DRV is it,
# but confirm comm=python3; the driver never forks a child interpreter)
PID=""
for i in $(seq 1 60); do
  if [ "$(cat /proc/$DRV/comm 2>/dev/null)" = "python3" ]; then PID=$DRV; break; fi
  sleep 0.1
done
echo "df py-pid=$PID (driver=$DRV)"

pidstat -t -p "${PID:-1}" 1 "$DUR" > "$OUT/pidstat_df_threads.txt" 2>&1 &
# sample WITH GIL (default) — shows on-CPU python work; and the raw dump for native hint
if [ -n "$PID" ]; then
  "$PYSPY" record -p "$PID" -d "$DUR" -r 100 -o "$OUT/flame_df.svg" 2>"$OUT/pyspy.err" &
  PS=$!
  # also record --idle to see wall-time distribution incl. GPU/IO waits
  "$PYSPY" record -p "$PID" -d "$DUR" -r 100 --idle -o "$OUT/flame_df_idle.svg" 2>"$OUT/pyspy_idle.err" &
  PS2=$!
  wait "$PS" "$PS2" 2>/dev/null
fi
echo "=== pyspy stderr ==="; tail -2 "$OUT/pyspy.err" 2>/dev/null
ls -la "$OUT"/*.svg 2>/dev/null
wait "$DRV" 2>/dev/null
echo "DF_FLAME_DONE out=$OUT"
