#!/usr/bin/env bash
# =============================================================================
# prof_flamegraph.sh — 4B CPU flamegraph of the two VIO processes via py-spy
# (sampling, no code change, low overhead). Answers: inside the ~15s/wrist
# per-frame perception, where does CPU time go — feature extract / match /
# gtsam / serialization — and is it stuck single-threaded (GIL)?
#
# Launches ONE wrist build (perception + build_map) on an idle GPU, waits for
# steady-state, then py-spy record --pid on BOTH node processes -> SVG. Also
# runs pidstat -t (per-thread %CPU) to expose GIL-serial hot threads.
#
# Usage: prof_flamegraph.sh <episode.mcap> <gpu> [out] [duration_s]
# Run INSIDE the container (py-spy installed in /opt/venv). Needs ptrace
# (SYS_PTRACE / ptrace_scope<=1). Run on an OTHERWISE-IDLE target GPU.
# =============================================================================
set +u
EP="${1:?usage: prof_flamegraph.sh <episode.mcap> <gpu> [out] [dur]}"
GPU="${2:?need gpu}"
OUT="${3:-/data/prof_scratch/flame_out}"
DUR="${4:-25}"
mkdir -p "$OUT"

for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
ONE=/tinynav/tool/umi/_build_map_one_sensor.sh
PYSPY=/opt/venv/bin/py-spy

# launch the wrist build in the background (own domain 155)
CUDA_VISIBLE_DEVICES="$GPU" UMI_PERCEPTION_WARMUP_SEC=1 \
  bash "$ONE" "$EP" left_wrist 155 20 > "$OUT/build.log" 2>&1 &
BUILDER=$!

echo "waiting for steady-state perception (per-frame loop)..."
# wait until perception is actually processing frames (MAPPING_PERCENT>0 in build log)
for i in $(seq 1 60); do
  grep -q "MAPPING_PERCENT:[1-9]" "$OUT/build.log" 2>/dev/null && break
  sleep 1
done
sleep 2

PPID_PERC=$(pgrep -f "perception_node.py" | head -1)
PPID_BUILD=$(pgrep -f "build_map_node.py" | head -1)
echo "perception pid=$PPID_PERC  build_map pid=$PPID_BUILD"

# per-thread CPU (GIL evidence) for both, in background
pidstat -t -p "${PPID_PERC:-1}" 1 "$DUR" > "$OUT/pidstat_perception_threads.txt" 2>&1 &
pidstat -t -p "${PPID_BUILD:-1}" 1 "$DUR" > "$OUT/pidstat_build_threads.txt" 2>&1 &

# flamegraphs (native stacks too, to catch gtsam/CUDA C++)
if [ -n "$PPID_PERC" ]; then
  "$PYSPY" record -p "$PPID_PERC" -d "$DUR" -r 100 --nonblocking --native -o "$OUT/flame_perception.svg" 2>"$OUT/pyspy_perc.err" &
  PS1=$!
fi
if [ -n "$PPID_BUILD" ]; then
  "$PYSPY" record -p "$PPID_BUILD" -d "$DUR" -r 100 --nonblocking --native -o "$OUT/flame_build.svg" 2>"$OUT/pyspy_build.err" &
  PS2=$!
fi
wait "$PS1" "$PS2" 2>/dev/null

echo "=== pyspy stderr (perc) ==="; tail -3 "$OUT/pyspy_perc.err" 2>/dev/null
echo "=== pyspy stderr (build) ==="; tail -3 "$OUT/pyspy_build.err" 2>/dev/null
ls -la "$OUT"/*.svg 2>/dev/null
wait "$BUILDER" 2>/dev/null
echo "FLAME_DONE out=$OUT"
