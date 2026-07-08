#!/usr/bin/env bash
# =============================================================================
# prof_effective_util.sh — measure TRUE effective util on ONE card under a fixed
# number of pipeline workers, with high-frequency sampling, MPS off vs on.
#
# Motivation: `dmon -s u` sm% is coarse (1 Hz, "≥1 SM busy" fraction) and UNDER-
# reports under MPS (concurrent kernels finish in shorter denser bursts -> fewer
# active sample intervals despite MORE real work). To answer "is effective util
# >80%?" honestly we sample GPU util at 10 Hz (nvidia-smi -lms 100) AND integrate
# it, on a single card kept continuously fed by a steady N-worker pipeline queue.
#
# Also records CPU busy on that run. Compares MPS off vs on at the SAME N.
#
# Usage: prof_effective_util.sh <eps_dir> <N> <dur_s> [gpu]
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; N="${2:-8}"; DUR="${3:-180}"; GPU="${4:-0}"
for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
DRIVER=/tinynav/tool/umi/direct_feed_build_map.py
MPS=/tmp/prof_mps.sh
OUT=/data/p1_pool/eff_util; mkdir -p "$OUT"

mapfile -t EPLIST < <(find "$EPS" -name episode.mcap 2>/dev/null | sort)
NP=${#EPLIST[@]}

feed_card() {  # tag: keep exactly N pipeline workers in flight on GPU $GPU for DUR
  local tag="$1"
  local end=$(( $(date +%s) + DUR ))
  local launch=0; declare -A P=()
  local done_file="$OUT/${tag}.completions"; : > "$done_file"
  while [ "$(date +%s)" -lt "$end" ]; do
    for pid in "${!P[@]}"; do kill -0 "$pid" 2>/dev/null || unset P[$pid]; done
    while [ "${#P[@]}" -lt "$N" ]; do
      ep="${EPLIST[$((launch % NP))]}"
      ( CUDA_VISIBLE_DEVICES=$GPU timeout 220 python3 "$DRIVER" --bag_file "$ep" \
          --sensor left_wrist --map_save_path "${ep%.mcap}/eff_${tag}_db" --no_verbose_timer \
          >/dev/null 2>&1; echo x >> "$done_file" ) &
      P[$!]=1; launch=$((launch+1))
    done
    sleep 0.5
  done
  for pid in "${!P[@]}"; do wait "$pid" 2>/dev/null; done
}

sample_and_run() {  # tag
  local tag="$1"
  # 10 Hz GPU util (util.gpu + sm via dmon in parallel), mpstat for CPU
  nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits \
    -lms 100 -i "$GPU" > "$OUT/gpu10hz_${tag}.txt" 2>/dev/null & local G=$!
  mpstat 1 > "$OUT/mpstat_${tag}.txt" 2>/dev/null & local M=$!
  feed_card "$tag"
  kill "$G" "$M" 2>/dev/null
  # effective GPU util = mean of 10Hz utilization.gpu over the run (trim first 20%)
  local gutil cpub comp
  gutil=$(awk -F, 'NR>1{v[NR]=$1} END{s=int(NR*0.2); n=0; for(i=s;i<=NR;i++){sum+=v[i];n++} printf "%.1f",(n?sum/n:0)}' "$OUT/gpu10hz_${tag}.txt")
  cpub=$(awk '/all/ && $NF ~ /^[0-9.]+$/ {idle+=$NF;n++} END{printf "%.1f",(n?100-idle/n:0)}' "$OUT/mpstat_${tag}.txt")
  comp=$(wc -l < "$OUT/${tag}.completions")
  echo "EFF_UTIL_ROW,tag=$tag,N=$N,gpu_util_10hz_pct=$gutil,cpu_busy_pct=$cpub,completed=$comp"
}

echo "=== effective util, N=$N pipeline workers, one card (gpu $GPU), dur ${DUR}s ==="
bash "$MPS" stop >/dev/null 2>&1
echo "--- MPS off ---"; sample_and_run off
echo "--- MPS on ---"
eval "$(bash "$MPS" start "$GPU" 2>/dev/null)"
sample_and_run on
bash "$MPS" stop >/dev/null 2>&1
echo "EFF_UTIL_DONE"
