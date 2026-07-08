#!/usr/bin/env bash
# =============================================================================
# prof_df_mps_ab.sh — MPS A/B for direct-feed on the CONTENDED regime.
#
# The same-GPU N-proc experiment showed df saturates ONE card at N~6-8 (per-proc
# latency +56%, sm 73%) — i.e. df is now GPU-context-serialization bound (kernels
# time-slice through per-process contexts). The report REFUTED MPS for stock, but
# ONLY because stock never loaded the GPU (sm 7%). df is a different regime, so
# MPS — one shared context, kernels run concurrently — should help NOW.
#
# A/B: N df wrists concurrent on ONE card (GPU 0), MPS OFF then ON. Same episodes,
# same N. Measures per-proc latency, aggregate throughput, GPU sm%.
#
# Usage: prof_df_mps_ab.sh <eps_dir> <N> [out]
# Run INSIDE the container on an idle node.
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; N="${2:-8}"; OUT="${3:-/data/p1_pool/df_mps_ab}"
mkdir -p "$OUT"
for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
DRIVER=/tinynav/tool/umi/direct_feed_build_map.py
MPS=/tmp/prof_mps.sh

mapfile -t EPLIST < <(find "$EPS" -name episode.mcap 2>/dev/null | sort | head -"$N")
NP=${#EPLIST[@]}

run_group() {  # tag  (env already has MPS vars if ON)
  local tag="$1"
  local gt0 gt1 lats="$OUT/lat_${tag}.txt"; : > "$lats"
  nvidia-smi dmon -s u -d 1 -o T -i 0 > "$OUT/dmon_${tag}.txt" 2>/dev/null & local DMON=$!
  gt0=$(date +%s.%N)
  local pids=()
  for ((k=0;k<N;k++)); do
    local ep="${EPLIST[$((k % NP))]}"
    ( t0=$(date +%s.%N)
      CUDA_VISIBLE_DEVICES=0 python3 "$DRIVER" --bag_file "$ep" --sensor left_wrist \
        --map_save_path "${ep%.mcap}/mps_${tag}_k${k}_db" --no_verbose_timer > "$OUT/${tag}_k${k}.log" 2>&1
      t1=$(date +%s.%N); awk -v a=$t0 -v b=$t1 'BEGIN{printf "%.1f\n",b-a}' >> "$lats" ) &
    pids+=($!)
  done
  for p in "${pids[@]}"; do wait "$p"; done
  gt1=$(date +%s.%N); kill "$DMON" 2>/dev/null
  local gwall mean thru sm
  gwall=$(awk -v a=$gt0 -v b=$gt1 'BEGIN{printf "%.1f",b-a}')
  mean=$(awk '{s+=$1;n++} END{printf "%.1f",(n?s/n:0)}' "$lats")
  thru=$(awk -v n=$N -v w=$gwall 'BEGIN{printf "%.0f",(w>0)?n*3600/w:0}')
  sm=$(awk '!/^#/ && NF>=4 {v+=$3;c++} END{printf "%.1f",(c?v/c:0)}' "$OUT/dmon_${tag}.txt")
  echo "MPS_AB_ROW,variant=$tag,N=$N,wall_s=$gwall,mean_perproc_s=$mean,throughput_wrist_per_hr=$thru,gpu_sm_pct=$sm" | tee -a "$OUT/mps_ab.csv"
}

echo "=== MPS A/B, N=$N df on GPU0 ==="
echo "--- A: MPS OFF ---"
bash "$MPS" stop >/dev/null 2>&1   # ensure off
run_group off

echo "--- B: MPS ON ---"
eval "$(bash "$MPS" start 0 2>/dev/null)"   # exports pipe dir into this env
run_group on
bash "$MPS" stop >/dev/null 2>&1

echo "=== RESULT ==="; cat "$OUT/mps_ab.csv"
echo "DF_MPS_AB_DONE"
