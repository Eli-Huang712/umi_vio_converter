#!/usr/bin/env bash
# =============================================================================
# prof_df_contention.sh — locate df's soft wall by two contention experiments.
#
# EXP-A same-GPU N-proc: run N df wrists CONCURRENTLY on ONE card (N=1,2,4,6,8),
#   measure per-proc latency + aggregate throughput + GPU sm%. N<=8 uses only a
#   few CPU cores total, so CPU can't be the confound -> any per-proc latency
#   inflation here is GPU-context serialization. (Same decisive test the report
#   used to REFUTE the hypothesis for stock; now re-run for df which actually
#   uses the GPU.)
#
# EXP-B full-node latency vs PAR: per-episode df wall at PAR=1 (isolated) vs the
#   steady-state PAR=64 per-proc wall (from the earlier sweep logs). If latency
#   inflates while CPU/GPU stay unsaturated -> the wall is per-proc contention,
#   not a resource total.
#
# Usage: prof_df_contention.sh <eps_dir> <out>
# Run INSIDE the container on an OTHERWISE-IDLE node.
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; OUT="${2:-/data/p1_pool/df_contention}"
mkdir -p "$OUT"
for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
DRIVER=/tinynav/tool/umi/direct_feed_build_map.py

mapfile -t EPLIST < <(find "$EPS" -name episode.mcap 2>/dev/null | sort | head -8)
NP=${#EPLIST[@]}
echo "pool=$NP episodes"

run_df() {  # ep gpu tag map
  local ep="$1" gpu="$2" tag="$3" map="$4"
  local t0 t1
  t0=$(date +%s.%N)
  CUDA_VISIBLE_DEVICES="$gpu" python3 "$DRIVER" --bag_file "$ep" --sensor left_wrist \
    --map_save_path "$map" --no_verbose_timer > "$OUT/${tag}.log" 2>&1
  t1=$(date +%s.%N)
  awk -v a=$t0 -v b=$t1 'BEGIN{printf "%.1f", b-a}'
}
export -f run_df; export DRIVER OUT

echo "=== EXP-A: same-GPU (gpu0) N-proc df, per-proc latency + agg throughput ==="
echo "N,wall_group_s,mean_perproc_s,throughput_wrist_per_hr,gpu_sm_pct" > "$OUT/sameGPU.csv"
for N in 1 2 4 6 8; do
  # sample GPU sm% during this group
  nvidia-smi dmon -s u -d 1 -o T -i 0 > "$OUT/dmon_N${N}.txt" 2>/dev/null & DMON=$!
  gt0=$(date +%s.%N)
  pids=(); lats_file="$OUT/lat_N${N}.txt"; : > "$lats_file"
  for ((k=0;k<N;k++)); do
    ep="${EPLIST[$((k % NP))]}"
    ( lat=$(run_df "$ep" 0 "A_N${N}_k${k}" "${ep%.mcap}/prof_A_N${N}_k${k}_db"); echo "$lat" >> "$lats_file" ) &
    pids+=($!)
  done
  for p in "${pids[@]}"; do wait "$p"; done
  gt1=$(date +%s.%N); kill "$DMON" 2>/dev/null
  gwall=$(awk -v a=$gt0 -v b=$gt1 'BEGIN{printf "%.1f",b-a}')
  mean=$(awk '{s+=$1;n++} END{printf "%.1f",(n?s/n:0)}' "$lats_file")
  thru=$(awk -v n=$N -v w=$gwall 'BEGIN{printf "%.0f",(w>0)?n*3600/w:0}')
  sm=$(awk '!/^#/ && NF>=4 {v+=$3;c++} END{printf "%.1f",(c?v/c:0)}' "$OUT/dmon_N${N}.txt")
  echo "$N,$gwall,$mean,$thru,$sm" | tee -a "$OUT/sameGPU.csv"
done

echo "=== EXP-B: isolated PAR=1 per-episode df wall (8 gpus, one ep each, no contention) ==="
echo "ep,wall_s" > "$OUT/isolated.csv"
pids=()
for ((k=0;k<NP && k<8;k++)); do
  ep="${EPLIST[$k]}"
  ( lat=$(run_df "$ep" "$k" "B_iso_k${k}" "${ep%.mcap}/prof_B_k${k}_db"); echo "$(basename $(dirname $ep)),$lat" >> "$OUT/isolated.csv" ) &
  pids+=($!)
done
for p in "${pids[@]}"; do wait "$p"; done
echo "--- isolated PAR=1 df walls ---"; cat "$OUT/isolated.csv"
echo "DF_CONTENTION_DONE"
