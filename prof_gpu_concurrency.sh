#!/usr/bin/env bash
# =============================================================================
# prof_gpu_concurrency.sh — the DECISIVE GPU-context-serialization experiment.
#
# Runs N single-wrist build_map processes CONCURRENTLY on ONE GPU (distinct
# episodes, distinct ROS domains, all pinned to the same card) and measures:
#   - per-process build_map wall time (latency inflation as N grows)
#   - aggregate GPU sm% via `nvidia-smi dmon -s u` @1Hz (integral = GPU-busy-s)
#   - aggregate throughput (N wrists / max wall)
# Signature test:
#   * pure context-serialization -> aggregate sm% ~flat vs N, per-proc latency
#     scales ~linearly with N, throughput flat.
#   * genuine spatial overlap    -> sm% climbs with N, per-proc latency ~flat,
#     throughput climbs until SM saturates.
# Reused for the MPS A/B: set CUDA_MPS via prof_mps.sh before calling.
#
# Usage: prof_gpu_concurrency.sh <scratch_eps_dir> <gpu> <N> <domain_base> [out]
# Run INSIDE the container on an OTHERWISE-IDLE target GPU.
# =============================================================================
set +u
EPS="${1:?usage: prof_gpu_concurrency.sh <eps_dir> <gpu> <N> <domain_base> [out]}"
GPU="${2:?need gpu}"
N="${3:?need N concurrent procs}"
DBASE="${4:-100}"
OUT="${5:-/data/prof_scratch/conc_out}"
TAG="n${N}${CUDA_MPS_PIPE_DIRECTORY:+_mps}"
mkdir -p "$OUT/$TAG"

for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
ONE=/tinynav/tool/umi/_build_map_one_sensor.sh

# NB: uses the first N size-sorted episodes, so different N use different episode
# SETS (size mix varies) -> cross-N raw throughput carries episode-mix noise. The
# CLEAN comparisons are: (a) MPS off-vs-on at the SAME N/episodes, (b) the sm%
# engine-activity curve (episode-independent), (c) per-proc latency inflation.
mapfile -t EPLIST < <(find "$EPS" -name episode.mcap 2>/dev/null | sort | head -n "$N")
[ "${#EPLIST[@]}" -ge "$N" ] || { echo "need >=$N episodes, have ${#EPLIST[@]}"; exit 1; }

echo "=== concurrency N=$N on GPU $GPU (MPS=${CUDA_MPS_PIPE_DIRECTORY:-off}) ==="
# dmon sampler: sm% + mem% @1Hz for the target GPU
nvidia-smi dmon -i "$GPU" -s u -d 1 -o T > "$OUT/$TAG/dmon.txt" 2>/dev/null &
DMON=$!
sleep 1
T0=$(date +%s.%N)
pids=()
for i in $(seq 0 $((N-1))); do
  ep="${EPLIST[$i]}"
  dom=$((DBASE + i))
  # each proc: one wrist (left) on the shared GPU, own domain+workdir; timed
  ( s=$(date +%s.%N)
    CUDA_VISIBLE_DEVICES="$GPU" UMI_PERCEPTION_WARMUP_SEC=1 \
      bash "$ONE" "$ep" left_wrist "$dom" 20 > "$OUT/$TAG/proc${i}.log" 2>&1
    e=$(date +%s.%N)
    awk -v s=$s -v e=$e -v i=$i 'BEGIN{printf "proc%d wall=%.1f\n", i, e-s}' > "$OUT/$TAG/proc${i}.wall"
  ) &
  pids+=($!)
done
for p in "${pids[@]}"; do wait "$p"; done
T1=$(date +%s.%N)
kill "$DMON" 2>/dev/null

MAXWALL=$(awk -v a=$T0 -v b=$T1 'BEGIN{printf "%.1f", b-a}')
# per-proc wall stats
cat "$OUT/$TAG"/proc*.wall > "$OUT/$TAG/walls.txt" 2>/dev/null
MEDWALL=$(awk '{print $2}' "$OUT/$TAG"/proc*.wall 2>/dev/null | sed 's/wall=//' | sort -n | awk '{a[NR]=$1} END{print (NR%2)?a[int(NR/2)+1]:(a[NR/2]+a[NR/2+1])/2}')
# dmon integral: sm% column. Header lines start with #; columns: time gpu sm mem enc dec
SMINT=$(awk '!/^#/ && NF>=4{ sm=$3+0; s+=sm/100*1; n++; if(sm>mx)mx=sm } END{printf "%.1f %.0f %.0f", s, (n?s*100/n:0), mx}' "$OUT/$TAG/dmon.txt")
read GBS SMAVG SMMAX <<< "$SMINT"
# throughput: N wrists in MAXWALL -> wrists/hr and ep/hr (2 wrists/ep)
THR=$(awk -v n=$N -v w=$MAXWALL 'BEGIN{printf "%.0f", n*3600/w}')
echo "RESULT_ROW,$TAG,N=$N,gpu=$GPU,mps=${CUDA_MPS_PIPE_DIRECTORY:+on},maxwall_s=$MAXWALL,med_proc_wall_s=$MEDWALL,gpu_busy_s=$GBS,sm_avg_pct=$SMAVG,sm_max_pct=$SMMAX,wrists_per_hr=$THR" | tee -a "$OUT/results.csv"
echo "CONC_DONE tag=$TAG"
