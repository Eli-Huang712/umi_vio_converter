#!/usr/bin/env bash
# =============================================================================
# prof_par_sweep.sh — full-node saturation point with REAL utilization.
#
# Runs a pool of FULL 2-wrist episodes at parallelism PAR across NGPU GPUs and
# measures, unlike the old report which only had loadavg/nvidia-smi-util:
#   - throughput (ep/hr) from completed count / wall
#   - REAL CPU %busy = 100 - mean(mpstat %idle)   (immune to D-state loadavg)
#   - REAL GPU engine activity = mean sm% across all NGPU via nvidia-smi dmon
#   - cgroup CPU core-seconds over the batch, failures
# Each worker: 1 episode, 2 wrists SEQUENTIAL (the stock FULL path), pinned to
# gpu = slot%NGPU, ROS domain = DOMAIN_BASE + slot (kept <=232). CUDA pin is
# re-exported inside _build_map_one_sensor's tmux (the documented gotcha).
#
# Usage: prof_par_sweep.sh <eps_dir> <PAR> <n_eps> [ngpu] [domain_base] [out]
# Run INSIDE the container on an OTHERWISE-IDLE node (needs all NGPU free).
# =============================================================================
set +u
EPS="${1:?usage: prof_par_sweep.sh <eps_dir> <PAR> <n_eps> [ngpu] [dbase] [out]}"
PAR="${2:?need PAR}"
NEP="${3:?need n_eps}"
NGPU="${4:-8}"
DBASE="${5:-40}"
OUT="${6:-/data/prof_scratch/sweep_out}"
CG=/sys/fs/cgroup
mkdir -p "$OUT/par${PAR}"

for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
ONE=/tinynav/tool/umi/_build_map_one_sensor.sh

mapfile -t EPLIST < <(find "$EPS" -name episode.mcap 2>/dev/null | sort | head -n "$NEP")
NEP=${#EPLIST[@]}
echo "=== PAR=$PAR n_eps=$NEP ngpu=$NGPU dbase=$DBASE ==="
[ "$NEP" -ge 1 ] || { echo "no episodes"; exit 1; }

# domain safety: every slot gets a GLOBALLY-UNIQUE domain (DBASE + slot), never
# recycled. All episodes are named episode.mcap, so _build_map_one_sensor's tmux
# session name is disambiguated ONLY by domain — reusing a domain while its prior
# holder is still running makes the newcomer's `tmux kill-session` murder the
# running build (and causes ROS topic cross-talk). Unique domains eliminate both.
# Cap: DBASE + NEP <= 232 (FastDDS ROS_DOMAIN_ID hard limit).
if [ $((DBASE + NEP)) -gt 232 ]; then
  echo "DBASE($DBASE)+NEP($NEP)>232 (FastDDS cap): lower DBASE or NEP"; exit 1
fi

# clean any prior products so every ep is FULL (idempotent path would SKIP)
for ep in "${EPLIST[@]}"; do rm -rf "${ep%.mcap}"/*_db "${ep%.mcap}"/*_with_pose.mcap 2>/dev/null; done

read_cpu_usec() { awk '/usage_usec/{print $2}' "$CG/cpu.stat"; }

# one worker = one episode, both wrists sequential, pinned to its gpu+domain
worker() {
  local slot="$1" ep="$2"
  local gpu=$((slot % NGPU))
  local dom=$((DBASE + slot))          # GLOBALLY-UNIQUE domain (no recycle)
  local rc=0
  for sensor in left_wrist right_wrist; do
    CUDA_VISIBLE_DEVICES="$gpu" UMI_PERCEPTION_WARMUP_SEC=1 UMI_BUILD_MAP_TIMEOUT_SEC=300 \
      bash "$ONE" "$ep" "$sensor" "$dom" 20 > "$OUT/par${PAR}/slot${slot}_${sensor}.log" 2>&1 || rc=1
  done
  echo "$rc" > "$OUT/par${PAR}/slot${slot}.rc"
}
export -f worker
export ONE OUT PAR NGPU DBASE

# background samplers: mpstat %idle @1s, dmon sm% across all gpus @1s
mpstat 1 > "$OUT/par${PAR}/mpstat.txt" 2>/dev/null &
MPSTAT=$!
nvidia-smi dmon -s u -d 1 -o T > "$OUT/par${PAR}/dmon.txt" 2>/dev/null &
DMON=$!
sleep 2

C0=$(read_cpu_usec); T0=$(date +%s.%N)
# feed slots through xargs -P PAR
i=0
for ep in "${EPLIST[@]}"; do echo "$i|$ep"; i=$((i+1)); done | \
  xargs -d '\n' -P "$PAR" -I{} bash -c 'IFS="|" read -r slot ep <<< "{}"; worker "$slot" "$ep"'
T1=$(date +%s.%N); C1=$(read_cpu_usec)
kill "$MPSTAT" "$DMON" 2>/dev/null

WALL=$(awk -v a=$T0 -v b=$T1 'BEGIN{printf "%.1f",b-a}')
OK=$(grep -l '^0$' "$OUT/par${PAR}"/slot*.rc 2>/dev/null | wc -l)
FAIL=$((NEP - OK))
THR=$(awk -v ok=$OK -v w=$WALL 'BEGIN{printf "%.0f", (w>0)?ok*3600/w:0}')
CPU=$(awk -v a=$C0 -v b=$C1 'BEGIN{printf "%.1f",(b-a)/1e6}')
CPU_PER_EP=$(awk -v c=$CPU -v ok=$OK 'BEGIN{printf "%.1f",(ok>0)?c/ok:0}')
# real CPU busy = 100 - mean %idle (last column of mpstat 'all' rows)
CPUBUSY=$(awk '/all/ && $NF ~ /^[0-9.]+$/ {idle+=$NF; n++} END{printf "%.1f",(n?100-idle/n:0)}' "$OUT/par${PAR}/mpstat.txt")
# real GPU engine activity = mean sm% over all gpu rows (skip header/warmup)
GPUSM=$(awk '!/^#/ && NF>=4 {sm+=$3; n++} END{printf "%.1f",(n?sm/n:0)}' "$OUT/par${PAR}/dmon.txt")
echo "SWEEP_ROW,par=$PAR,n_eps=$NEP,ok=$OK,fail=$FAIL,wall_s=$WALL,throughput_eph=$THR,cpu_busy_pct=$CPUBUSY,gpu_sm_pct=$GPUSM,cpu_core_s=$CPU,core_s_per_ep=$CPU_PER_EP" | tee -a "$OUT/sweep_results.csv"
echo "SWEEP_DONE par=$PAR"
