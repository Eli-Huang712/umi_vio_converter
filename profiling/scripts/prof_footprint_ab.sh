#!/usr/bin/env bash
# =============================================================================
# prof_footprint_ab.sh — per-episode CPU core-seconds (C), stock vs direct-feed.
#
# The report frames the single-node CPU roofline as 192*3600/C ep/hr, where C =
# CPU core-seconds per 2-wrist episode (cgroup v2 cpu.stat usage_usec delta), and
# measured C_stock ~= 235. This runs ONE episode (2 wrists sequential) under stock
# and under direct-feed on an OTHERWISE-IDLE container, brackets each with cpu.stat,
# and reports C + wall + avg cores + implied roofline for both. The C_stock/C_df
# ratio is the roofline shift (headroom P1 buys on a single node).
#
# Also grabs per-thread peak %CPU (pidstat -t) for each path: stock build_map was
# GIL-pinned near 1 core (top/sum=0.92); this shows what df does instead.
#
# Usage: prof_footprint_ab.sh <episode.mcap> <gpu> [out]
# Run INSIDE the container, nothing else on the node (cpu.stat is container-wide).
# =============================================================================
set +u
EP="${1:?usage: prof_footprint_ab.sh <episode.mcap> <gpu> [out]}"
GPU="${2:?need gpu}"
OUT="${3:-/data/p1_scratch/footprint_ab}"
CG=/sys/fs/cgroup
mkdir -p "$OUT"

for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
ONE=/tinynav/tool/umi/_build_map_one_sensor.sh
read_cpu_usec() { awk '/usage_usec/{print $2}' "$CG/cpu.stat"; }

measure() {  # mode domain_base
  local mode="$1" dbase="$2" env=""
  [ "$mode" = "df" ] && env="UMI_DIRECT_FEED=1"
  # clean products so both start from FULL
  rm -rf "${EP%.mcap}"/*_db "${EP%.mcap}"/*_with_pose.mcap 2>/dev/null
  local c0 c1 t0 t1 d=$dbase
  c0=$(read_cpu_usec); t0=$(date +%s.%N)
  for sensor in left_wrist right_wrist; do
    env $env CUDA_VISIBLE_DEVICES="$GPU" UMI_PERCEPTION_WARMUP_SEC=1 \
      bash "$ONE" "$EP" "$sensor" "$d" 20 > "$OUT/${mode}_${sensor}.log" 2>&1
    d=$((d+1))
  done
  t1=$(date +%s.%N); c1=$(read_cpu_usec)
  local wall c cores roof
  wall=$(awk -v a=$t0 -v b=$t1 'BEGIN{printf "%.1f",b-a}')
  c=$(awk -v a=$c0 -v b=$c1 'BEGIN{printf "%.1f",(b-a)/1e6}')
  cores=$(awk -v c=$c -v w=$wall 'BEGIN{printf "%.2f",(w>0)?c/w:0}')
  roof=$(awk -v c=$c 'BEGIN{printf "%.0f",(c>0)?192*3600/c:0}')
  echo "FOOTPRINT_ROW,mode=$mode,wall_s=$wall,cpu_core_s=$c,avg_cores=$cores,cpu_roofline_eph=$roof" | tee -a "$OUT/footprint_ab.csv"
}

echo "=== footprint A/B on $(basename $(dirname $EP)) gpu $GPU ==="
measure stock 210
measure df 220
echo "FOOTPRINT_AB_DONE"
