#!/usr/bin/env bash
# =============================================================================
# prof_footprint.sh — 4A end-to-end single-episode footprint, N episodes x M
# runs, medians (VIO is non-deterministic so single points are unsafe).
#
# Extends profile_one.sh: per (episode,run) it brackets the whole 2-wrist FULL
# build with cgroup v2 cpu.stat (real CPU core-seconds C), io.stat (bytes),
# samples GPU sm/mem, tracks peak RSS, and keeps every perception/build log so a
# later pass can extract stage times (model load / first frame / steady per-frame
# / mapping "Grand total" / merge). Appends one CSV row per (episode,run,phase).
#
# Usage: prof_footprint.sh <scratch_dir> <gpu_id> <n_episodes> <n_runs> [out_dir]
#   <scratch_dir> must already contain N copied raw episode dirs (see prof_prep_scratch.sh)
# Env: UMI_PERCEPTION_WARMUP_SEC (default 1), UMI_BUILD_MAP_TIMEOUT_SEC (default 300)
# Run INSIDE the tinynav container on an OTHERWISE-IDLE target GPU.
# =============================================================================
set +u
SCRATCH="${1:?usage: prof_footprint.sh <scratch_dir> <gpu_id> <n_episodes> <n_runs> [out_dir]}"
GPU="${2:?need gpu_id}"
NEP="${3:-3}"
NRUN="${4:-3}"
OUT="${5:-/data/prof_scratch/footprint_out}"
WARMUP="${UMI_PERCEPTION_WARMUP_SEC:-1}"
CG=/sys/fs/cgroup
mkdir -p "$OUT"
CSV="$OUT/footprint.csv"

for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
ONE=/tinynav/tool/umi/_build_map_one_sensor.sh
MERGE=/tinynav/tool/umi/merge_sqlite_mcap.py

read_cpu_usec() { awk '/usage_usec/{print $2}' "$CG/cpu.stat"; }
read_io_bytes() { awk -F'[ =]' '{for(i=1;i<=NF;i++){if($i=="rbytes")r+=$(i+1); if($i=="wbytes")w+=$(i+1)}} END{print r+0, w+0}' "$CG/io.stat" 2>/dev/null; }

# pick a SIZE SPREAD of NEP episodes (C scales with frame count / size), so the
# medians represent small/median/large, not an alphabetical accident.
mapfile -t ALL < <(find "$SCRATCH" -maxdepth 3 -name '*.mcap' ! -name '*_with_pose*' 2>/dev/null \
                    | xargs -I{} stat -c '%s {}' {} 2>/dev/null | sort -n | awk '{print $2}')
TOT=${#ALL[@]}
EPS=()
if [ "$TOT" -ge "$NEP" ] && [ "$NEP" -ge 1 ]; then
  for k in $(seq 0 $((NEP-1))); do
    idx=$(( k * (TOT-1) / (NEP>1?NEP-1:1) ))   # spread across the size-sorted list
    EPS+=("${ALL[$idx]}")
  done
else
  EPS=("${ALL[@]}")
fi
echo "found $TOT episodes in $SCRATCH; picked ${#EPS[@]} across size spread:"
for e in "${EPS[@]}"; do echo "  $(stat -c '%s' "$e") bytes  $e"; done
[ "${#EPS[@]}" -ge 1 ] || { echo "no episodes; run prof_prep_scratch.sh first"; exit 1; }

[ -f "$CSV" ] || echo "episode,run,phase,wall_s,cpu_core_s,avg_cores,io_read_mb,io_write_mb,gpu_busy_s,gpu_util_peak,gpu_mem_peak_mib,rss_peak_mib,merge_s" > "$CSV"

sample_gpu() { # bg: ts sm mem_used every 0.5s -> $1
  while :; do
    ts=$(date +%s.%N)
    line=$(nvidia-smi --id="$GPU" --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits 2>/dev/null | tr -d ' ')
    echo "$ts $line"; sleep 0.5
  done > "$1" 2>/dev/null
}
sample_rss() { # bg: peak summed RSS (MiB) of perception+build across time -> $1
  # NOTE: match against full args (ps comm truncates to 'python3').
  local mx=0
  while :; do
    cur=$(ps -eo rss,args 2>/dev/null | awk '/perception_node\.py|build_map_node\.py/{s+=$1} END{print int(s/1024)}')
    [ "${cur:-0}" -gt "$mx" ] && mx=$cur
    echo "$mx" > "$1"; sleep 0.5
  done
}

emit() { # episode run phase wall cpu io_r io_w gbs gutil gmem rss merge
  local ep="$1" run="$2" phase="$3" wall="$4" cpu="$5" ior="$6" iow="$7" gbs="$8" gutil="$9" gmem="${10}" rss="${11}" merge="${12}"
  awk -v ep="$ep" -v r="$run" -v ph="$phase" -v w="$wall" -v c="$cpu" -v ir="$ior" -v iw="$iow" \
      -v gbs="$gbs" -v gu="$gutil" -v gm="$gmem" -v rss="$rss" -v mg="$merge" \
    'BEGIN{ac=(w>0)?c/w:0; printf "%s,%s,%s,%.1f,%.1f,%.2f,%.0f,%.0f,%s,%s,%s,%s,%s\n", ep,r,ph,w,c,ac,ir/1048576,iw/1048576,gbs,gu,gm,rss,mg}' >> "$CSV"
}

for ep in "${EPS[@]}"; do
  ename=$(basename "$(dirname "$ep")")/$(basename "$ep")
  for run in $(seq 1 "$NRUN"); do
    tag="${ename//\//_}_r${run}"
    logd="$OUT/logs/$tag"; mkdir -p "$logd"
    # clean prior artifacts so no SKIP / stale poses
    rm -rf "${ep%.mcap}"/*_db "${ep%.mcap}"/*_with_pose.mcap 2>/dev/null

    sample_gpu "$logd/gpu.txt" &  GS=$!
    sample_rss "$logd/rss.txt" &  RS=$!

    C0=$(read_cpu_usec); read IOR0 IOW0 < <(read_io_bytes); T0=$(date +%s.%N)
    for sensor in left_wrist right_wrist; do
      CUDA_VISIBLE_DEVICES="$GPU" UMI_PERCEPTION_WARMUP_SEC="$WARMUP" \
        bash "$ONE" "$ep" "$sensor" 140 20 > "$logd/${sensor}.log" 2>&1
    done
    T1=$(date +%s.%N)
    # merge (tactile) — small, measured separately
    Tm0=$(date +%s.%N)
    python3 "$MERGE" --input_mcap "$ep" --pose-sensors "left_wrist,right_wrist" > "$logd/merge.log" 2>&1 || echo "merge_rc=$?" >> "$logd/merge.log"
    Tm1=$(date +%s.%N)
    C1=$(read_cpu_usec); read IOR1 IOW1 < <(read_io_bytes)
    kill "$GS" "$RS" 2>/dev/null

    WALL=$(awk -v a=$T0 -v b=$T1 'BEGIN{printf "%.1f",b-a}')
    MWALL=$(awk -v a=$Tm0 -v b=$Tm1 'BEGIN{printf "%.1f",b-a}')
    CPU=$(awk -v a=$C0 -v b=$C1 'BEGIN{printf "%.1f",(b-a)/1e6}')
    read -a GF < <(awk 'NF>=2{split($2,a,","); s+=a[1]/100*0.5; if(a[1]+0>mu)mu=a[1]; if(a[2]+0>mm)mm=a[2]} END{printf "%.1f %s %s", s,mu+0,mm+0}' "$logd/gpu.txt")
    RSS=$(cat "$logd/rss.txt" 2>/dev/null); RSS=${RSS:-0}   # already MiB from sampler
    emit "$ename" "$run" "build2wrist" "$WALL" "$CPU" "$((IOR1-IOR0))" "$((IOW1-IOW0))" "${GF[0]}" "${GF[1]}" "${GF[2]}" "$RSS" "$MWALL"
    echo "[$tag] wall=${WALL}s merge=${MWALL}s C=${CPU} core-s gpu_busy=${GF[0]}s util_peak=${GF[1]}% mem_peak=${GF[2]}MiB rss=${RSS}MiB"
  done
done
echo "FOOTPRINT_DONE csv=$CSV"
