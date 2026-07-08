#!/usr/bin/env bash
# =============================================================================
# prof_par_frames.sh — PAR throughput in FRAMES/hr (size-invariant), stock vs
# direct-feed. Runs INSIDE the container on an OTHERWISE-IDLE node.
#
# Why frames not episodes: UMI episodes vary ~4x in frame count (73-461/wrist),
# so ep/hr is confounded by the pool's size mix. Real work scales with FRAMES
# (per-frame perception dominates). Frame count per episode = left_h264/video
# message count summed over BOTH wrists (matches the report's ~456 frames/ep,
# 545K frames/hr @ PAR=64 baseline).
#
# Each worker = 1 episode, 2 wrists SEQUENTIAL, pinned gpu=slot%NGPU, unique
# ROS domain DBASE+slot. MODE=stock (default) or df (UMI_DIRECT_FEED=1).
# Throughput = SUM(frames of COMPLETED episodes) * 3600 / wall.
#
# Usage: prof_par_frames.sh <eps_dir> <PAR> <n_eps> <MODE> [ngpu] [dbase] [out]
# =============================================================================
set +u
EPS="${1:?usage: prof_par_frames.sh <eps_dir> <PAR> <n_eps> <stock|df> [ngpu] [dbase] [out]}"
PAR="${2:?need PAR}"
NEP="${3:?need n_eps}"
MODE="${4:?need MODE stock|df}"
NGPU="${5:-8}"
DBASE="${6:-40}"
OUT="${7:-/data/p1_pool/par_frames_out}"
CG=/sys/fs/cgroup
mkdir -p "$OUT/${MODE}_par${PAR}"

for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
ONE=/tinynav/tool/umi/_build_map_one_sensor.sh

mapfile -t EPLIST < <(find "$EPS" -name episode.mcap 2>/dev/null | sort | head -n "$NEP")
NEP=${#EPLIST[@]}
[ "$NEP" -ge 1 ] || { echo "no episodes"; exit 1; }
if [ $((DBASE + NEP)) -gt 232 ]; then echo "DBASE+NEP>232 (FastDDS cap)"; exit 1; fi
[ "$MODE" = "df" ] && DF_ENV="UMI_DIRECT_FEED=1" || DF_ENV=""

echo "=== MODE=$MODE PAR=$PAR n_eps=$NEP ngpu=$NGPU dbase=$DBASE ==="

# --- precompute frames per episode (both wrists' left-camera msg count) -------
FRAMES_CSV="$OUT/frames_manifest.csv"
if [ ! -s "$FRAMES_CSV" ]; then
  python3 - "$EPS" > "$FRAMES_CSV" <<'PY'
import sys, os, glob
from mcap.reader import make_reader
root=sys.argv[1]
L="coracam_lefthand/left_h264/video"; R="coracam_righthand/left_h264/video"
for ep in sorted(glob.glob(os.path.join(root,"*","episode.mcap"))):
    try:
        with open(ep,"rb") as f: s=make_reader(f).get_summary()
        tb={cid:ch.topic for cid,ch in s.channels.items()}
        cc=s.statistics.channel_message_counts if s.statistics else {}
        lw=sum(v for k,v in cc.items() if L in tb.get(k,""))
        rw=sum(v for k,v in cc.items() if R in tb.get(k,""))
        print(f"{ep},{lw+rw}")
    except Exception as e:
        print(f"# {ep}: {e}", file=sys.stderr)
PY
fi
frames_for() { awk -F, -v e="$1" '$1==e{print $2}' "$FRAMES_CSV"; }

# clean products so every ep is FULL
for ep in "${EPLIST[@]}"; do rm -rf "${ep%.mcap}"/*_db "${ep%.mcap}"/*_with_pose.mcap 2>/dev/null; done
read_cpu_usec() { awk '/usage_usec/{print $2}' "$CG/cpu.stat"; }

worker() {
  local slot="$1" ep="$2"
  local gpu=$((slot % NGPU)) dom=$((DBASE + slot)) rc=0
  for sensor in left_wrist right_wrist; do
    env $DF_ENV CUDA_VISIBLE_DEVICES="$gpu" UMI_PERCEPTION_WARMUP_SEC=1 UMI_BUILD_MAP_TIMEOUT_SEC=400 \
      bash "$ONE" "$ep" "$sensor" "$dom" 20 > "$OUT/${MODE}_par${PAR}/slot${slot}_${sensor}.log" 2>&1 || rc=1
  done
  echo "$rc" > "$OUT/${MODE}_par${PAR}/slot${slot}.rc"
  echo "$ep" > "$OUT/${MODE}_par${PAR}/slot${slot}.ep"
}
export -f worker; export ONE OUT PAR NGPU DBASE MODE DF_ENV

mpstat 1 > "$OUT/${MODE}_par${PAR}/mpstat.txt" 2>/dev/null & MPSTAT=$!
nvidia-smi dmon -s u -d 1 -o T > "$OUT/${MODE}_par${PAR}/dmon.txt" 2>/dev/null & DMON=$!
sleep 2

C0=$(read_cpu_usec); T0=$(date +%s.%N)
i=0
for ep in "${EPLIST[@]}"; do echo "$i|$ep"; i=$((i+1)); done | \
  xargs -d '\n' -P "$PAR" -I{} bash -c 'IFS="|" read -r slot ep <<< "{}"; worker "$slot" "$ep"'
T1=$(date +%s.%N); C1=$(read_cpu_usec)
kill "$MPSTAT" "$DMON" 2>/dev/null

WALL=$(awk -v a=$T0 -v b=$T1 'BEGIN{printf "%.1f",b-a}')
OK=0; FAIL=0; FRAMES=0
for rcf in "$OUT/${MODE}_par${PAR}"/slot*.rc; do
  slot_ep="${rcf%.rc}.ep"; ep=$(cat "$slot_ep" 2>/dev/null)
  if [ "$(cat "$rcf")" = "0" ]; then
    OK=$((OK+1)); fr=$(frames_for "$ep"); FRAMES=$((FRAMES + ${fr:-0}))
  else FAIL=$((FAIL+1)); fi
done
EPH=$(awk -v ok=$OK -v w=$WALL 'BEGIN{printf "%.0f",(w>0)?ok*3600/w:0}')
FPH=$(awk -v fr=$FRAMES -v w=$WALL 'BEGIN{printf "%.0f",(w>0)?fr*3600/w:0}')
CPU=$(awk -v a=$C0 -v b=$C1 'BEGIN{printf "%.1f",(b-a)/1e6}')
CPUBUSY=$(awk '/all/ && $NF ~ /^[0-9.]+$/ {idle+=$NF; n++} END{printf "%.1f",(n?100-idle/n:0)}' "$OUT/${MODE}_par${PAR}/mpstat.txt")
GPUSM=$(awk '!/^#/ && NF>=4 {sm+=$3; n++} END{printf "%.1f",(n?sm/n:0)}' "$OUT/${MODE}_par${PAR}/dmon.txt")
echo "PARFRAMES_ROW,mode=$MODE,par=$PAR,n_eps=$NEP,ok=$OK,fail=$FAIL,wall_s=$WALL,frames=$FRAMES,ep_per_hr=$EPH,frames_per_hr=$FPH,cpu_busy_pct=$CPUBUSY,gpu_sm_pct=$GPUSM,cpu_core_s=$CPU" | tee -a "$OUT/par_frames_results.csv"
echo "PARFRAMES_DONE mode=$MODE par=$PAR"
