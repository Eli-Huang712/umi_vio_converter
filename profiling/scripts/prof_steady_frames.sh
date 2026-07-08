#!/usr/bin/env bash
# =============================================================================
# prof_steady_frames.sh — STEADY-STATE frames/hr at fixed concurrency, size-
# invariant, drain- and hang-immune. Matches the report's steady-state method
# (trim ramp/drain) instead of end-to-end batch wall (which, with only ~2 waves
# over 128 eps, is dominated by the drain tail).
#
# A dispatcher keeps exactly PAR episodes in flight from a work queue; as each
# finishes it logs "<epoch_s> <frames> <rc>" and the next episode launches. We
# run for DURATION_S, then compute frames/hr from the cumulative-frames vs time
# slope over a TRIMMED window [TRIM_S, DURATION_S - TRIM_S] (steady state only).
#
# Each unit = 1 episode, 2 wrists SEQUENTIAL, pinned gpu=(launch_idx%NGPU),
# unique rotating ROS domain, per-wrist `timeout` watchdog (both stock & df) so
# a stuck DDS handshake can't wedge a slot. MODE=stock|df.
#
# Usage: prof_steady_frames.sh <eps_dir> <PAR> <MODE> <DURATION_S> [ngpu] [trim_s] [out]
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; PAR="${2:?need PAR}"; MODE="${3:?need stock|df}"
DURATION="${4:?need duration_s}"; NGPU="${5:-8}"; TRIM="${6:-120}"
OUT="${7:-/data/p1_pool/steady_out}/${MODE}_par${PAR}"
mkdir -p "$OUT"
CG=/sys/fs/cgroup

for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
ONE=/tinynav/tool/umi/_build_map_one_sensor.sh
[ "$MODE" = "df" ] && DF_ENV="UMI_DIRECT_FEED=1" || DF_ENV=""

mapfile -t EPLIST < <(find "$EPS" -name episode.mcap 2>/dev/null | sort)
NPOOL=${#EPLIST[@]}
[ "$NPOOL" -ge 1 ] || { echo "no episodes"; exit 1; }

# frames per episode (both wrists' left-camera msg count) — precomputed once
FRAMES_CSV="${OUT%/*}/frames_manifest.csv"
if [ ! -s "$FRAMES_CSV" ]; then
  python3 - "$EPS" > "$FRAMES_CSV" <<'PY'
import sys, os, glob
from mcap.reader import make_reader
root=sys.argv[1]; L="coracam_lefthand/left_h264/video"; R="coracam_righthand/left_h264/video"
for ep in sorted(glob.glob(os.path.join(root,"*","episode.mcap"))):
    try:
        with open(ep,"rb") as f: s=make_reader(f).get_summary()
        tb={cid:ch.topic for cid,ch in s.channels.items()}
        cc=s.statistics.channel_message_counts if s.statistics else {}
        lw=sum(v for k,v in cc.items() if L in tb.get(k,"")); rw=sum(v for k,v in cc.items() if R in tb.get(k,""))
        print(f"{ep},{lw+rw}")
    except Exception as e: print(f"# {ep}: {e}", file=sys.stderr)
PY
fi
frames_for() { awk -F, -v e="$1" '$1==e{print $2}' "$FRAMES_CSV"; }

COMPLETION_LOG="$OUT/completions.txt"; : > "$COMPLETION_LOG"
START=$(date +%s.%N)
# NB: format with %.3f — awk's default OFMT (%.6g) renders epoch+dur in scientific
# notation ("1.78e+09"), which then compares wrong. Full fixed-point avoids that.
END_LAUNCH=$(awk -v s=$START -v d=$DURATION 'BEGIN{printf "%.3f", s+d}')

# clean products for the whole pool so every launch is FULL
for ep in "${EPLIST[@]}"; do rm -rf "${ep%.mcap}"/*_db "${ep%.mcap}"/*_with_pose.mcap 2>/dev/null; done

run_unit() {  # slot ep   (slot in [0,PAR) -> unique domain among in-flight units)
  local slot="$1" ep="$2"
  local gpu=$((slot % NGPU)) dom=$((40 + slot)) rc=0
  for sensor in left_wrist right_wrist; do
    env $DF_ENV CUDA_VISIBLE_DEVICES="$gpu" UMI_PERCEPTION_WARMUP_SEC=1 UMI_BUILD_MAP_TIMEOUT_SEC=200 \
      timeout 220 bash "$ONE" "$ep" "$sensor" "$dom" 20 > "$OUT/slot${slot}_${sensor}.log" 2>&1
    local urc=$?
    [ "$urc" = 0 ] || rc=1
    # STOCK leak guard: the stock path detaches a tmux session and blocks on
    # `tmux wait-for`; if our `timeout` kills the outer script the panes keep
    # running and hold the GPU. The session name is deterministic, so on any
    # non-zero exit reap it explicitly. (df runs in-process; timeout suffices.)
    if [ "$urc" != 0 ] && [ "$MODE" = "stock" ]; then
      local base="umi_${ep##*/}_${sensor}"; base="${base//[^A-Za-z0-9_]/_}"
      tmux kill-session -t "${base}_d${dom}" 2>/dev/null || true
    fi
  done
  local fr; fr=$(frames_for "$ep"); [ "$rc" = 0 ] || fr=0
  echo "$(date +%s.%N) ${fr:-0} $rc" >> "$COMPLETION_LOG"
}

# background CPU/GPU samplers
mpstat 1 > "$OUT/mpstat.txt" 2>/dev/null & MPSTAT=$!
nvidia-smi dmon -s u -d 1 -o T > "$OUT/dmon.txt" 2>/dev/null & DMON=$!

echo "=== STEADY MODE=$MODE PAR=$PAR dur=${DURATION}s pool=$NPOOL trim=${TRIM}s ==="
launch=0
declare -A PID_SLOT=()          # pid -> slot held
declare -a FREE_SLOTS=()        # stack of free slots in [0,PAR)
for ((s=0; s<PAR; s++)); do FREE_SLOTS+=("$s"); done
# keep exactly PAR units in flight until the launch deadline, cycling the pool.
# Each in-flight unit holds a UNIQUE slot in [0,PAR) -> unique ROS domain (40+slot),
# so no two concurrent units collide on tmux session name / DDS domain.
while (( $(awk -v n=$(date +%s.%N) -v e=$END_LAUNCH 'BEGIN{print (n<e)?1:0}') )); do
  for pid in "${!PID_SLOT[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      FREE_SLOTS+=("${PID_SLOT[$pid]}"); unset PID_SLOT[$pid]
    fi
  done
  while [ "${#PID_SLOT[@]}" -lt "$PAR" ] && [ "${#FREE_SLOTS[@]}" -gt 0 ]; do
    last=$(( ${#FREE_SLOTS[@]} - 1 ))
    slot="${FREE_SLOTS[$last]}"; unset "FREE_SLOTS[$last]"; FREE_SLOTS=("${FREE_SLOTS[@]}")
    ep="${EPLIST[$((launch % NPOOL))]}"
    run_unit "$slot" "$ep" &
    PID_SLOT[$!]="$slot"; launch=$((launch+1))
  done
  sleep 1
done
# let in-flight units drain (they still log; steady window already excludes drain)
for pid in "${!PID_SLOT[@]}"; do wait "$pid" 2>/dev/null; done
kill "$MPSTAT" "$DMON" 2>/dev/null

# --- steady-state frames/hr over trimmed window ---
python3 - "$COMPLETION_LOG" "$START" "$DURATION" "$TRIM" "$MODE" "$PAR" <<'PY' | tee "$OUT/steady_result.txt"
import sys
log, start, dur, trim, mode, par = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), sys.argv[5], sys.argv[6]
rows=[]
for line in open(log):
    p=line.split()
    if len(p)>=3: rows.append((float(p[0])-start, int(p[1]), int(p[2])))
rows.sort()
lo, hi = trim, dur - trim
win=[(t,fr,rc) for (t,fr,rc) in rows if lo<=t<=hi]
frames=sum(fr for _,fr,_ in win)
ok=sum(1 for _,_,rc in win if rc==0); fail=sum(1 for _,_,rc in win if rc!=0)
span=hi-lo
fph=frames*3600/span if span>0 else 0
eph=ok*3600/span if span>0 else 0
print(f"STEADY_ROW,mode={mode},par={par},window_s={span:.0f},completed_ok={ok},completed_fail={fail},"
      f"frames_in_window={frames},frames_per_hr={fph:.0f},ep_per_hr={eph:.0f}")
PY

# steady-state utilization over the same trimmed window (approx: whole run minus edges)
CPUBUSY=$(awk '/all/ && $NF ~ /^[0-9.]+$/ {idle+=$NF; n++} END{printf "%.1f",(n?100-idle/n:0)}' "$OUT/mpstat.txt")
GPUSM=$(awk '!/^#/ && NF>=4 {sm+=$3; n++} END{printf "%.1f",(n?sm/n:0)}' "$OUT/dmon.txt")
echo "STEADY_UTIL,mode=$MODE,par=$PAR,cpu_busy_pct=$CPUBUSY,gpu_sm_pct=$GPUSM"
echo "STEADY_DONE mode=$MODE par=$PAR"
