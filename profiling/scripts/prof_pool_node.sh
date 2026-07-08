#!/usr/bin/env bash
# =============================================================================
# prof_pool_node.sh — full-node persistent-pool + MPS throughput (frames/hr).
# Combines Stage A (pipeline+MPS) with Stage B (persistent workers): N workers,
# each a persistent process that runs a shard of episodes (fresh nodes/episode,
# so no state leak — gated bit-identical), pinned round-robin to 8 GPUs under MPS.
#
# Throughput = total frames of all shard episodes / wall (whole-batch, not steady
# window — the pool has no per-episode spawn so ramp/drain is small). Reports
# 10 Hz GPU util (gpu0 sample) + node CPU busy + failures.
#
# Usage: prof_pool_node.sh <eps_dir> <n_workers> <n_eps> [mps on|off]
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; NW="${2:-48}"; NEP="${3:-128}"; MPS_MODE="${4:-on}"; STAGGER="${5:-0}"
for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
POOL=/tinynav/tool/umi/direct_feed_pool.py
MPS=/tmp/prof_mps.sh
OUT=/data/p1_pool/pool_node; mkdir -p "$OUT"

# frames manifest (both wrists' left-camera msg counts) reused from steady harness
FRAMES_CSV=/data/p1_pool/steady_out/frames_manifest.csv
[ -s "$FRAMES_CSV" ] || FRAMES_CSV="$OUT/frames_manifest.csv"
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

# clean products
for ep in $(find "$EPS" -name episode.mcap | sort | head -"$NEP"); do
  rm -rf "${ep%.mcap}"/*_db 2>/dev/null
done
docker_noop=1
pkill -f direct_feed 2>/dev/null; pkill -f nvidia-cuda-mps 2>/dev/null; sleep 2

if [ "$MPS_MODE" = "on" ]; then
  eval "$(bash "$MPS" start 0,1,2,3,4,5,6,7 2>/dev/null)"
  echo "MPS on pipe=$CUDA_MPS_PIPE_DIRECTORY"
fi

# 10Hz gpu0 util + node cpu sampler
nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits -lms 100 -i 0 > "$OUT/gpu10hz.txt" 2>/dev/null & GS=$!
mpstat 1 > "$OUT/mpstat.txt" 2>/dev/null & MS=$!

T0=$(date +%s.%N)
python3 "$POOL" run --eps_dir "$EPS" --sensors left_wrist,right_wrist \
  --n_workers "$NW" --ngpu 8 --n_eps "$NEP" --out "$OUT" --timeout_s 3600 \
  --stagger_s "$STAGGER" > "$OUT/orch.log" 2>&1
T1=$(date +%s.%N)
kill "$GS" "$MS" 2>/dev/null
[ "$MPS_MODE" = "on" ] && bash "$MPS" stop >/dev/null 2>&1

# tally: total frames of completed episodes, failures
python3 - "$OUT" "$FRAMES_CSV" "$T0" "$T1" "$NW" "$MPS_MODE" <<'PY'
import sys, os, glob
out, fcsv, t0, t1, nw, mps = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5], sys.argv[6]
frames = {}
for line in open(fcsv):
    if line.startswith("#") or "," not in line: continue
    p, n = line.rsplit(",", 1); frames[p] = int(n)
ok = fail = tot_frames = 0
# count WORKER_ITEM ok / FAIL across worker logs; frames credited per episode (both wrists share ep frame count -> credit half per wrist)
seen = {}
for wl in glob.glob(os.path.join(out, "worker_*.log")):
    for line in open(wl):
        if "WORKER_ITEM ok" in line:
            ok += 1
            # path is last token
            ep = line.strip().split()[-1] + "/episode.mcap"
            fr = frames.get(ep, 0)
            tot_frames += fr / 2.0   # each ep counted once per wrist
        elif "WORKER_ITEM FAIL" in line:
            fail += 1
wall = t1 - t0
fph = tot_frames * 3600 / wall if wall > 0 else 0
gutil = 0.0
g = os.path.join(out, "gpu10hz.txt")
if os.path.exists(g):
    vals = [float(x) for x in open(g) if x.strip().replace('.','',1).isdigit()]
    s = int(len(vals)*0.15)
    gutil = sum(vals[s:])/max(1,len(vals[s:]))
cpub = 0.0
m = os.path.join(out, "mpstat.txt")
if os.path.exists(m):
    idle=[];
    for line in open(m):
        if "all" in line:
            f=line.split()
            try: idle.append(float(f[-1]))
            except: pass
    if idle: cpub = 100 - sum(idle)/len(idle)
print(f"POOL_NODE_ROW,mps={mps},n_workers={nw},ok={ok},fail={fail},wall_s={wall:.1f},"
      f"frames={tot_frames:.0f},frames_per_hr={fph:.0f},gpu_util_10hz_pct={gutil:.1f},cpu_busy_pct={cpub:.1f}")
PY
echo "POOL_NODE_DONE"
