#!/usr/bin/env bash
# Continuous-queue FULL VIO dispatcher (runs INSIDE the tinynav container).
# Keeps exactly PAR episodes in flight; each in-flight worker holds a unique
# slot in [0,PAR) -> unique ROS_DOMAIN_ID (DBASE+slot) and a round-robin GPU
# (slot%NGPU). Uses the direct-feed + pipeline optimized path. Per-episode
# timeout watchdog. Idempotent: --mode auto SKIPs already-converted episodes.
#
# Usage (inside container):
#   PAR=48 DBASE=100 bash /tmp/vio_batch_dispatch.sh /tmp/eplist.txt
set -u
LIST="${1:?usage: vio_batch_dispatch.sh <episode-list-file>}"
PAR="${PAR:-48}"
DBASE="${DBASE:-100}"
# GPUS = comma list of physical GPU indices to use (default all 8). Workers are
# round-robined ONLY over these, so we never touch a co-tenant's cards.
GPUS="${GPUS:-0,1,2,3,4,5,6,7}"
IFS=',' read -ra GPU_ARR <<< "$GPUS"
NGPU=${#GPU_ARR[@]}
TIMEOUT="${TIMEOUT:-900}"          # per-episode wall cap (2 wrists + merge)
POSE_SENSORS="${POSE_SENSORS:-left_wrist,right_wrist}"   # COMMA-separated; head excluded by default
# Separate output tree (products NOT nested in the raw tree). Per episode:
#   raw:     $RAW_ROOT/<subset>/<hash>/episode.mcap
#   product: $VIO_ROOT/<subset>/<hash>/episode_with_pose.mcap
# and the raw-tree build scratch ($RAW_ROOT/<subset>/<hash>/episode/) is removed
# after a successful convert so the raw tree stays pristine.
RAW_ROOT="${RAW_ROOT:-/data/umi_raw_data_260714}"
VIO_ROOT="${VIO_ROOT:-/data/umi_vio_data_260714}"
LOGDIR="${LOGDIR:-$VIO_ROOT/_vio_logs}"
mkdir -p "$LOGDIR"
RESULTS="$LOGDIR/results.tsv"
: > "$RESULTS"

# DDS domain ceiling guard (FastDDS: domainId <= 232)
if [ $((DBASE + PAR)) -gt 232 ]; then
  echo "ERROR: DBASE($DBASE)+PAR($PAR) > 232 (FastDDS domain cap)"; exit 2
fi

run_one() {
  local ep="$1" slot="$2"
  local dom=$((DBASE + slot)) gpu=${GPU_ARR[$((slot % NGPU))]}
  local tag; tag=$(echo "$ep" | md5sum | cut -c1-12)
  # product path in the separate VIO tree (mirror the raw subtree)
  local rel="${ep#$RAW_ROOT/}"; local reldir; reldir=$(dirname "$rel")
  local out="$VIO_ROOT/$reldir/episode_with_pose.mcap"
  local scratch="${ep%.mcap}"          # raw-tree build scratch dir
  mkdir -p "$(dirname "$out")"
  local t0; t0=$(date +%s)
  UMI_DIRECT_FEED=1 UMI_PIPELINE=1 CUDA_VISIBLE_DEVICES="$gpu" ROS_DOMAIN_ID="$dom" \
    timeout "$TIMEOUT" bash /tinynav/tool/umi/umi_vio_converter.sh "$ep" \
      --mode auto --pose-sensors "$POSE_SENSORS" --output "$out" \
      > "$LOGDIR/$tag.log" 2>&1
  local rc=$?
  if [ "$rc" -eq 0 ]; then
    # carry the raw sidecar <hash>.json next to the product (task annotation).
    # dirname ep = $RAW_ROOT/<subset>/<hash>; dirname out = $VIO_ROOT/<subset>/<hash>
    local sj; sj=$(ls "$(dirname "$ep")"/*.json 2>/dev/null | head -1)
    [ -n "$sj" ] && cp -p "$sj" "$(dirname "$out")/" 2>/dev/null
    # remove the raw-tree scratch (guarded: must be <RAW_ROOT>/*/episode)
    case "$scratch" in
      "$RAW_ROOT"/*/episode) [ -d "$scratch" ] && rm -rf "$scratch" ;;
    esac
  fi
  local t1; t1=$(date +%s)
  printf '%s\t%d\t%d\t%s\n' "$(date +%s)" "$rc" "$((t1-t0))" "$ep" >> "$RESULTS"
}
export -f run_one
export DBASE NGPU TIMEOUT POSE_SENSORS LOGDIR RESULTS GPUS RAW_ROOT VIO_ROOT

# slot semaphore via FIFO: prime with PAR unique slot tokens
FIFO=$(mktemp -u); mkfifo "$FIFO"; exec 9<>"$FIFO"; rm -f "$FIFO"
for s in $(seq 0 $((PAR-1))); do echo "$s" >&9; done

echo "DISPATCH_START $(date +%F' '%T) PAR=$PAR DBASE=$DBASE TIMEOUT=$TIMEOUT eps=$(wc -l < "$LIST")"
GSTART=$(date +%s)
while IFS= read -r ep; do
  [ -z "$ep" ] && continue
  read -r slot <&9              # block until a slot frees
  { run_one "$ep" "$slot"; echo "$slot" >&9; } &
done < "$LIST"
wait

# --- retry round: re-run rc!=0 episodes once with a longer timeout ---
RETRY_TIMEOUT="${RETRY_TIMEOUT:-1800}"
FAILED=$(awk -F'\t' '$2!=0{print $4}' "$RESULTS" | sort -u)
if [ -n "$FAILED" ]; then
  nfail=$(echo "$FAILED" | grep -c .)
  echo "RETRY_ROUND start failed=$nfail timeout=$RETRY_TIMEOUT"
  : > "$RESULTS.retry"
  TIMEOUT="$RETRY_TIMEOUT"
  while IFS= read -r ep; do
    [ -z "$ep" ] && continue
    read -r slot <&9
    { RESULTS="$RESULTS.retry" run_one "$ep" "$slot"; echo "$slot" >&9; } &
  done <<< "$FAILED"
  wait
  # merge: an episode that passed on retry flips to ok
  rok=$(awk -F'\t' '$2==0' "$RESULTS.retry" | wc -l)
  rfail=$(awk -F'\t' '$2!=0' "$RESULTS.retry" | wc -l)
  echo "RETRY_ROUND done retry_ok=$rok retry_still_fail=$rfail"
fi
GEND=$(date +%s)

# final tally: an ep is OK if it succeeded in either the main pass or the retry
ok=$( { awk -F'\t' '$2==0{print $4}' "$RESULTS"; awk -F'\t' '$2==0{print $4}' "$RESULTS.retry" 2>/dev/null; } | sort -u | wc -l)
alleps=$(grep -c . "$LIST")
fail=$((alleps - ok))
n=$alleps; wall=$((GEND-GSTART))
echo "DISPATCH_DONE wall=${wall}s ok=$ok fail=$fail total=$n"
awk -v w="$wall" -v ok="$ok" 'BEGIN{ if(w>0) printf "THROUGHPUT_RAW=%.0f ep/hr (ok/wall)\n", ok*3600.0/w }'
# trimmed steady-state: drop first & last PAR completions, rate over the middle
sort -n "$RESULTS" | awk -F'\t' -v par="$PAR" '
  $2==0{ts[++c]=$1}
  END{ if(c>2*par){ a=ts[par+1]; b=ts[c-par]; if(b>a) printf "THROUGHPUT_TRIMMED=%.0f ep/hr (steady, %d eps over %ds)\n",(c-2*par)*3600.0/(b-a),c-2*par,b-a } else print "THROUGHPUT_TRIMMED=n/a (need >2*PAR ok completions)" }'
