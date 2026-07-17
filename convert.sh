#!/usr/bin/env bash
# =============================================================================
# convert.sh <input_folder> <output_folder>
#
# Convert every episode under <input_folder> into a VIO product tree rooted at
# <output_folder>. The output MIRRORS the input subtree, e.g.
#   <input>/A_019/<hash>/episode.mcap
#     -> <output>/A_019/<hash>/episode_with_pose.mcap/   (VIO poses + tactile)
#        <output>/A_019/<hash>/<hash>.json               (task-annotation sidecar)
#
# Independent output tree; the input tree is left with only its original files
# (episode.mcap + sidecar json) — the transient build scratch is auto-removed
# whether the episode succeeds OR fails, so the source never keeps residue.
# Idempotent: re-run to resume (already-converted episodes are SKIPped).
#
# Runs INSIDE the tinynav container (needs ROS + the umi tool). Env knobs are
# forwarded to vio_batch_dispatch.sh: PAR, GPUS, DBASE, POSE_SENSORS, TIMEOUT,
# RETRY_TIMEOUT. Defaults: PAR=32, GPUS=all 8, pose=left_wrist,right_wrist.
#
# Example (host-side, detached):
#   docker cp convert.sh vio_batch_dispatch.sh tinynav_flatbuffer:/tmp/
#   tmux new-session -d -s conv "docker exec -e PAR=32 -e GPUS=0,1,2,3,4,5,6,7 \
#     tinynav_flatbuffer bash /tmp/convert.sh /data/my_raw /data/my_vio \
#     > ~/conv.log 2>&1; echo done > ~/conv.rc"
# =============================================================================
set -u
IN="${1:?usage: convert.sh <input_folder> <output_folder>}"
OUT="${2:?usage: convert.sh <input_folder> <output_folder>}"
IN="${IN%/}"; OUT="${OUT%/}"
EP_NAME="${EP_NAME:-episode.mcap}"     # raw episode filename to discover

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISPATCH="$SELF_DIR/vio_batch_dispatch.sh"
[ -f "$DISPATCH" ] || DISPATCH=/tmp/vio_batch_dispatch.sh   # container staging fallback
[ -f "$DISPATCH" ] || { echo "vio_batch_dispatch.sh not found next to convert.sh or at /tmp" >&2; exit 2; }
[ -d "$IN" ] || { echo "input folder not found: $IN" >&2; exit 2; }

# discover every episode under the input folder (any subtree depth)
LIST=$(mktemp)
find "$IN" -type f -name "$EP_NAME" 2>/dev/null | sort > "$LIST"
n=$(grep -c . "$LIST")
if [ "$n" -eq 0 ]; then echo "no $EP_NAME found under $IN" >&2; rm -f "$LIST"; exit 2; fi

mkdir -p "$OUT"
echo "CONVERT input=$IN output=$OUT episodes=$n par=${PAR:-32} gpus=${GPUS:-0-7}"
echo "  products -> $OUT/<rel>/episode_with_pose.mcap  (+ sidecar <hash>.json)"
echo "  logs     -> $OUT/_vio_logs/  (results.tsv, per-episode logs)"

# hand off to the proven dispatcher with input/output roots wired in
RAW_ROOT="$IN" VIO_ROOT="$OUT" bash "$DISPATCH" "$LIST"
rc=$?
rm -f "$LIST"
exit "$rc"
