#!/usr/bin/env bash
# =============================================================================
# batch_backfill.sh — 批量给已有 with_pose 产物补触觉（纯 CPU，不占 GPU）。
#
# 对某个根目录下所有 raw episode 并行跑 BACKFILL：把 raw 里的触觉按 log_time
# 插进对应的 *_with_pose.mcap。幂等（已补的自动 SKIP）、非破坏（保留 .pre_tactile.bak）、
# 可断点续跑（重跑时已完成的判 SKIP）。
#
# 用法（容器内）：
#   PAR=32 bash tool/umi/batch_backfill.sh <根目录> [episode.mcap 的相对 glob]
#
# 例：plant_collection（raw = <hash>/episode.mcap）
#   PAR=32 bash tool/umi/batch_backfill.sh \
#       /data/plant_collection/raw/26-06-23/dataloop-umi/184
#
# 例：pick_tiny_objects（raw = <hash>/<hash>.mcap）
#   PAR=32 RAW_GLOB='*/*.mcap' bash tool/umi/batch_backfill.sh \
#       /data/pick_tiny_objects/vio_todo
#
# 环境变量：
#   PAR        并行 worker 数（默认 32；补触觉是 CPU/IO 型，32 在 192 核上很轻松）
#   RAW_GLOB   相对根目录定位 raw 的 glob（默认 '*/episode.mcap'）
#   LOGDIR     日志目录（默认 <根目录>/../backfill_logs）
#   LIMIT      只处理前 N 个（默认 0=全部；用于先小跑试水）
# =============================================================================
set +u

BASE="${1:?用法: batch_backfill.sh <根目录> ; 见脚本头注释}"
RAW_GLOB="${2:-${RAW_GLOB:-*/episode.mcap}}"
PAR="${PAR:-32}"
LIMIT="${LIMIT:-0}"
SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="${LOGDIR:-${BASE%/}/../backfill_logs}"
mkdir -p "$LOGDIR"
RESULTS="$LOGDIR/results.tsv"; : > "$RESULTS"
: > "$LOGDIR/errors.log"

# 找到 python-mcap（rosbag2 已装的 venv）
PY="${PY:-/opt/venv/bin/python3}"
command -v "$PY" >/dev/null 2>&1 || PY=python3

# Source ROS overlays 一次；worker 通过 export 继承
for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
export AMENT_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH PATH ROS_VERSION ROS_DISTRO

# 单集 worker（内联，避免额外文件）
worker() {
  local RAW="$1" PY="$2" SELF_DIR="$3"
  # with_pose 路径：<raw 所在目录>/episode/episode_with_pose.mcap
  # （若你的布局不同，改这一行即可）
  local WP="$(dirname "$RAW")/episode/episode_with_pose.mcap"
  local H="$(basename "$(dirname "$RAW")")"
  local t0; t0=$(date +%s)
  if [ ! -f "$WP/metadata.yaml" ]; then echo -e "${H}\tNO_WP\t0\t0"; return; fi
  local out rc el
  out=$("$PY" "$SELF_DIR/backfill_tactile.py" --with-pose "$WP" --raw "$RAW" 2>&1); rc=$?
  el=$(( $(date +%s) - t0 ))
  if [ $rc -ne 0 ]; then echo -e "${H}\tFAIL\t${el}\t0"; echo "[$H] $out" | tail -3 >&2; return; fi
  if echo "$out" | grep -q "action=skip"; then echo -e "${H}\tSKIP\t${el}\t0"; return; fi
  local ins; ins=$(echo "$out" | grep -oE "inserted_tactile_messages=[0-9]+" | grep -oE "[0-9]+$")
  echo -e "${H}\tOK\t${el}\t${ins:-0}"
}
export -f worker

mapfile -t RAWS < <(cd "$BASE" && ls -1d $RAW_GLOB 2>/dev/null | sed "s#^#${BASE%/}/#" | sort)
[ "$LIMIT" -gt 0 ] && RAWS=("${RAWS[@]:0:$LIMIT}")
N=${#RAWS[@]}
echo "episodes=$N par=$PAR base=$BASE glob=$RAW_GLOB"
[ "$N" -eq 0 ] && { echo "没找到 raw（检查 RAW_GLOB）"; exit 1; }
G0=$(date +%s)

printf '%s\n' "${RAWS[@]}" | xargs -P "$PAR" -I {} bash -c 'worker "$1" "'"$PY"'" "'"$SELF_DIR"'"' _ {} \
  >> "$RESULTS" 2>>"$LOGDIR/errors.log"

WALL=$(( $(date +%s) - G0 ))
echo "=================================================="
echo "TOTAL_WALL_S=$WALL EPISODES=$N PAR=$PAR"
echo "--- 结果统计 ---"; awk -F'\t' '{c[$2]++} END{for(k in c) print k"="c[k]}' "$RESULTS"
echo "--- OK 耗时(s) ---"; awk -F'\t' '$2=="OK"{print $3}' "$RESULTS" | sort -n | \
  awk '{a[NR]=$1;s+=$1} END{if(NR)print "min="a[1]" median="a[int(NR/2)]" max="a[NR]" mean="s/NR" n="NR}'
echo "--- 触觉插入总数 ---"; awk -F'\t' '$2=="OK"{s+=$4} END{print "inserted_msgs_total="s+0}' "$RESULTS"
echo "日志: $RESULTS  |  错误: $LOGDIR/errors.log"
