#!/usr/bin/env bash
# =============================================================================
# prof_pool_gate.sh — Stage B correctness gate: does reusing a PROCESS across
# episodes leak native state (the historical failure)? Runs INSIDE the container.
#
# (1) Run 3 episodes (one sensor) SEQUENTIALLY in ONE persistent worker.
# (2) Run the SAME 3 episodes each in its OWN fresh process (the safe reference).
# (3) Compare per-episode poses with EXACT numpy equality. direct-feed is
#     deterministic, so pooled==fresh MUST hold for every episode. Any DIFFER
#     (esp. growing keyframe drift) == native leak recurs == pool unsafe -> disable.
#
# Usage: prof_pool_gate.sh <eps_dir> [n_eps] [sensor]
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; NEP="${2:-3}"; SENSOR="${3:-right_wrist}"
for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
POOL=/tinynav/tool/umi/direct_feed_pool.py
DRIVER=/tinynav/tool/umi/direct_feed_build_map.py
OUT=/data/p1_pool/pool_gate; mkdir -p "$OUT"

mapfile -t EPLIST < <(find "$EPS" -name episode.mcap 2>/dev/null | sort | head -"$NEP")
echo "pool gate: ${#EPLIST[@]} episodes, sensor=$SENSOR (GPU 0)"

# (1) pooled: one persistent worker, 3 episodes sequential
: > "$OUT/shard.txt"
for ep in "${EPLIST[@]}"; do
  h=$(basename "$(dirname "$ep")")
  echo "$ep|$SENSOR|$OUT/${h}_pool_db" >> "$OUT/shard.txt"
done
echo "--- running persistent worker (3 eps sequential) ---"
CUDA_VISIBLE_DEVICES=0 python3 "$POOL" worker --shard "$OUT/shard.txt" > "$OUT/worker.log" 2>&1
grep -E "WORKER_ITEM|WORKER_DONE" "$OUT/worker.log"

# (2) fresh: each episode its own process
echo "--- running fresh processes (reference) ---"
for ep in "${EPLIST[@]}"; do
  h=$(basename "$(dirname "$ep")")
  CUDA_VISIBLE_DEVICES=0 python3 "$DRIVER" --bag_file "$ep" --sensor "$SENSOR" \
    --map_save_path "$OUT/${h}_fresh_db" --no_verbose_timer > "$OUT/${h}_fresh.log" 2>&1
done

# (3) compare per-episode, exact
echo "=== POOL vs FRESH (want all IDENTICAL) ==="
for ep in "${EPLIST[@]}"; do
  h=$(basename "$(dirname "$ep")")
  python3 - "$OUT/${h}_pool_db/poses.npy" "$OUT/${h}_fresh_db/poses.npy" "${h:0:8}" <<'PY'
import sys, numpy as np
try:
    a = np.load(sys.argv[1], allow_pickle=True).item()
    b = np.load(sys.argv[2], allow_pickle=True).item()
except Exception as e:
    print(f"[{sys.argv[3]}] LOAD_FAIL {e}"); sys.exit(0)
tag = sys.argv[3]
if set(a) != set(b):
    print(f"[{tag}] KEY_MISMATCH pool_kf={len(a)} fresh_kf={len(b)}  <-- LEAK"); sys.exit(0)
eq = all(np.array_equal(a[k], b[k]) for k in a)
maxd = max((float(np.max(np.abs(a[k]-b[k]))) for k in a), default=0.0)
print(f"[{tag}] kf={len(a)} {'IDENTICAL' if eq else 'DIFFER (LEAK)'} maxabs={maxd:.2e}")
PY
done
echo "POOL_GATE_DONE"
