#!/usr/bin/env bash
# =============================================================================
# prof_pipeline_gate.sh — A1 correctness gate: pipelined decode vs batch decode
# must be BIT-IDENTICAL (direct-feed is deterministic; only the decode SCHEDULE
# differs, so poses.npy must match exactly). Runs INSIDE the container.
#
# For each (episode, sensor): run driver with pipeline (default) and with
# --no-pipeline, save both poses.npy, compare with exact numpy equality.
#
# Usage: prof_pipeline_gate.sh <eps_dir> [n_eps]
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; NEP="${2:-3}"
for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}
DRIVER=/tinynav/tool/umi/direct_feed_build_map.py
OUT=/data/p1_pool/pipeline_gate; mkdir -p "$OUT"

mapfile -t EPLIST < <(find "$EPS" -name episode.mcap 2>/dev/null | sort | head -"$NEP")
echo "gate over ${#EPLIST[@]} episodes x 2 sensors"

gpu=0
pids=()
for ep in "${EPLIST[@]}"; do
  h=$(basename "$(dirname "$ep")")
  for sensor in left_wrist right_wrist; do
    (
      g=$gpu
      CUDA_VISIBLE_DEVICES=$g python3 "$DRIVER" --bag_file "$ep" --sensor "$sensor" \
        --map_save_path "$OUT/${h}_${sensor}_pipe_db" --no_verbose_timer > "$OUT/${h}_${sensor}_pipe.log" 2>&1
      CUDA_VISIBLE_DEVICES=$g python3 "$DRIVER" --bag_file "$ep" --sensor "$sensor" --no-pipeline \
        --map_save_path "$OUT/${h}_${sensor}_batch_db" --no_verbose_timer > "$OUT/${h}_${sensor}_batch.log" 2>&1
      python3 - "$OUT/${h}_${sensor}_pipe_db/poses.npy" "$OUT/${h}_${sensor}_batch_db/poses.npy" "${h:0:8}/${sensor}" <<'PY'
import sys, numpy as np
pipe = np.load(sys.argv[1], allow_pickle=True).item()
batch = np.load(sys.argv[2], allow_pickle=True).item()
tag = sys.argv[3]
if set(pipe) != set(batch):
    print(f"[{tag}] KEY_MISMATCH pipe_kf={len(pipe)} batch_kf={len(batch)}"); sys.exit(0)
alleq = all(np.array_equal(pipe[k], batch[k]) for k in pipe)
maxd = max((float(np.max(np.abs(pipe[k]-batch[k]))) for k in pipe), default=0.0)
print(f"[{tag}] kf={len(pipe)} {'IDENTICAL' if alleq else 'DIFFER'} maxabs={maxd:.2e}")
PY
    ) &
    pids+=($!)
    gpu=$(( (gpu + 1) % 8 ))
  done
done
for p in "${pids[@]}"; do wait "$p"; done
echo "PIPELINE_GATE_DONE"
