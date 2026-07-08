#!/usr/bin/env bash
# =============================================================================
# prof_util_sweep.sh — Stage A3: find the PAR that maximizes EFFECTIVE util
# (CPU & GPU both high) under pipeline-decode (+optional MPS), 0 failures.
#
# Pipeline is the driver default (UMI_PIPELINE=1). This sweeps PAR and, with MPS,
# records steady frames/hr + CPU% + GPU sm% per PAR so we can pick the "fat worker"
# sweet spot (expected well below PAR=64) where both resources are >80% and no
# episode fails. Runs INSIDE the container on an idle 8-GPU node.
#
# Usage: prof_util_sweep.sh <eps_dir> <mps on|off> <dur_s> <PARS...>
#   e.g. prof_util_sweep.sh /data/p1_pool/raw on 300 16 24 32 48
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; MPS_MODE="${2:-on}"; DUR="${3:-300}"; shift 3 2>/dev/null
PARS=("$@"); [ ${#PARS[@]} -eq 0 ] && PARS=(16 24 32 48)
OUT=/data/p1_pool/util_sweep; mkdir -p "$OUT"
MPS=/tmp/prof_mps.sh

cleanup_procs() {
  docker_noop=1
  pkill -f perception_node 2>/dev/null; pkill -f build_map_node 2>/dev/null
  pkill -f direct_feed 2>/dev/null; tmux kill-server 2>/dev/null; sleep 3
}

if [ "$MPS_MODE" = "on" ]; then
  eval "$(bash "$MPS" start 0,1,2,3,4,5,6,7 2>/dev/null)"
  echo "MPS on, pipe=$CUDA_MPS_PIPE_DIRECTORY"
fi

for PAR in "${PARS[@]}"; do
  cleanup_procs
  echo ">>> pipeline+MPS=$MPS_MODE PAR=$PAR"
  # prof_steady_frames inherits CUDA_MPS_PIPE_DIRECTORY (workers connect to MPS);
  # pipeline is on by default in the driver.
  bash /tmp/prof_steady_frames.sh "$EPS" "$PAR" df "$DUR" 8 90 "$OUT/mps_${MPS_MODE}"
done
cleanup_procs

if [ "$MPS_MODE" = "on" ]; then
  bash "$MPS" stop 2>&1 | tail -1
fi
echo "=== SUMMARY (mps=$MPS_MODE, pipeline=on) ==="
grep -hE "STEADY_ROW|STEADY_UTIL" "$OUT/mps_${MPS_MODE}"/*/steady_result.txt 2>/dev/null || true
echo "UTIL_SWEEP_DONE mps=$MPS_MODE"
