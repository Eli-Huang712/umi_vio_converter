#!/usr/bin/env bash
# =============================================================================
# prof_node_mps.sh — full-node df PAR=64 steady throughput WITH MPS on all 8 GPUs,
# to convert the per-card MPS gain (+52% at N=8) into a node-level frames/hr number.
# Baseline (MPS off) is the earlier steady df PAR=64 = 1,612,520 frames/hr.
#
# Starts one MPS control daemon spanning GPUs 0-7 (one server per GPU, on demand),
# exports the pipe dir so every worker connects to it, runs the steady harness,
# then stops MPS. Runs INSIDE the container.
#
# Usage: prof_node_mps.sh <eps_dir> <PAR> <dur_s> [trim_s] [out]
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; PAR="${2:-64}"; DUR="${3:-360}"; TRIM="${4:-90}"
OUT="${5:-/data/p1_pool/node_mps}"
mkdir -p "$OUT"
MPS=/tmp/prof_mps.sh

echo "=== start MPS daemon on GPUs 0-7 ==="
eval "$(bash "$MPS" start 0,1,2,3,4,5,6,7 2>/dev/null)"
echo "CUDA_MPS_PIPE_DIRECTORY=$CUDA_MPS_PIPE_DIRECTORY"
# sanity: control daemon responds
echo get_default_active_thread_percentage | nvidia-cuda-mps-control 2>&1 | head -1

echo "=== run steady df PAR=$PAR WITH MPS ==="
# the steady harness inherits CUDA_MPS_PIPE_DIRECTORY -> workers connect to MPS
bash /tmp/prof_steady_frames.sh "$EPS" "$PAR" df "$DUR" 8 "$TRIM" "$OUT"

echo "=== stop MPS ==="
bash "$MPS" stop 2>&1 | tail -2
echo "NODE_MPS_DONE"
