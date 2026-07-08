#!/usr/bin/env bash
# =============================================================================
# prof_mps.sh — start/stop an NVIDIA MPS control daemon scoped to chosen GPUs.
#
# MPS funnels every client process's kernels through ONE shared GPU context, so
# their kernels can actually run concurrently on the SMs instead of being
# time-sliced across per-process contexts. Zero VIO code change. This is the
# lead candidate for the soft bottleneck; use with prof_gpu_concurrency.sh /
# prof_par_sweep.sh for the A/B.
#
# Usage:
#   prof_mps.sh start <gpu_ids csv, e.g. 0 or 0,1>   # export the printed vars
#   prof_mps.sh stop
# The daemon + client must share CUDA_MPS_PIPE_DIRECTORY. This script prints the
# exports; the CALLER must eval/source them into the environment that launches
# the workers (tmux panes need them re-exported — see prof_par_sweep.sh).
# Run INSIDE the container.
# =============================================================================
set +u
CMD="${1:?usage: prof_mps.sh start <gpus>|stop}"
PIPE="${CUDA_MPS_PIPE_DIRECTORY:-/tmp/nvidia-mps}"
LOG="${CUDA_MPS_LOG_DIRECTORY:-/tmp/nvidia-mps-log}"

case "$CMD" in
  start)
    GPUS="${2:?need gpu ids csv}"
    mkdir -p "$PIPE" "$LOG"
    export CUDA_VISIBLE_DEVICES="$GPUS"
    export CUDA_MPS_PIPE_DIRECTORY="$PIPE"
    export CUDA_MPS_LOG_DIRECTORY="$LOG"
    # already running?
    if pgrep -x nvidia-cuda-mps-control >/dev/null 2>&1; then
      echo "MPS control already running" >&2
    else
      nvidia-cuda-mps-control -d
      sleep 1
    fi
    # probe it responds
    echo get_default_active_thread_percentage | nvidia-cuda-mps-control 2>&1 | head -1 >&2
    echo "export CUDA_MPS_PIPE_DIRECTORY=$PIPE"
    echo "export CUDA_MPS_LOG_DIRECTORY=$LOG"
    echo "# MPS started on GPUs $GPUS (pipe=$PIPE)" >&2
    ;;
  stop)
    if pgrep -x nvidia-cuda-mps-control >/dev/null 2>&1; then
      echo quit | CUDA_MPS_PIPE_DIRECTORY="$PIPE" nvidia-cuda-mps-control 2>/dev/null
      sleep 1
    fi
    pkill -x nvidia-cuda-mps-server 2>/dev/null
    pkill -x nvidia-cuda-mps-control 2>/dev/null
    rm -rf "$PIPE" "$LOG" 2>/dev/null
    echo "MPS stopped + pipe cleaned" >&2
    ;;
  *) echo "usage: prof_mps.sh start <gpus>|stop" >&2; exit 2;;
esac
