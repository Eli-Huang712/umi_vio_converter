#!/usr/bin/env bash
# =============================================================================
# prof_run_gpu_experiments.sh — orchestrate the single-GPU experiments that
# together settle the GPU-context-serialization hypothesis. All run on ONE idle
# GPU with the CPU nearly free (N<=8 procs use <=~25/192 cores), so any per-proc
# latency inflation is provably GPU-side, not CPU contention.
#
#   (A) concurrency sweep  N = 1,2,4,6,8   (chart 5 + clean GPU-seconds G at N=1)
#   (B) MPS A/B            N = 8, off vs on (chart 7)
#
# Usage: prof_run_gpu_experiments.sh <eps_dir> <gpu> [out]
# Run INSIDE the recreated (SYS_PTRACE) container.
# =============================================================================
set +u
EPS="${1:?usage: prof_run_gpu_experiments.sh <eps_dir> <gpu> [out]}"
GPU="${2:-0}"
OUT="${3:-/data/prof_scratch/conc_out}"
SCR=/data/prof_scratch/scripts
mkdir -p "$OUT"

echo "############ (A) concurrency sweep on GPU $GPU (MPS off) ############"
for N in 1 2 4 6 8; do
  # distinct domain base per N to avoid any residual-domain overlap between runs
  DB=$((100 + N*10))
  bash "$SCR/prof_gpu_concurrency.sh" "$EPS" "$GPU" "$N" "$DB" "$OUT"
done

echo "############ (B) MPS A/B at N=8 on GPU $GPU ############"
# MPS ON: start daemon scoped to this GPU, re-export its env for the workers
eval "$(bash "$SCR/prof_mps.sh" start "$GPU" 2>/dev/null)"
echo "MPS env: CUDA_MPS_PIPE_DIRECTORY=$CUDA_MPS_PIPE_DIRECTORY"
# domain base 200 -> domains 200..207 for N=8 (<=232 FastDDS cap). NB: the MPS-on
# run must NOT reuse the MPS-off domains while those are still around; the off run
# has fully exited by here, and 200-207 is disjoint from the sweep's 140-187.
CUDA_MPS_PIPE_DIRECTORY="$CUDA_MPS_PIPE_DIRECTORY" CUDA_MPS_LOG_DIRECTORY="$CUDA_MPS_LOG_DIRECTORY" \
  bash "$SCR/prof_gpu_concurrency.sh" "$EPS" "$GPU" 8 200 "$OUT"
bash "$SCR/prof_mps.sh" stop
echo "GPU_EXPERIMENTS_DONE"
