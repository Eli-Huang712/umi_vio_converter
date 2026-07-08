#!/usr/bin/env bash
# =============================================================================
# prof_par_frames_df_sweep.sh — run direct-feed frames/hr at several PAR points
# to find df's peak (df uses ~4.7 cores/proc, so its CPU-saturating PAR ~40,
# below stock's 64). Runs INSIDE the container. Cleans procs/tmux between runs.
#
# Usage: prof_par_frames_df_sweep.sh <eps_dir> <n_eps> [ngpu] [out] [PARS...]
# =============================================================================
set +u
EPS="${1:?need eps_dir}"; NEP="${2:?need n_eps}"; NGPU="${3:-8}"
OUT="${4:-/data/p1_pool/par_frames_out}"
shift 4 2>/dev/null
PARS=("$@"); [ ${#PARS[@]} -eq 0 ] && PARS=(32 48 64)

cleanup() {
  pkill -f perception_node.py 2>/dev/null; pkill -f build_map_node.py 2>/dev/null
  pkill -f direct_feed_build_map 2>/dev/null; tmux kill-server 2>/dev/null
  sleep 3
}

for PAR in "${PARS[@]}"; do
  cleanup
  echo ">>> df PAR=$PAR"
  bash /tmp/prof_par_frames.sh "$EPS" "$PAR" "$NEP" df "$NGPU" 40 "$OUT"
done
cleanup
echo "DF_SWEEP_DONE"
