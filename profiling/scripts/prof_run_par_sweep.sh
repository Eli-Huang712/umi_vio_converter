#!/usr/bin/env bash
# =============================================================================
# prof_run_par_sweep.sh — full-node saturation sweep across the 8 idle GPUs.
# Re-measures PAR 32/64 (real-utilization, cross-node check) and ADDS the
# missing 80/96 points the prior report lacked. Episode count scales with PAR
# so each point keeps >=2x oversubscription (ramp bias small) at ~8-10 min.
#
# Domain safety: DBASE=40, so DBASE+PAR<=136 <=232 (FastDDS cap) for PAR<=96.
# Usage: prof_run_par_sweep.sh <eps_dir> [out]
# Run INSIDE the container on the OTHERWISE-IDLE node (all 8 GPUs free).
# =============================================================================
set +u
EPS="${1:?usage: prof_run_par_sweep.sh <eps_dir> [out]}"
OUT="${2:-/data/prof_scratch/sweep_out}"
SCR=/data/prof_scratch/scripts
mkdir -p "$OUT"

# PAR -> n_eps (>=2x oversubscription). Domains are stride-1 unique (DBASE+slot),
# so DBASE + n_eps <= 232 (FastDDS cap). DBASE=10 -> n_eps <= 222.
declare -A NEPS=( [32]=128 [64]=160 [80]=200 [96]=200 )
DBASE=10
for PAR in 32 64 80 96; do
  N=${NEPS[$PAR]}
  echo "############ PAR=$PAR n_eps=$N (dbase=$DBASE, domains $DBASE..$((DBASE+N-1))) ############"
  bash "$SCR/prof_par_sweep.sh" "$EPS" "$PAR" "$N" 8 "$DBASE" "$OUT"
  sleep 3
done
echo "PAR_SWEEP_ALL_DONE"
cat "$OUT/sweep_results.csv" 2>/dev/null
