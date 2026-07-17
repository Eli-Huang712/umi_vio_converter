#!/usr/bin/env bash
# Remove raw-tree build scratch dirs left by FAILED conversions.
# Guarded: only deletes directories literally named "episode" that sit directly
# under a raw hash dir ($RAW_ROOT/<subset>/<hash>/episode). NEVER touches
# episode.mcap (a FILE) or the sidecar <hash>.json. Idempotent.
set -u
RAW_ROOT="${RAW_ROOT:-/data/umi_raw_data_260714}"
SUBSETS="${SUBSETS:?set SUBSETS='A_019 A_020 E_001'}"

removed=0 kept_nonempty=0
for s in $SUBSETS; do
  while IFS= read -r d; do
    # guard: path must be exactly <RAW_ROOT>/<subset>/<hash>/episode
    case "$d" in
      "$RAW_ROOT/$s"/*/episode) ;;
      *) echo "SKIP_UNEXPECTED $d" >&2; continue ;;
    esac
    rm -rf "$d" && removed=$((removed+1))
  done < <(find "$RAW_ROOT/$s" -mindepth 2 -maxdepth 2 -type d -name episode 2>/dev/null)
done

# sanity: raw hash dirs should now hold only episode.mcap + <hash>.json
leftover=0
for s in $SUBSETS; do
  n=$(find "$RAW_ROOT/$s" -mindepth 2 -maxdepth 2 -type d -name episode 2>/dev/null | wc -l)
  leftover=$((leftover+n))
done
echo "SCRATCH_CLEAN node=$(hostname) removed=$removed leftover=$leftover"
