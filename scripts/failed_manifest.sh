#!/usr/bin/env bash
# Enumerate VIO conversion FAILURES per node into a manifest:
#   <subset> <hash> <rc> <elapsed_s> <error_snippet>
# Failure = raw episode has NO product (episode_with_pose.mcap missing) in VIO tree.
# Cross-checks results.tsv (col2=rc) against the actual product tree, and greps the
# per-episode log for the root cause (e.g. focal=0 calibration defect).
set -u
RAW_ROOT="${RAW_ROOT:?set RAW_ROOT to the input tree}"
VIO_ROOT="${VIO_ROOT:?set VIO_ROOT to the output tree}"
LOGDIR="$VIO_ROOT/_vio_logs"
OUT="${OUT:-$VIO_ROOT/_vio_logs/failed_manifest.tsv}"
SUBSETS="${SUBSETS:?set SUBSETS='A_019 A_020 E_001'}"

printf 'subset\thash\trc\telapsed_s\terror\n' > "$OUT"
nfail=0
for s in $SUBSETS; do
  for rawd in "$RAW_ROOT/$s"/*/; do
    [ -d "$rawd" ] || continue
    h=$(basename "$rawd")
    # a hash is FAILED iff it has no product in the VIO tree
    if [ ! -d "$VIO_ROOT/$s/$h/episode_with_pose.mcap" ]; then
      ep="$RAW_ROOT/$s/$h/episode.mcap"
      # rc + elapsed from results.tsv (main + retry), take the last record for this ep
      line=$(grep -F "$ep" "$LOGDIR"/results.tsv "$LOGDIR"/results.tsv.retry 2>/dev/null | tail -1)
      rc=$(echo "$line" | awk -F'\t' '{print $2}'); el=$(echo "$line" | awk -F'\t' '{print $3}')
      # root-cause snippet from the per-episode log
      tag=$(echo "$ep" | md5sum | cut -c1-12)
      err=$(grep -aoE "focal_length must be positive[^\"]*|ValueError:[^\"]*|too_many_attempts|short_read|Error:[^\"]*" "$LOGDIR/$tag.log" 2>/dev/null | head -1 | cut -c1-80)
      [ -z "$err" ] && err="(no log / unknown)"
      printf '%s\t%s\t%s\t%s\t%s\n' "$s" "$h" "${rc:-NA}" "${el:-NA}" "$err" >> "$OUT"
      nfail=$((nfail+1))
    fi
  done
done
echo "FAILED_MANIFEST node=$(hostname) subsets='$SUBSETS' failures=$nfail out=$OUT"
echo "--- 失败原因分布 ---"
tail -n +2 "$OUT" | awk -F'\t' '{print $5}' | sed -E 's/got [0-9.]+//; s/[0-9]+//g' | sort | uniq -c | sort -rn | head
echo "--- 各子集失败数 ---"
tail -n +2 "$OUT" | awk -F'\t' '{print $1}' | sort | uniq -c
