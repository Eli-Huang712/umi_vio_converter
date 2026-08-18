#!/usr/bin/env bash
# Backfill the per-episode sidecar <hash>.json into the VIO product tree.
# The raw→VIO conversion emitted only episode_with_pose.mcap and dropped the
# sidecar JSON that carries the task annotation. This copies each raw
#   $RAW_ROOT/<subset>/<hash>/<hash>.json
# next to its product
#   $VIO_ROOT/<subset>/<hash>/<hash>.json
# for every hash dir that already has a product. Idempotent (skips if the dest
# json exists and matches), non-destructive (only ever writes *.json into the
# product dir; never touches the mcap or the raw tree). Runs INSIDE the
# conversion environment.
set -u
RAW_ROOT="${RAW_ROOT:?set RAW_ROOT to the input tree}"
VIO_ROOT="${VIO_ROOT:?set VIO_ROOT to the output tree}"

copied=0 skipped=0 missing_src=0 total=0
# Visit every product (dir containing episode_with_pose.mcap)
while IFS= read -r wp; do
  total=$((total+1))
  d=$(dirname "$wp")                       # $VIO_ROOT/<subset>/<hash>
  rel="${d#$VIO_ROOT/}"                     # <subset>/<hash>
  srcdir="$RAW_ROOT/$rel"                   # raw sibling dir
  # the single sidecar json in the raw hash dir
  src=$(ls "$srcdir"/*.json 2>/dev/null | head -1)
  if [ -z "$src" ]; then missing_src=$((missing_src+1)); continue; fi
  dst="$d/$(basename "$src")"
  if [ -f "$dst" ] && cmp -s "$src" "$dst"; then skipped=$((skipped+1)); continue; fi
  cp -p "$src" "$dst" && copied=$((copied+1)) || echo "COPY_FAIL $src -> $dst" >&2
done < <(find "$VIO_ROOT" -type d -name episode_with_pose.mcap 2>/dev/null)

echo "BACKFILL_DONE root=$VIO_ROOT products=$total copied=$copied skipped=$skipped missing_src=$missing_src"
