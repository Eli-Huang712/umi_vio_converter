#!/usr/bin/env bash
# =============================================================================
# prof_copy_eps.sh — copy N raw episode.mcap files h1 -> h2 over the fast
# internal link (562 MB/s), placing ONLY the raw mcap (never the episode/
# workdir with existing with_pose products) into a clean scratch pool on h2 so
# every episode dispatches as FULL.
#
# RUN ON h1 (with agent forwarding: ssh -A h1 'bash -s' < prof_copy_eps.sh N).
# Requires: agent-forwarded key that authenticates h1 -> 214.30.239.42 (h2).
# =============================================================================
set -uo pipefail
N="${1:-256}"
SRC=/data/shared/datasets/plant_collection/raw/26-06-23/dataloop-umi/184
H2=214.30.239.42
DST=/data/shared/datasets/prof_scratch/eps
SSH_H2="ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ServerAliveInterval=10"

cd "$SRC" || { echo "no src $SRC"; exit 1; }
# build a validated list of <hash>/episode.mcap that actually exist
: > /tmp/eplist.txt
cnt=0
for h in $(ls); do
  [ -f "$h/episode.mcap" ] || continue
  echo "$h/episode.mcap" >> /tmp/eplist.txt
  cnt=$((cnt+1))
  [ "$cnt" -ge "$N" ] && break
done
echo "listed $cnt episode.mcap files; streaming to h2:$DST"

$SSH_H2 "$H2" "mkdir -p $DST"
# stream tar of just the raw mcaps; preserves <hash>/episode.mcap layout
tar cf - -T /tmp/eplist.txt | $SSH_H2 "$H2" "tar xf - -C $DST && echo H2_EXTRACT_OK"
echo "verifying on h2..."
$SSH_H2 "$H2" "find $DST -name episode.mcap | wc -l | sed 's/^/n_episode_mcap=/'; du -sh $DST 2>/dev/null"
echo "COPY_EPS_DONE"
