#!/usr/bin/env bash
# =============================================================================
# prof_validate_p1.sh — P1 direct-feed correctness gate (runs INSIDE h2 host).
#
# For each (episode, sensor): run stock N_STOCK times + direct-feed N_DF times,
# save each poses.npy, then compute:
#   * stock self-jitter  : ATE/RPE between stock runs   (the tolerance band)
#   * df determinism     : ATE/RPE between df runs       (expect ~0)
#   * df vs stock        : ATE/RPE df[0] vs stock[0]      (must be <= band)
# Also records wall time per run.
#
# Usage:  prof_validate_p1.sh <episode_dir_hash> [gpu_id]
# Env:    N_STOCK (default 3), N_DF (default 2), SENSORS (default "left_wrist right_wrist")
# =============================================================================
set -u
HASH="${1:?usage: prof_validate_p1.sh <episode_hash> [gpu]}"
GPU="${2:-0}"
N_STOCK="${N_STOCK:-3}"
N_DF="${N_DF:-2}"
SENSORS="${SENSORS:-left_wrist right_wrist}"
# DBASE must be GLOBALLY UNIQUE per concurrent episode: tmux session names are
# disambiguated only by domain (all bags are episode.mcap), and ROS domains must
# not overlap. Space parallel invocations >= (N_STOCK+N_DF)*2 apart.
DBASE="${DBASE:-150}"
C=tinynav_flatbuffer
EP="/data/p1_scratch/raw/${HASH}/episode.mcap"
WORK="/data/p1_scratch/raw/${HASH}/episode"
OUT="/tmp/val/${HASH}"

dex() { docker exec -e CUDA_VISIBLE_DEVICES="$GPU" "$@" "$C" bash -lc "
  set +u
  for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do [ -f \$f ] && source \$f; done
  export PYTHONPATH=/tinynav:\$PYTHONPATH
  $1"; }

docker exec "$C" bash -lc "mkdir -p $OUT"

run_one() {  # mode sensor idx domain
  local mode="$1" sensor="$2" idx="$3" domain="$4"
  local t0 t1 extra_env=""
  [ "$mode" = "df" ] && extra_env="-e UMI_DIRECT_FEED=1"
  t0=$(date +%s.%N)
  docker exec -e CUDA_VISIBLE_DEVICES="$GPU" $extra_env "$C" bash -lc "
    set +u
    for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do [ -f \$f ] && source \$f; done
    export PYTHONPATH=/tinynav:\$PYTHONPATH
    bash /tinynav/tool/umi/_build_map_one_sensor.sh $EP $sensor $domain 20" \
    > "/tmp/val_${HASH}_${mode}_${sensor}_${idx}.log" 2>&1
  local rc=$?
  t1=$(date +%s.%N)
  local wall; wall=$(awk -v a=$t0 -v b=$t1 'BEGIN{printf "%.1f", b-a}')
  if [ $rc -ne 0 ]; then echo "  ${mode}[$idx] $sensor FAILED rc=$rc wall=${wall}s"; return 1; fi
  docker exec "$C" bash -lc "cp $WORK/${sensor}_db/poses.npy $OUT/${mode}_${sensor}_${idx}.npy"
  local nkf; nkf=$(docker exec "$C" bash -lc "python3 -c 'import numpy as np;print(len(np.load(\"$OUT/${mode}_${sensor}_${idx}.npy\",allow_pickle=True).item()))'")
  echo "  ${mode}[$idx] $sensor wall=${wall}s kf=${nkf}"
}

cmp() {  # label a b
  docker exec "$C" bash -lc "PYTHONPATH=/tinynav python3 /tmp/prof_compare_poses.py $OUT/$2 $OUT/$3 --label '$1'"
}

echo "########## EPISODE $HASH (gpu $GPU, dbase $DBASE) ##########"
D=$DBASE
for sensor in $SENSORS; do
  echo "=== $sensor : $N_STOCK stock + $N_DF df runs ==="
  for i in $(seq 0 $((N_STOCK-1))); do run_one stock "$sensor" "$i" "$D"; D=$((D+1)); done
  for i in $(seq 0 $((N_DF-1))); do run_one df "$sensor" "$i" "$D"; D=$((D+1)); done
  echo "--- $sensor comparisons ---"
  for i in $(seq 1 $((N_STOCK-1))); do cmp "stock_jitter_0v$i:$sensor" "stock_${sensor}_0.npy" "stock_${sensor}_${i}.npy"; done
  for i in $(seq 1 $((N_DF-1))); do cmp "df_determinism_0v$i:$sensor" "df_${sensor}_0.npy" "df_${sensor}_${i}.npy"; done
  cmp "df_vs_stock:$sensor" "df_${sensor}_0.npy" "stock_${sensor}_0.npy"
done
echo "VALIDATE_DONE $HASH"
