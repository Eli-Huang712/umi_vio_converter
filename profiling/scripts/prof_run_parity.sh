#!/usr/bin/env bash
# =============================================================================
# prof_run_parity.sh — build stock & df *_with_pose.mcap for ONE episode from
# already-saved poses.npy, then diff non-pose channels. Runs INSIDE h2 host.
#
# merge_sqlite_mcap only reads <work>/<sensor>_db/poses.npy and passes every
# other channel through verbatim from the raw bag (flatbuffer_reader), so the
# tactile/IMU/gripper/camera_info channels must be byte-identical between the
# stock-poses merge and the df-poses merge. This proves that empirically.
#
# Usage: prof_run_parity.sh <episode_hash>
# Requires /tmp/val/<hash>/{stock,dffix}_{left,right}_wrist_0.npy to exist.
# =============================================================================
set -u
HASH="${1:?need episode hash}"
C=tinynav_flatbuffer
RAW="/data/p1_scratch/raw/${HASH}/episode.mcap"
VAL="/tmp/val/${HASH}"

dex() { docker exec "$C" bash -lc "
  set +u
  for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do [ -f \$f ] && source \$f; done
  export PYTHONPATH=/tinynav:\$PYTHONPATH
  $1"; }

build_wp() {  # variant  poses_prefix
  local variant="$1" prefix="$2"
  local work="/data/p1_scratch/parity_${variant}"
  dex "rm -rf $work && mkdir -p $work/left_wrist_db $work/right_wrist_db
    cp $VAL/${prefix}_left_wrist_0.npy  $work/left_wrist_db/poses.npy
    cp $VAL/${prefix}_right_wrist_0.npy $work/right_wrist_db/poses.npy
    python3 /tinynav/tool/umi/merge_sqlite_mcap.py \
      --input_mcap $RAW \
      --pose-sensors left_wrist,right_wrist \
      --left_db $work/left_wrist_db --right_db $work/right_wrist_db \
      --output_mcap $work/episode_with_pose.mcap" > "/tmp/parity_${variant}.log" 2>&1
  echo "  $variant merge rc=$? -> $work/episode_with_pose.mcap"
}

echo "########## PARITY $HASH ##########"
build_wp stock stock
build_wp df    dffix
echo "=== channel parity (stock vs df with_pose.mcap) ==="
dex "python3 /tmp/prof_channel_parity.py \
  /data/p1_scratch/parity_stock/episode_with_pose.mcap \
  /data/p1_scratch/parity_df/episode_with_pose.mcap"
echo "PARITY_RUN_DONE"
