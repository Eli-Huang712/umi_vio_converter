# Architecture

## Execution flow

```text
convert.sh
  -> vio_batch_dispatch.sh
    -> umi_vio_converter.sh
      -> umi_vio_converter.py
        -> _build_map_one_sensor.sh
          -> direct_feed_build_map.py
            -> tinynav.core.perception_node       (external)
            -> tinynav.core.build_map_node        (external)
        -> patched/merge_sqlite_mcap.py
        -> backfill_tactile.py                     (BACKFILL only)
```

## Repository ownership

This repository owns:

- UMI FlatBuffer/video/IMU decoding adapters;
- per-sensor VIO invocation and process isolation;
- batch concurrency, GPU assignment and ROS domain assignment;
- pose/tactile merge and sidecar propagation;
- retry, logs and output-tree layout.

The compatible TinyNav runtime owns:

- the VIO/perception implementation;
- map-building nodes and GTSAM/TensorRT integration;
- neural-network models and GPU-specific TensorRT engines;
- ROS 2 base runtime and message packages.

Keeping this boundary explicit prevents a source-only checkout from being mistaken for a
standalone, hardware-independent VIO package.

## Per-episode state machine

`umi_vio_converter.py` selects an action from output state:

```text
missing metadata.yaml        -> FULL
metadata exists, no tactile  -> BACKFILL
metadata exists + tactile    -> SKIP
```

`FULL` runs each enabled pose sensor, then merges poses and tactile data. `BACKFILL`
rewrites only the existing output bag to add tactile channels. `SKIP` is idempotent.

## Concurrency model

The dispatcher gives every in-flight episode:

- one slot in `[0, PAR)`;
- one ROS domain ID `DBASE + slot`;
- one GPU chosen round-robin from `GPUS`.

This isolates ROS discovery and avoids touching GPUs not listed by the caller. The final
batch result is successful only when every episode succeeds in the first or retry round.

## Data integrity boundary

Products are written under a separate output root. The relative input subtree is mirrored,
and an existing JSON sidecar is copied next to the output. Temporary build scratch is
removed after each attempt. Acceptance requires checking the final MCAP schema and counts;
file existence alone is insufficient.
