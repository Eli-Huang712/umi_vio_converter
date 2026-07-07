# FULL-conversion throughput benchmark

Measured on an idle **8× NVIDIA H200** node (192 CPU cores), tinynav container,
real UMI episodes, end-to-end FULL conversion (VIO build_map ×2 wrists → merge
with tactile). Every data point is a clean run (0 failures, 0 skips).

| PAR | procs/GPU | throughput | mean s/episode | loadavg /192 | GPU util peak | GPU mem/GPU |
|----:|----------:|-----------:|---------------:|-------------:|--------------:|------------:|
| 1   | –         | ~46 ep/hr  | 78             | ~1           | –             | 2.4 GB      |
| 8   | 1         | 275 ep/hr  | 99             | 32 (17%)     | 12%           | 2.4 GB      |
| 24  | 3         | 714 ep/hr  | ~120           | 99 (52%)     | 61%           | 7.3 GB      |
| 32  | 4         | 883 ep/hr  | 126            | 72 (38%)     | 43%           | 9.7 GB      |
| 48  | 6         | 1077 ep/hr | 158            | 108 (56%)    | 96%           | 14.6 GB     |
| 64  | 8         | 1188 ep/hr | 193            | 177 (92%)    | 80%           | 19.5 GB     |

## Takeaways
- **Recommended sustainable: PAR=48 (6/GPU) ≈ 1077 ep/hr.** CPU at 56%, GPU mem
  14.6/143 GB — comfortable headroom, safe on a shared node.
- **Absolute max: PAR=64 (8/GPU) ≈ 1188 ep/hr**, but loadavg 177/192 (92% CPU) —
  dedicated-node only. The 48→64 step buys +10% throughput for +33% workers.
- **Binding constraint is CPU** (192 cores), then per-process GPU compute. GPU
  memory never binds (peak 31/143 GB).
- Per-episode latency grows 78→193 s with parallelism — this is the expected
  latency-for-throughput trade (model load + `--play_rate 20` playback floor +
  sequential 2-sensor build_map), not a regression.

## Gotchas baked into the tooling
- **ROS2/FastDDS caps `ROS_DOMAIN_ID` ≤ 232.** Keep `DOMAIN_BASE + PAR ≤ 232`
  (the batch driver default base 70 is fine up to ~160 workers). Exceeding it
  makes DDS participants crash instantly with "domainId over 232".
- **tmux panes do not inherit the `docker exec` env**, so `CUDA_VISIBLE_DEVICES`
  is re-exported inside the tmux command in `_build_map_one_sensor.sh` —
  otherwise every parallel worker piles onto GPU 0.

## Cost note
The 3-way dispatch means existing pose-only products never re-run VIO: they take
the **BACKFILL** path (pure CPU, seconds/episode, no GPU). Only raw episodes with
no `*_with_pose.mcap` pay the FULL cost above.
