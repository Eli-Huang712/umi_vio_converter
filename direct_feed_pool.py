#!/usr/bin/env python3
"""direct_feed_pool.py — persistent-worker pool for direct-feed VIO conversion.

!!! VERDICT: SHELVED — DO NOT USE IN PRODUCTION. !!!
Node-level testing (see P1_IMPLEMENTATION.md §6.5-D, profiling/results/p2_sweep_evidence.md)
showed the pool (a) does NOT raise node throughput (the node is GPU-bound at the
pipeline+MPS peak, so the pool's per-episode warm-up savings free CPU that has nowhere
to go — 1.84M < 1.95M frames/hr steady), and (b) UNDER 48-way concurrency + MPS becomes
LESS RELIABLE: one episode crashed with `IndexError: index 16384 / size 512` and another
worker segfaulted, losing its whole shard. That crash was isolated to the concurrent+MPS
regime — the same episode succeeds in a fresh process, as the 4th item of a single-worker
sequence, and in a 12-episode single-worker run — so it is an MPS-under-heavy-concurrency
fragility, not a reproducible single-process cross-episode leak. Either way it is a failure
with a larger blast radius (whole shard vs one episode). Production path = pipeline + MPS +
FRESH PROCESS PER EPISODE (direct_feed_build_map.py). Kept only as a reproducible negative result.

STAGE B (gated experiment). Motivation: at the pipeline+MPS peak, spawning a fresh
process per episode pays process spawn + a CUDA/rclpy shutdown hang (E7) and creates
synchronized startup bursts at high PAR (host over-subscription). A persistent worker
that stays alive across episodes avoids per-episode spawn + moves the shutdown hang out
of the hot path (only once at worker exit) + lets a continuous work queue smooth the load.

CORRECTNESS IS THE RED LINE. History (提速测试报告.md §4): the old persistent worker
matched baseline on job 1 then DRIFTED (native state leak in gtsam/CUDA/TRT C++,
keyframe count +2/episode, four fixes failed) — silently corrupting poses. So here:
  * every episode gets FRESH PerceptionNode + BuildMapNode + models (no reuse of SLAM
    state, intrinsics — dodging the `if K is None` trap — or model infer caches);
  * only the PROCESS is reused (imports once, numba disk-cache warm, rclpy.init once);
  * ``prof_pool_gate.sh`` HARD-gates: pooled poses must be BIT-IDENTICAL to a fresh
    process for every episode. Any drift => the leak recurs => the pool is disabled by
    default and we fall back to fresh-process-per-episode (Stage A pipeline+MPS).

Modes:
  worker  : process a shard (one persistent process, rclpy.init once) — the reusable unit.
  run     : orchestrator — shard a work list across N GPU-pinned worker subprocesses.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time


def _worker(shard_path: str):
    """Process every (bag, sensor, map_path) in shard_path in ONE persistent process.

    rclpy is init'd once here; each episode uses fresh nodes (manage_rclpy=False), so
    no SLAM/intrinsics state carries over. The shutdown hang (if any) happens once, at
    the very end, and is bounded by the orchestrator's per-worker timeout.
    """
    import rclpy

    sys.path.insert(0, "/tinynav")
    from tool.umi.direct_feed_build_map import run_conversion

    items = []
    with open(shard_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            bag, sensor, map_path = line.split("|")
            items.append((bag, sensor, map_path))

    rclpy.init()
    ok = 0
    try:
        for i, (bag, sensor, map_path) in enumerate(items):
            t0 = time.perf_counter()
            try:
                run_conversion(bag, sensor, map_path, verbose_timer=False,
                               pipeline=True, manage_rclpy=False)
                ok += 1
                print(f"WORKER_ITEM ok idx={i} sensor={sensor} wall={time.perf_counter()-t0:.1f}s "
                      f"{os.path.dirname(bag)}", flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"WORKER_ITEM FAIL idx={i} sensor={sensor} err={type(exc).__name__}:{exc}", flush=True)
    finally:
        print(f"WORKER_DONE ok={ok}/{len(items)}", flush=True)
        try:
            rclpy.shutdown()
        except Exception:
            pass


def _ros_env_prefix():
    """Shell snippet to source ROS overlays (workers are launched via bash -lc)."""
    return (
        "set +u; "
        "for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash "
        "/3rdparty/plotjuggler_ws/install/local_setup.bash "
        "/3rdparty/message_filters_ws/install/local_setup.bash; do [ -f \"$f\" ] && source \"$f\"; done; "
        "export PYTHONPATH=/tinynav:$PYTHONPATH; "
    )


def _run_pool(eps, sensors, n_workers, ngpu, out_dir, timeout_s, stagger_s=0.0):
    """Orchestrator: static-shard the (episode, sensor) work list across n_workers
    persistent worker subprocesses, each pinned to gpu = worker_idx % ngpu (MPS env,
    if set, is inherited so workers share the GPU context).

    ``stagger_s`` delays each successive worker launch to avoid a synchronized cold
    start (48 workers all loading models + JIT + CUDA/MPS context at once thrashes the
    node — first-episode wall balloons ~3x). Staggering smooths that startup burst.
    """
    os.makedirs(out_dir, exist_ok=True)
    work = [(bag, s, f"{os.path.dirname(bag)}/{s}_db") for bag in eps for s in sensors]
    shards = [[] for _ in range(n_workers)]
    for i, item in enumerate(work):
        shards[i % n_workers].append(item)

    procs = []
    for w, shard in enumerate(shards):
        if not shard:
            continue
        shard_file = os.path.join(out_dir, f"shard_{w}.txt")
        with open(shard_file, "w") as f:
            for bag, s, mp in shard:
                f.write(f"{bag}|{s}|{mp}\n")
        gpu = w % ngpu
        cmd = (
            _ros_env_prefix()
            + f"CUDA_VISIBLE_DEVICES={gpu} timeout {timeout_s} "
            + f"python3 {os.path.abspath(__file__)} worker --shard {shard_file} "
            + f"> {out_dir}/worker_{w}.log 2>&1"
        )
        procs.append(subprocess.Popen(["bash", "-lc", cmd]))
        print(f"launched worker {w} gpu={gpu} items={len(shard)}", flush=True)
        if stagger_s > 0:
            time.sleep(stagger_s)

    rc = [p.wait() for p in procs]
    print(f"POOL_DONE workers={len(procs)} rc={rc}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="mode", required=True)

    w = sub.add_parser("worker")
    w.add_argument("--shard", required=True)

    r = sub.add_parser("run")
    r.add_argument("--eps_dir", required=True)
    r.add_argument("--sensors", default="left_wrist,right_wrist")
    r.add_argument("--n_workers", type=int, default=8)
    r.add_argument("--ngpu", type=int, default=8)
    r.add_argument("--out", default="/data/p1_pool/pool_out")
    r.add_argument("--timeout_s", type=int, default=3600)
    r.add_argument("--n_eps", type=int, default=0, help="0 = all")
    r.add_argument("--stagger_s", type=float, default=0.0,
                   help="delay between worker launches to avoid synchronized cold start")

    args = ap.parse_args()
    if args.mode == "worker":
        import logging
        logging.basicConfig(level=logging.INFO)
        _worker(args.shard)
    else:
        import glob
        eps = sorted(glob.glob(os.path.join(args.eps_dir, "*", "episode.mcap")))
        if args.n_eps:
            eps = eps[: args.n_eps]
        _run_pool(eps, args.sensors.split(","), args.n_workers, args.ngpu, args.out, args.timeout_s,
                  stagger_s=args.stagger_s)


if __name__ == "__main__":
    main()
