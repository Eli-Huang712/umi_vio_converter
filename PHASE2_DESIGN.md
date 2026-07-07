# Phase 2 — persistent VIO worker (design + SHELVED with findings)

> **Status (2026-07-08): SHELVED — not shipped.** The persistent worker is
> implemented and functional (`persistent_worker.py` / `.sh`) but **does not pass
> the correctness gate**, and the ceiling is far lower than hoped. Two
> independent, empirically-established reasons — see **Outcome** at the bottom.
> Phase 1 (parallel wrists, committed `c88c6ea`) stands and is the shipped win.

## Goal
Raise the FULL-conversion **throughput ceiling** by eliminating the per-episode
fixed overhead measured at ~20 s of each 38.5 s build_map: TensorRT model load,
process spawn, CUDA context init, ROS discovery, and the launch `sleep`. Actual
mapping compute is only ~1.6 s; per-frame perception is the irreducible ~15 s.
Amortizing the fixed overhead can roughly **halve** per-episode work → ~2×
throughput (baseline 1188 ep/hr @ PAR=64).

## Hard constraint
VIO **output** (poses.npy) must be unchanged. A persistent process reuses the
CUDA context / TRT engines / Python interpreter across episodes; any state leak
would silently corrupt poses. → **pose-equivalence gate** is mandatory before
shipping (see Validation).

## Why 2 processes, not 1
`perception_node` (MultiThreadedExecutor, async infer) and `build_map_node`
(SingleThreadedExecutor, paced BagPlayer) run as **separate processes** talking
over ROS. Their real-time paced interplay determines which frames perception
finishes before build_map advances `/clock`. Merging them into one executor
changes threading/timing → risks changing poses. So the persistent design keeps
**two processes**, preserving the exact concurrency profile; only the process
*lifetime* changes (persist across episodes instead of one-shot).

## Architecture
Three roles in one file `persistent_worker.py` (selected by `--role`):

- **perception child** (`--role perception`): `rclpy.init()` once (domain fixed
  by `ROS_DOMAIN_ID` env at launch). Model classes monkeypatched to singletons
  (load once). Loop reading line commands on stdin:
  - `RUN` → construct fresh `PerceptionNode` (clean SLAM state, cached models),
    add to a `MultiThreadedExecutor`, spin via `spin_once` in a loop while
    watching stdin; reply `READY`.
  - `STOP` → destroy the node + executor; reply `STOPPED`.
  - `QUIT` → `rclpy.shutdown()`, exit.
- **buildmap child** (`--role buildmap`): `rclpy.init()` once, same domain,
  cached models. Loop:
  - `RUN <bag> <mapdir> <sensor> <play_rate>` → construct fresh `BagPlayer` +
    `BuildMapNode` + `ImageTransportsNode` in a `SingleThreadedExecutor`,
    `wait_for_perception_subscribers`, `while play_next(): spin_once`,
    `save_mapping()`, destroy all; reply `DONE <status>`.
  - `QUIT` → shutdown, exit.
- **orchestrator** (default): launch one perception + one buildmap child on a
  chosen `(domain, gpu)`, then per episode run the protocol:
  1. perception `RUN` → wait `READY`  (perception subscribes first)
  2. buildmap `RUN …` → wait `DONE`   (barrier passes once perception subscribed)
  3. perception `STOP` → wait `STOPPED` (fresh SLAM state next episode)
  Then merge (tactile) as today. For batch throughput, launch N such pairs, each
  on a distinct domain/GPU.

## Model caching (the amortization)
Monkeypatch the model wrappers imported into `perception_node` /
`build_map_node` (`SuperPointTRT`, `LightGlueTRT`, `StereoEngineTRT`, plus any
build_map retrieval/loop-closure engines) to return process-singletons: first
construction loads the `.plan`; later constructions in the same process return
the cached instance. Inference is deterministic and, within an episode, the
concurrency profile (one context, async infers) is identical to baseline; across
episodes it is sequential. No change to `models_trt.py` or the node files.

## Domain handling
`ROS_DOMAIN_ID` is fixed at `rclpy.init()`, so a persistent pair keeps one domain
for its whole life. Episodes reuse it; full node destroy + the
`wait_for_perception_subscribers` barrier isolate episode N from N+1. Parallel
pairs get distinct domains at launch. Keep every domain ≤ 232 (FastDDS cap).

## Validation gate (must pass before ship)
1. **Determinism baseline**: run the same episode twice fresh-process; compare
   poses.npy. Establishes whether the bar is bit-identical or a tolerance.
2. **Equivalence**: run K≥10 varied episodes through both the persistent worker
   and the fresh-process baseline; require poses within the determinism bar for
   **every** episode + every wrist. Any miss ⇒ do not ship; report the leak.
3. Re-check across a persistent run of many episodes (state-leak surfaces late):
   compare episode #1 vs episode #50 of a persistent run against their baselines.

## Files
- `persistent_worker.py` (new) — the three roles above.
- `umi_vio_converter.py` — add `--engine persistent` to route FULL through the
  worker instead of `_build_map_one_sensor.sh` (default stays the shell path
  until the gate passes).

---

## Outcome (2026-07-08) — why this is shelved

Implemented in full and driven through the gate (`persistent [E1,E2,E1]` vs
fresh-process baselines, on the plant_collection dataset). Two blockers, each
sufficient on its own:

**1. The reward is only ~15–18%, not ~2×.** Measured breakdown of a 77 s episode:
2×38.5 s build_map (the two wrists, sequential) whose *actual mapping compute is
~1.6 s*; the ~37 s/build_map is ~15 s irreducible per-frame perception + ~20 s
fixed overhead. But most of that "overhead" is per-frame perception and CUDA/ROS
init that a persistent process still pays — only model *load* (~5 s) and process
spawn are truly amortizable. Persistent E1 ran 62 s vs 77 s baseline = ~18 %. The
profiling was right: **per-frame perception dominates, so amortizing startup caps
the gain at ~15–18 %.**

**2. Correctness cannot be guaranteed.** Two empirical findings:
   - **The baseline VIO is not bit-reproducible.** Three identical fresh-process
     runs of the same episode gave three different right_wrist `poses.npy` (hashes
     `eb46939d` / `10fd2263` / `c411f5bf`, keyframe count 37/37/38). The pipeline
     is timing-sensitive: paced BagPlayer + async MultiThreadedExecutor perception
     → `ApproximateTimeSynchronizer` pairs stereo/imu frames differently under
     scheduling jitter. So a **bit-identical gate is impossible** and a tolerance
     gate would have to accept keyframe-count changes — i.e. genuinely different
     maps — which defeats the point of a "no VIO change" guarantee.
   - **The persistent worker leaks state across jobs anyway.** left_wrist is
     perfectly reproducible fresh (39/39/39) yet drifted to 41 on the 3rd job of a
     persistent run, and pose counts grow monotonically across jobs. Four
     principled fixes did **not** remove it: (a) dedicated fd control channel,
     (b) `rclpy.init/shutdown` per job (fresh DDS participant), (c) cache only the
     read-only TRT engine with fresh context/buffers/graph per job, (d) clear the
     three process-level `(a)lru_cache_numpy` memoizations (SuperPointTRT.infer,
     LightGlueTRT.infer, math_utils.estimate_pose) between jobs. The residual leak
     lives in native/C++ state (gtsam / CUDA / TensorRT) not resettable from
     Python without re-initialising it — which erases the amortization.

**Conclusion.** A persistent worker on this pipeline is a ~15–18 % throughput gain
bought with a real risk of silently corrupting VIO poses over a long run, on data
where that corruption is invisible and expensive. Not worth it. The real
throughput levers remain zero-risk and larger: **horizontal scaling** (H200-1 +
H200-2 ≈ 2×) and **BACKFILL-not-FULL** for episodes that already have poses.

The code is kept on this branch for reference (functional, just not
correctness-safe); it is **not** wired into `umi_vio_converter.py` and the default
FULL path is unchanged.
- No edits to tinynav `perception_node.py` / `build_map_node.py` / `models_trt.py`.
