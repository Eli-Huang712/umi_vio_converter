# P1 direct-feed — current-stage (PAR=64 peak) profiling evidence

All measured on h2 (8×H200, idle), 128-ep scratch pool, direct-feed (UMI_DIRECT_FEED=1).
Stock baseline for contrast is from the authoritative report (VIO转换瓶颈分析报告.md).

## E1. PAR steady-state throughput (frames/hr, trimmed window, 0 fail except PAR=128)
stock PAR=64:  600,440 fph | CPU 32.7% | GPU sm 5.1%
df    PAR=32: 1,270,020 fph | CPU 44.9% | GPU sm 49.0%
df    PAR=48: 1,533,500 fph | CPU 55.5% | GPU sm 60.2%
df    PAR=64: 1,612,520 fph | CPU 56.8% | GPU sm 60.5%   <-- df PEAK (+169% vs stock@64)
df    PAR=80: 1,561,420 fph | CPU 51.5% | GPU sm 52.0%   (regress)
df    PAR=96: 1,121,220 fph | CPU 56.7% | GPU sm 55.6%   (regress)
df    PAR=128:1,200,620 fph | CPU 58.0% | GPU sm 54.0%   (4 failures)
=> df peaks at PAR=64; neither CPU (57%) nor aggregate GPU sm (60%) saturates at the peak.

## E2. Single df process on-CPU flamegraph (py-spy, largest ep, 2058 samples)
53.6%  run_conversion:294 = read_episode  -> ENTIRE H.264 decode done UPFRONT before any VIO
         (line 215 _decode 14.9%, decode_compressed_video/flatbuffer_codec 9.1%, PyAV parse) — GPU IDLE during this
33.0%  run_conversion:357 = the VIO feed loop (run_until_complete(perception.process))
         dominated by asyncio event-loop frames (base_events.py:1909 _run_once, events.py:80)
         = the run_graph busy-wait spin: `while cudaEventQuery()==NotReady: await asyncio.sleep(0)`
          -> CPU spins a core polling the GPU while waiting for kernel completion
 7.7%  numba JIT COMPILATION at runtime (compiler.py/lowering.py/dispatcher.py) — paid FRESH every
         process because every episode = a new process (njit funcs in math_utils recompile)

## E3. Per-thread %CPU (pidstat -t, same run) — GIL check
main python thread (GIL holder): 74.7% = 0.75 core
~450 native threads (TRT/CUDA/PyAV) each ~1.9%, summing ~9-10 cores total
=> NOT GIL-bound (unlike stock build_map top/sum=0.92 pinned ~1 core). Parallelism comes from
   native libs (PyAV decode, TensorRT, gtsam C++, numba) releasing the GIL. Matches footprint avg 4.7 cores.

## E4. Same-GPU N-proc (df, ONE card, decisive GPU-serialization test)
 N  per-proc_latency  throughput(wrist/hr)  GPU_sm%
 1     19.7s               183               18.2
 2     18.2s               348               31.8     (near-linear, no latency penalty)
 4     21.9s               571               48.7     (per-proc inflating, throughput sublinear)
 6     23.8s               706               63.0
 8     30.7s (+56%)        750 (saturating)  73.2
=> ONE card saturates at N~6-8: per-proc latency +56%, marginal throughput collapses
   (+165/+223/+135/+44 per step), sm climbs 18->73%. 8 cards × ~8 procs = PAR~64 = observed peak.
CONTRAST stock same-GPU (report): wall FLAT ~68s, throughput LINEAR 105->421, sm 2->7%
   => stock's GPU was IDLE (CPU/GIL bound); df's GPU is now the ACTIVE CONTENDED resource.

## E5. Isolated PAR=1 df single-wrist walls: 11.0–24.0s (size-dependent), median ~18s.

## E6. MPS A/B (N=8 same card, df) — DECISIVE
MPS OFF: wall 38.1s | per-proc 30.4s | throughput 756 wrist/hr | sm 72.9%
MPS ON : wall 25.1s | per-proc 19.3s | throughput 1147 wrist/hr | sm 54.4%
=> MPS +52% throughput, per-proc latency −36% (30.4->19.3s ≈ isolated N=1 latency 19.7s — MPS
   erases the co-tenancy penalty at N=8). sm DROPS (73->54%) while throughput RISES: time-sliced
   contention (MPS off) reads as "busy" via context-switch/serialized waiting; MPS runs kernels
   concurrently so they finish faster (less wall at busy). CONFIRMS per-card GPU-context
   serialization is the binding constraint. (Report refuted MPS for STOCK = 0% because stock GPU
   was idle; df is the opposite regime.)

## E6b. FULL-NODE MPS A/B (df PAR=64 steady, all 8 GPUs) — node-level reality check
no-MPS: 1,612,520 frames/hr | CPU 56.8% | GPU sm 60.5%
MPS on: 1,861,200 frames/hr | CPU 58.9% | GPU sm 50.3% | 0 fail
=> node gain only +15.4% (vs +52% single-card). As predicted: MPS relieves the PROXIMATE
   per-card serialization, but at the node the ROOT cause (low GPU duty cycle from the upfront
   decode phase + host coordination) reasserts — NEITHER CPU (59%) nor GPU (50%) saturates even
   with MPS. The remaining headroom needs pipeline-decode (feed GPU during decode), not MPS alone.

## E7. Side observations
- df driver's CUDA/rclpy SHUTDOWN hangs AFTER work completes (process lingers holding ~1.1GB GPU
  mem in D-state, survives SIGKILL until driver call returns). Masked in production by the `timeout`
  watchdog in the df branch of _build_map_one_sensor.sh (work is already done). Real teardown issue.
- upfront full-episode decode (E2, 53%) means each proc alternates a long GPU-idle CPU-heavy decode
  phase with a GPU phase; across co-tenant procs this keeps a single card's sm below 100% even at N=8.
