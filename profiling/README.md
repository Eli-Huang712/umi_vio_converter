# profiling/ — VIO 转换瓶颈剖析（存档）

本目录是 2026-07-08 那轮 CPU/GPU 性能剖析的全部产物。**结论已整合进仓库根的
[`VIO转换瓶颈分析报告.md`](../VIO转换瓶颈分析报告.md)**（权威版，先读它）。本目录供复现与追溯。

## 一句话结论
批量 VIO 转换的单节点吞吐平台（PAR=64 ≈ 1196 集/时 ≈ 54.5 万帧/时）**不是任何计算硬墙**
（CPU 50%、GPU 7%、IO/显存余量 20×）。真瓶颈 = **`build_map` 进程的单线程 ROS2 图像消息序列化**
（GIL 钉在 ~1 核）。**MPS 实测零收益**（证伪 GPU-context 串行化假设）。→ P1 = 削减该序列化开销。

## 目录
- `scripts/` — 18 个可复现脚本（`prof_*.sh` / `prof_*.py` + `profile_one.sh`）。
  一键链路：`prof_recreate_container.sh` → `prof_footprint.sh` → `prof_run_par_sweep.sh`
  → `prof_run_gpu_experiments.sh` → `prof_flamegraph.sh` → `prof_plots.py`。
- `results/` — 7 类图（PNG/SVG）+ 原始采样数据（CSV/JSON/TXT）。
- `docs/` — 存档文档：
  - `VIO流程与瓶颈测试设计.md` — **P1 必读**：基于源码的流水线逐步解剖 + 时序/并发模型。
  - `剖析报告.md` — 本轮剖析的详细过程记录（已被根目录整合版取代）。
  - `提速测试报告.md` — Phase 1/2 工程史、并行度基准、常驻 worker 否决（负结果参考）。
  - `PROFILING_TASK_PROMPT.md` — 本轮剖析的原始任务书。

## 复现要点（踩过的坑）
- 图可**本地**重渲：`python3 -m venv v && v/bin/pip install matplotlib`，再
  `v/bin/python scripts/prof_plots.py --indir <放好raw csv的目录> --outdir out`（无需容器）。
- py-spy 火焰图：必须选 `comm=python3` 的真 PID（`pgrep -f` 会误中 tmux/bash 包装进程）；
  本容器 `/usr/bin/python3.10` 上 `--native` 失效，用 `--idle` 纯 Python 采样 + `pidstat -t` 补 GIL 证据。
- 高 PAR 需 `docker run --shm-size=16g`（默认 64MB `/dev/shm` 会被 FastDDS 段耗尽 → ~40% 失败）。
- 并行时 ROS 域号须全局唯一（`DBASE+slot`，且 `DBASE+n ≤ 232` FastDDS 上限），否则 tmux 会话名撞车。
