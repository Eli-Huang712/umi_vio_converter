# 任务：tinynav UMI VIO 转换的 CPU/GPU 性能剖析与上限探索

你要对一条 GPU VIO 转换流水线做**系统性的 CPU + GPU profiling（含可视化）**，定位性能瓶颈、量化理论上限、给出可落地的优化建议。这是一个已有大量前置结论的任务——**先读 §2「已知事实」，不要重复已完成的工作**。目标是把"瓶颈在哪、天花板多高、怎么突破"用**数据 + 图**讲清楚。

---

## 1. 环境与访问

- **节点**：`ssh jhhuang-h200-qinghua-1`（别名 h1；8×NVIDIA H200 143GB + 192 CPU 核）。
- **代码在容器里**：`docker exec tinynav_flatbuffer bash -lc "..."`（镜像 `tinynav_flatbuffer_saved`）。
- **路径映射（易错）**：容器 `/data/X` == 主机 `/data/shared/datasets/X`（== 主机 `/home/yuchi/shared/datasets/X`）。所有容器内命令用**容器视角** `/data/...`。
- **待剖析的流水线（单集）**：
  ```
  build_map_node ×(left_wrist, right_wrist)  ← 每腕 = perception_node + build_map_node 两个 ROS 进程，经 topic 通信、定速回放
                                              → 出 <sensor>_db/poses.npy
  merge_sqlite_mcap.py                        → 合并出 *_with_pose.mcap（带触觉）
  ```
- **单集入口（已部署，直接可跑）**：容器内 `bash /tinynav/tool/umi/umi_vio_converter.sh <episode.mcap> --mode full --build-sequential --ros-domain-id <N>`。跑单腕用 `/tinynav/tool/umi/_build_map_one_sensor.sh <mcap> <sensor> <domain> <play_rate>`。
- **测试数据（raw 集，5021 个）**：容器 `/data/plant_collection/raw/26-06-23/dataloop-umi/184/<hash>/episode.mcap`。**务必先 `cp` 到 scratch 目录再跑**（见 §6 约束）。
- **python**：容器内 `python3` = `/opt/venv/bin/python3`，**必须先 source ROS** 才有 rclpy/gtsam/mcap/rosbag2_py：
  ```bash
  for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
           /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
    [ -f "$f" ] && source "$f"; done
  export PYTHONPATH=/tinynav:$PYTHONPATH
  ```

---

## 2. 已知事实（**别重做，从这里往前推**）

前置报告：`datasets/umi_vio_converter/提速测试报告.md`（读 §2 全部）。前置脚本：`datasets/umi_vio_converter/profile_one.sh`（单集 footprint 探针，直接复用/扩展）。

已用可靠方法（cgroup v2 `cpu.stat` 的 `usage_usec` 前后取差）测得，并已交叉验证：

| 指标 | 值 | 来源/口径 |
|---|---|---|
| 单集墙钟（2 腕串行, 1 GPU, 空闲节点） | ~77s | 基线 |
| 每集 build_map 内真实建图计算 | ~1.6s | build_map 日志 "Grand total" |
| 每集逐帧感知（~170 帧 × 60–94ms） | ~15s/腕 | 感知日志 |
| **每集 CPU 核-秒 C** | **234.7** | cgroup `cpu.stat`（真实 busy） |
| 平均占用核数 (C/wall) | 3.13 | 感知轻度多线程 |
| I/O | 88MB 读 / 249MB 写 每集 | cgroup `io.stat` |
| 并行吞吐扫描（PAR=8/24/32/48/64） | 275/714/883/1077/1188 集/时 | 8 卡压测，0 失败 |
| 边际增益归零 → 观测平台 | ~1190 集/时（PAR≈64 达顶） | 边际外推 |

**已证的两条 roofline 结论（你要在此基础上深挖，而不是重复）：**
- **CPU-计算上限 = 192×3600/234.7 = 2945 集/时；GPU-计算上限 ≥ 2292 集/时；显存从不瓶颈（峰值 31/143GB）。**
- **观测平台 1190 只有计算上限的 40–52% → 既非 CPU 也非 GPU 计算硬墙 → 1190 是"软性瓶颈"，存在优化空间。**
- ⚠️ **loadavg 与 nvidia-smi `utilization.gpu` 都不能证明饱和**（前者含 D 态等待，后者是 kernel 驻留率非算力吞吐）。**不要用它们下饱和结论**，要用 cgroup CPU-busy、mpstat `%idle`、以及真实 GPU 引擎活跃度（DCGM `PROF_GR_ENGINE_ACTIVE` 或 Nsight，见 §4）。

**头号待验证假设**：软瓶颈的主因是**多进程共享单卡的 GPU-context 时间片串行化**——PAR=64 时每卡 8 进程，各自 kernel 挤过同一 CUDA context 串行执行，聚合 GPU 计算仅 ~50% 却单进程延迟从 77s 翻到 193s。**这条要用 GPU 时间线证据证实/证伪，并量化。**

**已知会踩的坑（务必规避）：**
- `ROS_DOMAIN_ID ≤ 232`（FastDDS 硬上限）：并行时 `域基址 + 并行数 ≤ 232`。
- **tmux pane 不继承 `docker exec` 的环境**：`CUDA_VISIBLE_DEVICES` 必须在 tmux 命令里重新 export，否则并行全挤 GPU0（`_build_map_one_sensor.sh` 已处理）。
- `build_map` 的 `wait_for_perception_subscribers` **无超时**：感知一崩就无限占卡——所有 build_map 调用**必须包 `timeout`**（脚本已有 `UMI_BUILD_MAP_TIMEOUT_SEC`，默认 300s）。
- **VIO 逐次不可复现**（定速回放+异步感知时序敏感，同集重跑 poses.npy 会变、关键帧数会变）→ **所有耗时/占用测量取多次中位数**，别只跑一次。

---

## 3. 目标（3 个必答问题）

1. **瓶颈到底是什么？** 用 CPU 时间线 + GPU 时间线证据，定位 1190 平台的真实成因（验证/证伪 GPU-context 串行化假设，并找出次要因素：进程 spawn/DDS 发现/模型加载/IO/GIL 串行段）。
2. **真实上限多高？** 把 roofline 补全（含真实 GPU 引擎活跃度、IO 带宽墙），给出"若消除已定位瓶颈，吞吐能到多少"的定量估计。
3. **怎么突破、收益多少？** 给出按"收益/风险/工作量"排序的优化清单，每项带**实测或有依据的预期收益**。**至少要对 NVIDIA MPS 做一次 A/B 实测**（它零代码、直冲 GPU-context 串行化，是头号候选）。

---

## 4. Profiling 计划（分层，每层都要产出数据 + 图）

> 先 `which nsys ncu dcgmi py-spy perf austin` 探明工具可用性；容器可能无外网，缺的记录下来并用替代方案。已确认容器**有** `mpstat pidstat`、python `psutil`；**无** `dcgmi`、无 py-spy 模块。`nsys`/`perf` 未知，先查。

### 4A. 端到端单集足迹（复用并扩展 `profile_one.sh`）
- 逐**阶段**拆时间：perception 启动/模型加载 → 首帧 → 逐帧稳态 → mapping 保存 → merge。用日志时间戳 + 探针。
- 逐资源：cgroup CPU 核-秒（已有 C=234.7，复核）、cgroup IO 字节、峰值 RSS。
- **取 ≥3 集 × ≥3 次**的中位数（VIO 非确定性）。

### 4B. CPU 剖析（定位 CPU 时间花在哪、是否串行）
- `mpstat -P ALL 1`：看是"少数核 100%"（串行瓶颈信号）还是"多核均摊"。
- `pidstat -t -u 1`：**per-thread** %CPU，找热线程 → GIL 串行化信号。
- **Python 火焰图**：优先 `py-spy record`（采样、无需改码、低开销）；无则装（`pip install py-spy` 或 austin），仍无则 `perf record -g` + FlameGraph（能连 gtsam/CUDA native 栈，需 `perf_event_paranoid` 允许）；最后才退 cProfile（会改时序）。产出**火焰图 SVG/PNG**。
- 目标：回答"逐帧感知 ~15s 里，CPU 上花的是特征提取/匹配预处理/gtsam 优化/序列化里的哪块，是否卡在单线程"。

### 4C. GPU 剖析（**核心**，验证 GPU-context 串行化）
- **真实引擎活跃度**（取代 nvidia-smi util）：优先 DCGM `dcgmi dmon -e 1002,1003,1005`（SM active / SM occupancy / mem BW）；无 dcgmi 则 `nsys` 的 GPU metrics，或 `nvidia-smi dmon -s ucm`（sm%/mem%）高频采样。
- **Nsight Systems 时间线（金标准）**：`nsys profile` 抓 CUDA API + kernel + 多进程时间线。**关键实验**：在**同一张卡上并发跑 2–4 个转换进程**，看 nsys 时间线里不同进程的 kernel 是否**串行错开**（证实 context 时间片）还是并发重叠。这是验证假设的决定性证据。产出**时间线截图**。
- 单进程 GPU 占空比：单集里 GPU 真正在算的时间 vs 空等（等 CPU 喂数据）的时间——判断是"GPU 喂不饱"还是"GPU 被串行化"。

### 4D. 饱和扫描（补全上限曲线）
- PAR = 1, 8, 16, 24, 32, 48, 64, **80, 96**（前 5 档已有，重点补 80/96 确认平台/回退）。**需要相对空闲的节点**（见 §6）。
- 每档同时记录：吞吐（集/时）、真实 CPU %busy（mpstat `%idle`→0?）、真实 GPU 引擎活跃度（DCGM/nsys，→100%?）、cgroup 核-秒、峰值 IO 带宽、失败数。
- 目的：用**真实**利用率确认平台成因，并看 PAR>64 是否还有零头或已回退。

### 4E. MPS A/B（头号优化的实测，必做）
- 开 **NVIDIA MPS**（`nvidia-cuda-mps-control -d`），同卡多进程共享一个 context。
- A/B：固定每卡 N 进程（如 4 或 8），MPS 关 vs 开，各测吞吐 + GPU 引擎活跃度 + 单集延迟。
- 结论：MPS 能否把"聚合 GPU 计算 ~50%"推高、把吞吐往 2292 上限推。给出实测收益 %。

---

## 5. 可视化要求（都要出图，PNG/SVG + 简短解读）

1. **Roofline 图**：横轴无所谓，标出 CPU-计算上限(2945)、GPU-计算上限(≥2292)、IO 带宽墙、观测平台(1190)——一眼看清"离哪个墙最近、还有多少余量"。
2. **吞吐 vs PAR 曲线** + 边际增益副轴（含新测的 80/96）。
3. **利用率 vs PAR**：真实 CPU %busy 与 真实 GPU 引擎活跃度双线（证明谁先/是否饱和）。
4. **单集阶段甘特图/时间线**：模型加载 / 逐帧感知 / mapping / merge 的时间占比。
5. **同卡多进程 GPU 时间线**（Nsight 截图）：展示 kernel 是否串行错开（假设的决定性图）。
6. **CPU 火焰图**（4B）。
7. **MPS A/B 柱状图**：关/开 的吞吐 + GPU 活跃度对比。

用 matplotlib/plotly 出图；`nsys` 时间线用 Nsight GUI 导出或截图。图存 `datasets/umi_vio_converter/profiling_out/`。

---

## 6. 硬约束（**必须遵守**）

- **不改 VIO 计算**：`perception_node.py` / `build_map_node.py` / `models_trt.py` 一律不动；只做观测与外部编排。
- **绝不碰真实产物**：测试一律用 raw 集的 `cp` scratch 副本（如 `/data/prof_scratch/`）。5021 个 `*_with_pose.mcap` 及其 `*.pre_tactile.bak` 备份是珍贵数据，只读。
- **共享节点礼仪**：先 `nvidia-smi` 看占用。**GPU0 有别人的 picpp 服务（~12GB）、可能有别的用户 8 卡满载的作业**——**避开被占的卡**，只用空闲卡；饱和扫描（PAR 80/96）需要多卡空闲，**没条件就等、或降规模并注明**，不要抢占别人在跑的 GPU。
- **超时兜底**：所有 build_map 调用包 `timeout`（防无限占卡）。
- **收尾清理**：跑完 kill 所有 perception_node/build_map_node/tmux 会话、删 scratch、关掉自己开的 MPS daemon；确认 GPU 归还。
- **别用 loadavg / nvidia-smi util 下饱和结论**（§2 已说明为何无效）。
- VIO 非确定性 → 多次取中位数，别单点下结论。

---

## 7. 交付物

1. **剖析报告**（markdown，中文）：回答 §3 三问，含 roofline 补全、瓶颈定位的证据链、按收益/风险/工作量排序的优化清单（每项带预期收益，MPS 项带实测数）。
2. **全部图**（§5，7 类）+ 原始采样数据（csv/json）。
3. **可复现脚本**（放 `datasets/umi_vio_converter/`，命名 `prof_*`），别人能一键重跑。
4. **更新** `提速测试报告.md`：把新的实测（真实 GPU 活跃度、PAR 80/96、MPS A/B、火焰图结论）并进 §2.5.1「待补实验」，并据结果更新结论。

## 8. 完成定义（DoD）

- GPU-context 串行化假设被**时间线证据**证实或证伪，且给出定量占比。
- roofline 四条线（CPU/GPU/IO/观测）都有实测数、并指明真正的绑定资源。
- MPS A/B 有实测吞吐差。
- 优化清单每项有依据的预期收益，Top1 有实测或强证据。
- 所有图产出、脚本可复现、环境清理干净、真实产物零改动。

---

### 附：快速自检（验证你的环境搭对了）
先跑一集 `profile_one.sh <scratch_copy> <空闲GPU>`，应得到 wall ≈ 77s、C ≈ 235 核-秒。对不上就先排查环境（多半是 ROS 没 source 或用错 GPU），再往下做。
