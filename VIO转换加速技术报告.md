# tinynav UMI VIO 转换加速技术报告

> **对象**：tinynav UMI VIO 转换流水线（raw `.mcap` → `*_with_pose.mcap`，为每集机器人数据补出双腕位姿轨迹）。
> **问题**：单节点批量转换吞吐被卡在一个远低于硬件上限的平台，需查明真因、量化天花板、并在**不改 VIO 数学、不损坏科研位姿、不浪费资源**的前提下突破。
> **方法**：系统化 CPU/GPU 剖析（cgroup 核-秒、mpstat `%idle`、nvidia-smi dmon `sm%` 与 10Hz `util.gpu`、同卡多进程扫描、py-spy 火焰图、NVIDIA MPS A/B）+ 两轮"诊断→优化→再诊断"闭环，全程轨迹级/通道级正确性闸门把关。
> **节点**：H200（8×NVIDIA H200 143GB + 192 CPU 核）。**实测日期**：2026-07-08。
> **代码**：`feature/p1-direct-feed`（P1）、`feature/p2-max-throughput`（P2）。本文合并原《VIO转换瓶颈分析报告》与《P1 实现说明》，并补入 P2 全部结果。

---

## 0. 摘要

从 stock 到最终配置，单节点吞吐提升 **3.25×**：

| 阶段 | 配置 | 吞吐（帧/时）※ | 相对 stock | 关键机理 |
|---|---|---:|---:|---|
| 基线 | stock 双进程 + DDS | **600K** | 1.0× | build_map 单线程 ROS 序列化，GIL 钉 ~1 核 |
| **P1** | 进程内同步直喂 | **1.61M** | **2.69×** | 删除 CDR 序列化 + DDS + executor，CPU/GPU 同时被解放 |
| **P2** | 直喂 + 流水化解码 + MPS | **1.95M** | **3.25×** | 抬高单卡 GPU 占空比、并发 kernel，单卡有效 GPU util >80% |

※ 帧/时 = size-invariant 吞吐口径（见 §2.3）；同一 128 集 scratch 池、8 卡空闲、稳态窗口、0 失败。

**一条贯穿全程的逻辑主线**——**每一次优化都会移动瓶颈的位置**，因此结论必须按"当时的瓶颈在哪"来读，否则会显得自相矛盾：

- **MPS**：在 stage 1（瓶颈在 CPU、GPU 空闲 7%）实测 **0% 收益、被否决**；在 stage 2（P1 把瓶颈推到 GPU）实测**单卡 +52%、被采纳**。同一手段，两次相反结论，因为中间瓶颈迁移了。
- **GPU-context 串行化**：stage 1 **被证伪**（同卡 N 扫描线性、MPS 0%）；stage 2 **被证实**（同卡 N 扫描在 6–8 饱和、MPS +52%）。同理。
- **常驻/热进程池**：stage 1 因 native 状态泄漏"静默损坏位姿"**被否决**；P2 用"每集全新 node"消除了泄漏、通过逐位正确性闸门，但因节点已 GPU-bound（省下的 CPU 无处可用）+ 高并发下可靠性下降，**仍被否决**——这次是吞吐与可靠性理由，不是正确性。

全程零 VIO 数学改动、真实产物零改动，正确性以轨迹级 ATE/RPE + 通道语义 parity 双闸门把关。

---

## 1. 流水线解剖：一集数据是怎么被转换的

每个 raw 集含双腕 + 头部相机的 H.264 视频、IMU、夹爪、触觉。VIO 转换要为**两只手腕**各跑一遍视觉惯性里程计（VIO），产出位姿轨迹，再合并回一个带位姿的 mcap。

```
一集 raw.mcap
   ├── build_map(left_wrist)  ┐  每腕 = 两个 ROS 进程：
   │                          │    perception_node ── (ROS topics/DDS) ──> build_map_node
   ├── build_map(right_wrist) ┘    经 BagPlayer 定速回放(play_rate=20)驱动，逐帧处理
   └── merge_sqlite_mcap ──> episode_with_pose.mcap（回插位姿 + 触觉）
```

**单腕内部的 stock 两进程架构**（理解 stage-1 瓶颈的关键）：
- **`perception_node`**：加载 TensorRT 模型（SuperPoint 特征点 + LightGlue 匹配 + Retinify 立体深度 + Dinov2 检索），逐帧做特征/匹配/深度/IMU 预积分/因子图。**多线程**（TRT 推理 + asyncio 立体 worker）。
- **`build_map_node`**：用 `BagPlayer` 定速回放原始 bag，把每帧图像/IMU 序列化成 `sensor_msgs/Image` 经 ROS topic 喂给感知；收回关键帧结果做位姿图/回环/occupancy。
- 两者经 ROS2 DDS（FastDDS，默认走 `/dev/shm` 共享内存传输）通信。

**三个源码事实**决定了整条正确性论证的边界（P1/P2 全程依赖）：
1. `merge_sqlite_mcap` **只读**每个 `*_db/poses.npy`；触觉/IMU/夹爪/camera_info 通道由 merge 从 raw bag 原样透传，**与 VIO 路径无关**。→ 正确性闸门 = 复现 perception 的关键帧位姿。
2. `build_map.solve_pose_graph` 实为 **no-op**（真解算被注释，`build_map_node.py:186-187`）→ `poses.npy` = perception 关键帧里程计按时间戳收集排序。
3. UMI flatbuffer bag **无 `/tf`**，merge 忽略 `T_rgb_to_infra1`/连续里程计/occupancy → 最小正确路径很小。

**单腕耗时拆解**（实测，一集中位）：

| 阶段 | 耗时 | 说明 |
|---|---|---|
| 进程 spawn + CUDA/ROS 初始化 + DDS 发现 | ~1s | 固定开销 |
| TRT 模型加载（3 个 .plan 反序列化） | ~1s | 每进程重付 |
| **逐帧感知（定速回放 + 特征/匹配/深度/SLAM）** | **~29s** | **主导项**，228 帧 × ~127ms/帧 |
| └─ 其中真正的建图数学（位姿图/回环/occupancy） | 仅 1.8s | profiler "Grand total" |

**两个贯穿全程的关键观察：**
1. **逐帧感知（29s）主导，固定开销（~2s）与建图数学（1.8s）都极小。** 任何"摊销启动开销"的路线天花板都极低——这是常驻 worker 天花板仅 ~15% 的根因（§7.3）。
2. **VIO 输出非逐次可复现。** 定速回放 + 异步多线程感知对调度抖动敏感，`ApproximateTimeSynchronizer` 在不同时序下把 stereo/IMU 配成不同对，导致同集重跑得到不同关键帧数与位姿。**"逐字节一致"因此不能作为任何提速方案的验收闸门**——正确性必须用轨迹级容差（§5.3）。

---

## 2. 方法论：如何正确地测量

三条方法学原则贯穿两轮剖析，是所有结论可信的前提。

### 2.1 只用能证明饱和的指标
初版分析曾据 loadavg=177/192 与 `nvidia-smi utilization.gpu`=80% 断言"CPU 已饱和"。**这两个指标都不能证明饱和**：
- **loadavg** 含不可中断（D 态）睡眠线程——等 GPU/磁盘/DDS 的进程也计入，却不占 CPU 核。
- **`utilization.gpu`** 是"有 ≥1 个 kernel 驻留的时间占比"，132 个 SM 里跑 1 个也读作 100%。

本报告一律用 **cgroup v2 `cpu.stat` 核-秒 + mpstat `%idle`**（真实 CPU busy）与 **dmon `sm%`**（真实引擎活跃度）。

### 2.2 `sm%` 在 MPS 下会低估——需要 10Hz `util.gpu` 补正（P2 新增）
`dmon sm%`（1Hz）在 MPS 下**系统性低估**有效利用率：MPS 把并发 kernel 压成更短更密的突发，更少的采样区间显示活动，**却做了更多功**。P2 实测：同 PAR 下无 MPS 的 sm% 反而高于开 MPS（63% vs 52%），而开 MPS 吞吐更高。因此 P2 的 ">80% util" 结论用 **`nvidia-smi --query-gpu=utilization.gpu -lms 100`（10Hz）在单张持续喂满的卡上测**，而非节点聚合 sm%。**真正的效率信号始终是吞吐（帧/时）。**

### 2.3 吞吐单位用"帧/时"而非"集/时"
UMI 每集时长基本恒定（~11s），但**帧数随内容在 73–461 帧/腕间变化**（中位 228）。真实计算量随帧数走、与集大小无关，故"集/时"被集大小分布干扰，**"帧/时"才是尺寸无关的物理量**（帧数 = 双腕 `left_h264/video` 消息数）。为衔接历史数据，关键处并列集/时（中位集 ≈ 456 帧 = 双腕）。

### 2.4 稳态窗口 + 每单元看门狗（P2 定型）
整批 wall-time（xargs 两波）会被 drain 尾巴严重污染，且 stock 的 `wait_for_perception_subscribers` **无超时**（`build_map_node.py:610`），高 PAR 下 DDS 握手偶发挂死会拖垮整批（首测得 219K 假值）。定型的测法：**连续队列保持恰好 PAR 个单元在飞 + 掐头去尾稳态窗口 + 每单元 `timeout` 看门狗**（含按确定性会话名回收泄漏的 tmux pane）。

### 2.5 零改动原则（正确性护栏）
`perception_node.py` / `build_map_node.py` / `models_trt.py`（VIO 数学）与 `merge_sqlite_mcap.py` 全程**一行未改**；测试**全用 raw 集的 scratch 副本**，未触碰任何真实产物。

---

## 3. Stage 1 诊断：吞吐平台在哪、为什么它不该存在

### 3.1 并行度扫描：平台是 PAR≈64、约 1196 集/时（54.5 万帧/时）

8 卡空闲节点上做 PAR（同时转换的集数）扫描，每档测真实利用率：

| PAR | 集/时 | 万帧/时 | 真实 CPU-busy | GPU sm% | 失败 |
|---:|---:|---:|---:|---:|---:|
| 8  | 275  | 12.5 | ~8% | ~2% | 0 |
| 32 | 829  | 37.8 | 33% | 5% | 0 |
| **64** | **1196** | **54.5** | **50%** | **7%** | 0 |
| 80 | 953 ↓ | 43.5 | 48% | 7% | 3 |
| 96 | 936 ↓ | 42.7 | 51% | 7% | 4 |

**吞吐在 PAR=64 见顶，PAR=80/96 不升反降（−20%）且开始崩溃。**

### 3.2 平台不该存在：四条 roofline 无一被撞到

| 资源 | 上限 | 口径（全部实测） |
|---|---:|---|
| **观测平台** | **1196 集/时** | PAR=64 实测峰 |
| CPU 计算 | 2949 集/时 | 192 核 × 3600 / **234 核-秒/集** |
| GPU 计算 | ~24000 集/时 | 8 卡 × 3600 / **1.2 GPU-秒/集** |
| IO 带宽 | ~66500 集/时 | 4.6 GB/s 盘写 × 3600 / 249MB 写/集 |
| 显存 | 从不瓶颈 | 峰值 ~20/143 GB（每进程仅 2.4GB 模型常驻） |

**观测平台只有最近那堵墙（CPU 计算）的 41%，离 GPU/IO 一两个数量级。** 没有任何硬墙被撞到，可吞吐就是上不去、单集延迟还随 PAR 一路膨胀（78s→193s）。存在一堵**限制每进程效率、而非任何全局资源总量**的"软墙"。

### 3.3 追凶：三个假设，逐一决定性证伪/证实

**假设 A：GPU-context 时间片串行化（初版头号嫌疑）——证伪。**
决定性实验（同卡多进程，N≤8 时总共只用 ≤25/192 核，CPU 不可能是混淆项）：

| N（同卡进程） | 整组墙钟 | 每进程延迟 | 吞吐(腕/时) | GPU sm% |
|---:|---:|---:|---:|---:|
| 1 | 34.3s | 34.3s | 105 | 2% |
| 2 | 67.2s | 50.9s | 107 | 2% |
| 8 | 68.4s | 48.1s | **421** | 7% |

从 N=2 到 8，整组墙钟几乎不变（~68s）而吞吐近线性涨（107→421，~4×）——**一张卡能从容吸收 ~8 个进程，GPU 不是墙**。**钉死它——MPS A/B**（N=8 同卡关/开 MPS）：**421→420，收益 −0.2%（统计为零）**。MPS 是消除 context 争用的专用工具，零提升 → 根本没有争用可消除（sm% 才 7%，卡是空的）→ 假设 A 被证伪。

**假设 B：`/dev/shm` / DDS 段耗尽——次要，零成本修复。**
容器默认 `/dev/shm`=64MB，PAR≥32（~64+ DDS participant）时耗尽 → `RTPS_TRANSPORT_SHM: Failed to create segment`，~40% 集失败。生产驱动靠自动重试掩盖（白烧算力）。修复零成本：`docker run --shm-size=16g`（宿主 2TB RAM），修复后 PAR=32 时 `/dev/shm` 仅用 1%、零失败。**本报告所有干净数据都在修复后测得。** 它也解释了 PAR>64 的崩溃（DDS 握手不稳）。

**假设 C：每进程单线程 CPU 关键路径——真凶。**
排除 GPU、把 DDS 归为次要后，瓶颈只能在单进程关键路径。py-spy 火焰图 + `pidstat -t`：

*build_map 火焰图热点：*
- **`sensor_msgs/msg/_image.py:267-268` 的 `Image.data` getter + 生成器表达式 = 34%+29%** —— 每条图像 ROS 消息在**纯 Python 里逐字节拷贝** CDR (反)序列化。
- rclpy executor spin_once = 17%；flatbuffer 解码 = 11%。

*每线程 GIL 证据：*

| 进程 | 进程 %CPU | 最忙线程 | top/sum |
|---|---:|---:|---:|
| **build_map** | 89.5% | 68% | **0.92** → 彻底 GIL 串行，钉 ~1 核 |
| perception | 97.5% | 4.5% | 0.06 → 真多线程，非瓶颈 |

**真凶确认：`build_map` 进程的单线程 ROS2 图像消息序列化**——每帧在 GIL 保护的纯 Python 里逐字节搬一遍图像 + rclpy 事件循环，占 build_map ~63%，**无法靠多核并行**。

### 3.4 一个机理解释所有现象

| 现象 | 机理 |
|---|---|
| CPU 只 50%、PAR=64 见顶 | 每 build_map ≈ 1 核（GIL），~128 个 build_map 填满 ~64 有效调度空间，远不到 192 核 |
| GPU 仅 7% | build_map 几乎不碰 GPU（在搬消息）；perception 用 GPU 但被上游喂数据速度限制 |
| MPS 零收益 | 无 GPU-context 争用可消除——瓶颈在 CPU 侧消息路径 |
| PAR>64 回退 | 过订使每进程延迟膨胀超过并行收益 + DDS 握手不稳崩溃 |

**Stage-1 绑定资源 = "每进程单线程消息搬运"这条效率软墙，而非任何资源总量。** 这解释了横向加核为何无效。→ 突破方向唯一：**削减/删除这条序列化路径本身**。

---

## 4. P1：进程内同步直喂（direct-feed）

### 4.1 改了什么
把 build_map 的喂帧路径从"`BagPlayer` 定速回放 → 序列化成 `sensor_msgs/Image` → 过 DDS → perception 反序列化 → `ApproximateTimeSynchronizer` 异步配对"改成**一个进程内、把解码后的 numpy 帧点对点同步喂给 perception + build_map**，直接删除火焰图里那 63% 的 CDR 序列化 + rclpy executor。**VIO 数学零改动**——两个 node 原样 import，只改"数据怎么进来"。

### 4.2 实现（`direct_feed_build_map.py`）
- **`read_episode`**：用 stock 同款原语（`flatbuffer_codec.decode_*` + PyAV H.264）从 raw bag 解出 numpy 帧、旋转后的 `Imu`、合成 `CameraInfo`——全程不序列化。
- **`ShimBridge` + `_NpImage`**：替换两个 node 的 `self.bridge`，`imgmsg_to_cv2` 直接返回 numpy（仅在 encoding 不同时做 cv2 转换）——**零 CDR、零逐字节拷贝**。
- **publisher 替换**：perception 的 keyframe pub 捕获到 holder，其余 pub 与 TF broadcaster 换 no-op（消息**构建**仍跑，VIO 行为不变；只砍 DDS 序列化）。
- **驱动主循环**：喂一次 `CameraInfo` 设 K/baseline，再按**严格时间戳序**喂 IMU + stereo；左右目最近邻配对（slop 0.02s；硬件同步 ~33µs，确定性）；复刻 perception 的 7.5Hz 节流；**IMU 1 步前瞻**（喂 `ts≤T` 全部 + 第一条 `ts>T`，复刻 stock 异步 deque 状态）；出关键帧就同步喂 build_map。
- 编排层：`_build_map_one_sensor.sh` 加 `UMI_DIRECT_FEED=1` 分支（单进程、无 tmux、`timeout` 看门狗、隔离 `ROS_DOMAIN_ID`），stock 双进程路径仍为默认。

### 4.3 吞吐收益

**足迹口径（`prof_footprint_ab.sh`，同集独占、cgroup 核-秒）：**

| 集 | | stock | direct-feed | 变化 |
|---|---|---:|---:|---:|
| ep0(134fr/腕) | 墙钟(双腕) | 67.9s | 20.9s | −69%(3.25×) |
| | CPU 核-秒 | 113.9 | 73.4 | −36% |
| | 平均占用核 | 1.68 | 3.51 | 摆脱 GIL 单核 |
| ep2(327fr/腕) | 墙钟(双腕) | 140.6s | 38.0s | −73%(3.7×) |
| | CPU 核-秒 | 271.9 | 180.1 | −34% |
| | 平均占用核 | 1.93 | 4.74 | 摆脱 GIL 单核 |

**PAR 稳态口径（`prof_steady_frames.sh`，128 集池、帧/时、0 失败）：**

| PAR | 模式 | 帧/时 | CPU | GPU sm% | vs stock@64 |
|---:|---|---:|---:|---:|---:|
| 64 | stock | 600,440 | 33% | 5% | 基准 |
| 48 | df | 1,533,500 | 56% | 60% | +155% |
| **64** | **df（峰值）** | **1,612,520** | 57% | 61% | **+169%（2.69×）** |
| 96 | df | 1,121,220 ↓ | 57% | 56% | +87% |

**为何 +169% ≫ 足迹预测的 +51~55%**：足迹只算"CPU 一堵墙右移"（核-秒下界）。但 direct-feed **同时解放了 GPU**——stock 被 GIL 序列化钉住，喂不动 GPU（PAR=64 时 CPU 33% + GPU 5% **两轴同时闲置**）；direct-feed 喂帧够快，把 GPU 拉到 60%、CPU 拉到 57%，**一次吃下两个轴的余量**。`平均占用核 1.9→4.7` 是 GIL 软墙消失的机理直证。

### 4.4 正确性

**轨迹级 ATE/RPE（`prof_compare_poses.py`，Umeyama 对齐后平移 RMSE）**，3 集×2 腕：

| 集/腕 | stock 自抖动 | df 逐次 determinism | **df vs stock ATE (max)** |
|---|---:|---:|---:|
| ep0/1/2 left | 0.000mm | **0.000mm** | **0.28–0.46mm** (≤1.6mm) |
| ep1 right | 0.000mm | **0.000mm** | 0.66mm (2.9mm) |
| ep0/ep2 right | 0.000mm | **0.000mm** | **~13mm** (42–71mm) |

三点结论 + 一个团队决策项：
1. **direct-feed 逐次完全确定**（重跑 0.000mm）——顺带修了 stock 的时序非确定性（红利）。
2. **4/6 亚毫米**，与 stock 不可区分——证明喂帧路径的 VIO 数学忠实（有系统 bug 会 6 个全偏）。
3. **2/6 动态 right_wrist 偏 ~13mm**（§4.5），非 bug。
4. **决策项**：13mm 是否落在下游训练可接受的 VIO 精度内，需团队判定（见 §5.4）。

**通道 parity（`prof_channel_parity.py`）**：`*_with_pose.mcap` 的触觉×4/IMU×3/夹爪×2/camera_info×6/video×6 **全部语义一致**，仅位姿话题按预期不同。闸门自证：stock-vs-stock 对照 = `PARITY_OK`。教训：**不能用逐字节 hash**——FastCDR padding 来自未清零缓冲，同一消息逐次序列化出不同字节（stock 对自己都 FAIL）；正确闸门是反序列化后字段值。

### 4.5 ~13mm 偏差的机理（非 bug）
stock 的 `stereo_queue(maxsize=1)` 在 worker 忙时**丢帧**（process ≈138ms/帧 ≈ 133ms 节流间隔，边缘态偶发）；direct-feed **不丢**——处理每个过节流的帧。故 **df 处理的帧是 stock 的严格超集**（每个偏差集都是 `a_only=2, b_only=0`：stock 关键帧全在 df 内，df 多 2 个）。这 2 帧改变滑窗 ISAM（`_N=10`）与 PnP 链，使 common 关键帧也重优化出不同解；平缓 left_wrist 亚毫米，动态 right_wrist 达 ~13mm。这正是任务书明确允许的"同步喂帧得到*不同但确定*的轨迹"，且 direct-feed **更"对"**（不丢帧、用满信息）。

---

## 5. Stage 2 诊断：瓶颈迁移到了 GPU

### 5.1 新谜题
P1 后 PAR=64 峰值处 **CPU 57%、GPU sm 60% 皆不饱和**，为何仍撞墙？补装 py-spy（离线 wheel）+ 单卡 N 扫描 + MPS A/B 重新定位。**这一步是全报告逻辑最关键处：stock 的瓶颈（CPU/GIL 单线程）已被 P1 消除，瓶颈迁移到了 GPU 侧——于是 stage-1 被证伪的假设，在 stage-2 需要重新检验。**

### 5.2 主约束（近因）= 单卡多进程 CUDA 上下文时间片串行化——这次证实
决定性数字（同卡 N=8 MPS A/B）：**开 MPS 后每进程延迟 30.4s→19.3s（回落到孤立单进程的 19.7s，共租惩罚被完全抹除），同卡吞吐 756→1147（+52%），而 sm 反降 73%→54%。** MPS 只改跨进程 GPU 上下文共享，它在降低占空比的同时提速 +52%，只有当"73% 是被串行化摊薄的低占用空转、而非算力饱和"时才可能。单卡 N 扫描佐证：N=1→8 边际吞吐 +165/+223/+135/+44 递减、N=8 墙钟 +56%、sm 只到 73% 就封顶——**留有余量却死掉 = 调度串行化而非算力墙**。

> **与 §3.3 假设 A 的对照——这不是矛盾，是瓶颈迁移的直接证据。** stage-1 同卡扫描：墙钟平、吞吐线性、sm 2→7%（GPU 空闲，无争用，MPS 0%）。stage-2 同卡扫描：墙钟膨胀、吞吐饱和、sm 到 73%（GPU 被喂到、有争用，MPS +52%）。**同一实验、相反结果**，因为 P1 把 GPU 从"闲置"变成了"被喂到"。stage-1"GPU 不是墙"与 stage-2"GPU 是墙"都成立——各自对应自己那一刻的瓶颈。

### 5.3 根因（上游）= 整段前置解码造成的低 GPU 占空比
单进程 on-CPU 火焰图：**53.6% = `read_episode` 在任何 VIO 前一次性解码整段 H.264（此间 GPU 全程空闲）**，仅 33% 是 VIO 喂入相位（且被 `run_graph` 的 `while cudaEventQuery: await asyncio.sleep(0)` busy-wait 轮询占据）。因每进程约半条命 GPU 空闲，才需 ~8 路共租才能喂饱一张卡——**这正是制造 8 路共租、触发上下文串行化的成因**。近因（串行化）之上是根因（低占空比）。

### 5.4 次约束 + 三级约束
- **次约束 = 主机侧过订阅**：PAR>64 是**塌陷**而非平台（96=1.12M<PAR48），纯每卡串行化只会缓降。~450 原生线程、瞬时 ~9-10 核/进程，高 PAR 下解码相位相互碰撞。峰值 64 = "GPU 侧填卡收益"与"CPU 侧过订代价"的平衡点。
- **三级**：busy-wait 空转（33% on-CPU，故"57% CPU"高估有用率）+ 每进程固定开销（numba JIT 7.7% 每进程重付、shutdown 挂死）。**非 GIL**（450 原生线程释放 GIL，对比 stock top/sum=0.92）。

**为何两轴都不饱和却撞墙**：低占空比 → 需多路共租填卡 → 共租在 GPU 相位碰撞 → sm~73% 即封单卡（N≈6-8）→ 节点 8×8=PAR64；再堆进程追未填满的余量就触发主机过订 → 回归+失败。**64 是两股力的平衡点，"都没打满"正是"复用效率+占空比联合封顶"的必然读数。** `sm 60% 是时间活动代理非算力饱和`——MPS 让 sm 降到 54% 而吞吐升 +52% 是决定性反证（§2.2）。

---

## 6. P2：逼近上限（流水化解码 + MPS）

目标：把有效 util 推到 >80%，**保正确率、不浪费资源**。§5 指明两个可攻击点——低 GPU 占空比（根因）与上下文串行化（近因），分别用流水化解码与 MPS 攻击。

### 6.1 流水化解码（`StreamingDecoder`）——攻击根因
把"整段前置解码"改成**后台解码线程 + 主线程 VIO 消费**：PyAV/cv2 解码释放 GIL → 解码与 TRT/gtsam 真并行，GPU 持续被喂而非等整段解码。在线配对与批量共用 `_nearest_right`，逐位一致。
- **正确性闸门 PASS**：流水 vs 批量（`--no-pipeline`）**6/6 集×腕逐位一致（maxabs=0）**——只改解码调度、不改任何输出。

### 6.2 MPS——攻击近因
编排层起停（`prof_mps.sh`），无 VIO 改动。单卡 A/B +52%（§5.2）；全节点 PAR=64 +15%（1.61M→1.86M）。**节点增益远小于单卡**，因为 MPS 只解近因（上下文串行化），根因（低占空比 + 主机协调）在节点级重新显形——这正是需要叠加流水化解码的原因。

### 6.3 util 甜点扫描（`prof_util_sweep.sh` + `prof_effective_util.sh`）

| 配置 | 峰值 PAR | 帧/时 | 单卡有效 GPU util(10Hz) | 失败 |
|---|---|---:|---:|---:|
| P1（无流水无 MPS） | 64 | 1,612,520 | — | 0 |
| MPS only | 64 | 1,861,200 | — | 0 |
| 流水（无 MPS） | 64 | 1,856,550 | **96%** | 0 |
| **流水 + MPS** | **48** | **1,951,830** | **86%** | 0 |

- **>80% util 达成**：单卡 10Hz `util.gpu` = 86%(MPS)/96%(无 MPS)。节点 dmon sm%~55% 是 §2.2 的测量假象，非空闲。**GPU 是绑定资源（单卡 ~86-96%），CPU 尚有余量（节点 ~57%）**。
- 流水把最优 PAR 从 64 降到 **48**（每 worker 更肥）——**更少进程、更高 util、0 失败 = 不浪费**。

### 6.4 净收益
**1.61M → 1.95M 帧/时（+21%），单卡有效 GPU util >80%，0 失败，逐位正确**——达成用户全部约束。全程栈：600K(stock) → 1.61M(P1) → 1.95M(P2) = **3.25× over stock**。

---

## 7. 否决方案与负结果（含逻辑反转的完整交代）

### 7.1 NVIDIA MPS：stage-1 否决 → stage-2 采纳
- **stage-1（否决）**：瓶颈在 CPU、GPU 空闲 7%，无 context 争用可消除，A/B 实测 421→420（0%）。
- **stage-2（采纳）**：P1 把瓶颈推到 GPU，单卡 N=8 有争用，A/B +52%，节点 +15%。
- **逻辑**：MPS 的价值取决于"GPU 是否有 context 争用"，而这由当时的瓶颈位置决定。两次结论都对。

### 7.2 GPU-context 串行化假设：stage-1 证伪 → stage-2 证实
同上——同卡 N 扫描 + MPS A/B 在两个阶段给出相反签名，因为 GPU 从"闲置"变"被喂到"。**这提醒：瓶颈分析的每条结论都带"当时瓶颈在哪"的隐含前提，优化后必须重测。**

### 7.3 常驻/热进程池：stage-1 因正确性否决 → P2 修好正确性但仍因吞吐/可靠性否决
- **stage-1 否决理由**：native 状态泄漏（gtsam/CUDA/TRT C++ 侧，关键帧数逐集 +2 单调增长，4 次修复未根除）**静默损坏位姿**；且天花板仅 ~15%（逐帧感知不可摊销）。
- **P2 重审**：`direct_feed_pool.py` 用"只复用进程、每集全新 node+模型"消除泄漏面。**小闸门（3 集连跑）PASS**：池 vs 全新进程逐位一致，warm worker ~6.8s vs fresh ~9.4s（~28% 更快/集）。
- **但节点级仍证伪**：(1) **不抬节点吞吐**（1.84M < 流水+MPS 1.95M）——节点已 GPU-bound，池省下的 CPU 无处可用（warm 提速是延迟收益，不在 GPU-bound 节点兑现为吞吐）；(2) **48 路 + MPS 高并发下可靠性下降**（一集 `IndexError`、一 worker 段错误 rc=139；归因已隔离——该集在全新进程/单 worker 顺序/12 集连跑三种情形全部成功，故属 **MPS 高并发可靠性问题**，非可复现的单进程跨集泄漏）；(3) **失败爆炸半径更大**（worker 崩溃丢整条 shard 5 集，fresh 只丢 1 集）。
- **裁决**：违反"保正确率、不浪费资源"，**池默认关闭、作为负结果存档**。生产用"流水+MPS+每集全新进程"（拿全部收益、失败半径最小）。

### 7.4 其他否决
- **提高 play_rate**：喂更快很可能丢帧、劣化位姿，且 VIO 非确定性使其难以干净验证。
- **横向堆核（单节点内）**：stage-1 已证无效——瓶颈是每进程串行段，不是资源总量。

---

## 8. 最终建议配置与优化清单

| 优先级 | 方案 | 收益 | 状态 | 依据 |
|---|---|---|---|---|
| **P0** | `--shm-size=16g` + 锁 PAR（stock=64 / 流水+MPS=48） | 消除 DDS 失败 + 避免 PAR 过订回退 | 纯编排、零风险 | §3.3-B、§3.1 |
| **P1** | 进程内同步直喂（direct-feed） | +169%（600K→1.61M 帧/时） | **已实现、已验收** | §4 |
| **P2** | 流水化解码 + MPS | 再 +21%（→1.95M），单卡 util >80% | **已实现、已验收** | §6 |
| P3 | 存量集走 BACKFILL 而非 FULL | 存量补触觉纯 CPU、~0.77s/集（快 ~50×） | 已交付 | — |
| ✗ | 常驻/热进程池 | GPU-bound 无吞吐增益 + 高并发可靠性下降 | 证伪、默认关闭 | §7.3 |
| ✗ | ~~MPS 单独用于 stock~~ | 0%（瓶颈不在 GPU） | stage-1 否决 | §7.1 |

**推荐生产配置**：`--shm-size=16g` + direct-feed（`UMI_DIRECT_FEED=1`）+ 流水化解码（`UMI_PIPELINE=1`，默认开）+ MPS + 每集全新进程 + PAR≈48。共享节点上 PAR=48 兼顾峰值吞吐与抗 co-tenant。

**尚待事项**：(1) right_wrist ~13mm 是否落在下游训练可接受精度内（团队判定，§4.5/§5.4）；(2) Nsight 时间线确认"GPU 端 kernel 串行 vs CPU 端 launch/query 串行"（py-spy `--native` 在本容器失效）；(3) 替换 busy-wait spin + 修 shutdown 挂死（释放 ~0.75 核/进程，右移 PAR 塌陷悬崖，廉价附带项）。

---

## 9. 换到更弱显卡上的调参方法（可复现流程，非定值）

> 本报告的一切定值（`PAR=48`、`N≈8/卡`、`--shm-size=16g`）都是 **H200 特有**的。换成更弱的卡（L4/A10/4090/T4…）时这些值会变，但**求它们的流程不变**。以下给出一套一次性、可复现的调参流程——本质就是"在新硬件上重跑一遍 §3/§5 的诊断闭环"，全部复用已有脚本。**不要照搬 48/8**。

### 9.1 唯一核心旋钮：`N*` = 单卡并发进程数

全报告的最优点都由这一个量派生：

```
PAR（节点并发集数）= N* × 卡数
```

`N*` 被两条上限夹逼，**取小者**：

| 上限 | 定义 | H200 实测 | 弱卡预期 |
|---|---|---:|---|
| **显存上限 `N_vram`** | `⌊可用显存 × 0.9 / 每进程峰值显存⌋` | 143GB/~2.4GB ≈ 60（从不瓶颈） | **常成硬约束**（16/24GB 卡 → N_vram=5~8） |
| **算力饱和上限 `N_gpu`** | 同卡 N 扫描中"每进程延迟开始膨胀 / 边际吞吐塌陷"的档 | 6–8 | **明显更小**（可能 2–4，极弱卡=1） |

再加一条对 **PAR** 的主机约束：CPU 过订。在 `PAR = N* × 卡数` 下若节点 CPU-busy > ~85% 或出现失败，回退 PAR。

### 9.2 调参流程（每步对应一个已有脚本）

**Step 0 — 前置**：`docker run --shm-size≥8g`（DDS + pipeline 队列）；部署 direct-feed + pipeline（`UMI_DIRECT_FEED=1 UMI_PIPELINE=1`）。这两项与显卡无关，任何卡都先开。

**Step 1 — 单进程基线**（1 进程 1 卡，`prof_effective_util.sh N=1` 或直接跑 driver）：记录 ①单进程 per-frame VIO 延迟；②`nvidia-smi` 每进程峰值显存。

**Step 2 — 算显存上限**：`N_vram = ⌊可用显存 × 0.9 / Step1的每进程峰值显存⌋`。（弱卡这步往往就把 N* 卡死了。）

**Step 3 — 测算力饱和上限**（关键一步）：`prof_df_contention.sh`（同卡 N=1,2,4,6,8…）。看 `per-proc latency` 与 `marginal throughput`。**判据**：per-proc 延迟相对 N=1 膨胀 **>~20%** 的那一档的**前一档** = `N_gpu`。弱卡这个拐点会明显靠前。

**Step 4 — 定 N* 与 PAR**：`N* = min(N_vram, N_gpu)`；`PAR = N* × 卡数`。

**Step 5 — 主机校验**：在该 PAR 跑 `prof_steady_frames.sh`，看节点 CPU-busy 与失败数。**CPU-busy > ~85% 或有失败** → PAR 逐档下调，取"0 失败且 CPU < ~80%"的最大 PAR。

**Step 6 — MPS 决策**（不是无脑开）：在 `N*` 同卡做 `prof_df_mps_ab.sh`。**+X% 显著且 0 失败 → 开**；**≈0 → 不开**（意味着 `N*≤1`，卡上没有 co-tenancy 可供 MPS 消除——见 §7.1 的 stage-1 情形）。**这一步直接决定 MPS 用不用，不能沿用 H200 的"开"。**

**Step 7 — 正确性闸门**（与显卡无关，测一次即可）：`prof_pipeline_gate.sh`（流水 vs 批量**逐位一致**）+ 抽 3 集×2 腕做 `prof_compare_poses.py`（ATE/RPE 落在 stock 容差内）。

### 9.3 方向性直觉（换卡前的预判，用于设扫描范围）

- **GPU 越弱 → per-frame VIO 越慢 → `N_gpu` 越小 → PAR 越小。** 别从 PAR=48 起扫，从 `N* = 2~4 × 卡数` 起。
- **显存越小 → `N_vram` 越可能成为硬夹逼。** 16GB 卡装不下 8 个 ~2.4GB 模型 + 工作集，N* 先被显存卡死。
- **解码是 CPU 侧、与 GPU 无关** → 弱卡上 GPU 更明确是瓶颈、CPU 余量更大 → **pipeline + MPS 的相对收益更大**（除非 N\*=1）。
- **极弱卡（一进程即饱和一卡，`N*=1`）**：MPS 无益（Step 6 会显示 ≈0）、**pipeline 仍有益**（消解码期 GPU 空窗）、`PAR = 卡数`、系统纯 GPU-compute-bound——此时唯一进一步的路是换更强卡或降模型精度（FP16/INT8，属改 VIO 数学，超本流程范围）。

### 9.4 一句话总则

**先测 `N_gpu`（Step 3）和 `N_vram`（Step 2）、取小得 `N*`、乘卡数得 PAR、用主机校验收口（Step 5）、用 MPS A/B 决定开不开（Step 6）。** direct-feed + pipeline 任何卡都开；`PAR` 与 `MPS 开关`按卡实测——这两个才是随硬件变的量。

---

## 10. 结论

1. **瓶颈是会移动的。** stage-1 真凶 = build_map 单线程 ROS 图像序列化（GIL 钉 ~1 核，63% 时间）；P1 删除它后瓶颈迁移到 GPU 侧（单卡上下文串行化 + 低占空比）。**任何"某手段无效/有害"的结论都只对当时的瓶颈成立**——MPS、GPU-context 串行化、常驻池三者的逻辑反转都源于此。
2. **突破路径始终是"直击每进程效率"而非堆资源。** P1 删序列化、P2 抬占空比+并发 kernel，都在提高单位工作的效率；横向加核/加 PAR 在软墙前无效。
3. **收益**：600K → 1.61M → 1.95M 帧/时 = **3.25× over stock**，单卡有效 GPU util 从 7% 提到 >80%，全程 0 失败、逐位/轨迹级正确、VIO 数学与真实产物零改动。

---

## 附录 A：方法与工具
- **真实利用率**：cgroup `cpu.stat` 核-秒 + mpstat `%idle`；GPU 用 dmon `sm%`（引擎活跃度）+ 10Hz `util.gpu`（MPS 下补正）。不用 loadavg / `utilization.gpu`。
- **火焰图**：py-spy 采样（P2 用离线 wheel 补装）。本容器 `--native` 失效，用纯 Python 采样 + `pidstat -t` 补 GIL 证据；须选 `comm=python3` 真 PID。
- **正确性**：轨迹级 ATE/RPE（Umeyama 对齐）+ 通道语义 parity（反序列化字段值，非逐字节 hash）；逐位一致仅用于"只改调度不改输出"的流水化/池闸门。
- **工具缺失应对**：无 nsys/dcgmi/perf。GPU-context 假设用"同卡多进程 sm% + 每进程延迟 + MPS A/B"替代 nsys 时间线（N≤8 时 CPU 不可能是混淆项）。
- **零改动**：VIO 数学与 merge 全程未改，测试全用 scratch 副本。

## 附录 B：可复现脚本与数据
- 剖析脚本（`profiling/scripts/`）：`prof_footprint_ab.sh`、`prof_steady_frames.sh`、`prof_flamegraph(_df).sh`、`prof_gpu_concurrency.sh`、`prof_mps.sh`、`prof_util_sweep.sh`、`prof_effective_util.sh`、`prof_compare_poses.py`、`prof_channel_parity.py`、`prof_pool_gate.sh`/`prof_pool_node.sh` 等。
- 原始数据（`profiling/results/`）：`par_sweep_raw.csv`、`concurrency_mps_raw.csv`、`flamegraph_analysis.txt`、`df_stage2_evidence.md`、`p2_sweep_evidence.md`、7 张图。
- 单集直喂复现：`UMI_DIRECT_FEED=1 UMI_PIPELINE=1 CUDA_VISIBLE_DEVICES=0 bash _build_map_one_sensor.sh <ep.mcap> left_wrist <domain> 20`。
