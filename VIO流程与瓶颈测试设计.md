# VIO 转换：流程剖析与性能瓶颈测试设计

> 目的：在**真正理解转换流程**（哪些环节在 CPU、哪些在 GPU、为何逐次不可复现）的基础上，
> 设计一套能测出性能上限的 profiling 方案。
> 本报告是**独立分析文档**，不修改 `提速测试报告.md`（那份是实验结果记录）。
> 所有流程结论均基于阅读 `tinynav/core/{perception_node,build_map_node,models_trt}.py` 源码。
> 日期：2026-07-08。

---

## 1. 转换流程全景

一条 raw `.mcap` → `*_with_pose.mcap` 的转换，对**每只手腕**跑一遍下面的两进程流水线，
两腕（left_wrist / right_wrist）默认**串行**，最后 merge 合并并加回触觉。
每步右侧标注执行资源：**[CPU]** / **[GPU]** / **[IO]**（磁盘或 ROS/DDS 传输）。

```
 raw .mcap ─[IO 读]─► ┌──────────────────── build_map_node.py 进程 ────────────────────┐
 (H.264 视频          │ BagPlayer (SingleThreadedExecutor) —— 定速喂数据                │
  +IMU+内参)          │   1. 读 bag 下一条消息                              [IO 读]     │
                      │   2. _pace_to_timestamp：sleep 到 play_rate 目标时刻 [CPU 空等] │
                      │   3. H.264 解码 (PyAV decode) + cv2.cvtColor         [CPU]★重    │
                      │   4. 发布 Image / IMU / /clock                       [IO: DDS]   │
                      │ BuildMapNode (后端，逐关键帧)                                    │
                      │   a. 关键帧特征 super_point_extractor.infer          [GPU]       │
                      │   b. 关键帧匹配 light_glue_matcher.infer             [GPU]       │
                      │   c. 检索嵌入 dinov2_model.infer                     [GPU]       │
                      │   d. 建库写盘 TinyNavDB(features/embeddings/depths)  [CPU+IO 写]★│
                      │   e. 回环检索 find_loop (numpy 余弦)                 [CPU]       │
                      │   f. 位姿图优化 solve_pose_graph (gtsam ≤1024 迭代)  [CPU]★      │
                      │   g. 占据栅格 generate_occupancy_map (raycast+SDF)   [CPU]       │
                      │   h. save_mapping (np.save poses/内参/dbs)           [CPU+IO 写]★│
                      └──────┬────────────────────────────────────▲─────────────────────┘
              原始图像/IMU   │ [IO: ROS/DDS topic]                 │ 关键帧位姿 /slam/*
                            ▼                                     │ [IO: ROS/DDS]
                      ┌──────────────────── perception_node.py 进程 ───────────────────┐
                      │ MultiThreadedExecutor（异步、多回调组）—— VSLAM 前端            │
                      │   立体同步 ApproximateTimeSynchronizer(slop=0.02)    [CPU]       │
                      │   stereo_queue(maxsize=1) + 单 worker 线程 (丢帧,§3) [CPU 调度]  │
                      │   process() 每帧：                                              │
                      │     · imgmsg_to_cv2 (cv_bridge Image→ndarray)        [CPU]       │
                      │     · 立体深度 stereo_engine.infer (Retinify)        [GPU]       │
                      │     · 视差滤波/掩码 (numpy)                          [CPU]       │
                      │     · 特征提取 superpoint.infer ×2                   [GPU]       │
                      │     · 特征匹配 light_glue.infer                      [GPU]       │
                      │     · IMU 预积分 integrateMeasurement (gtsam)        [CPU]       │
                      │     · PnP estimate_pose (numpy/cv2/gtsam)            [CPU]       │
                      │     · 后端因子图 [ISAM Processing] (gtsam 非线性优化)[CPU]★      │
                      │     · 发布里程计/关键帧 (ROS 序列化)                 [IO: DDS]    │
                      └────────────────────────────────────────────────────────────────┘
                            │
                            ▼  poses.npy （每腕一份）                        [IO 写]
 merge_sqlite_mcap.py ──► *_with_pose.mcap                                 [IO 读+写]
   · 原样 CDR 拷贝所有话题 + 插入 TCP 位姿 + 触觉 (纯拷贝/序列化, ~0.1s)      [CPU+IO]
```

> **★ = 每集耗时/资源的大头嫌疑**：CPU 侧的 H.264 解码(3)、shelve 建库写盘(d)、gtsam 位姿图/因子图优化(f, ISAM)、结果保存(h)。GPU 全是短小 TensorRT 推理（a/b/c 及前端 stereo/superpoint/lightglue）。哪几个 ★ 真占大头由火焰图定（§5.2）。
>
> **资源图例**：`[CPU]` 主机核心计算；`[GPU]` TensorRT 推理；`[IO 读/写]` 磁盘（`/data` LVM，实测每集读 88MB / 写 249MB）；`[IO: DDS]` 进程间 ROS2/FastDDS 传输（走共享内存 `/dev/shm`，本身也耗 CPU+内存带宽）。`[CPU 空等]` = 回放节流的 sleep，不占算力但占墙钟。

**三个进程各自的角色：**

| 进程 | Executor | 职责 | 关键特征 |
|---|---|---|---|
| `build_map_node`（内含 BagPlayer） | SingleThreaded | 定速喂数据 + 解码视频 + 建图后端 | 回放节奏由 `play_rate` 决定，与感知消费速度**解耦** |
| `perception_node` | **MultiThreaded** | 在线 VSLAM 前端：深度、特征、匹配、IMU、因子图 | **异步 + 单帧队列丢帧**（见 §3） |
| `merge_sqlite_mcap` | — | 纯 CDR 拷贝 + 插位姿 + 触觉 | 确定性、~0.1s、纯 CPU/IO |

关键结构事实（源码）：
- 感知订阅的是**原始图像**，H.264 解码发生在 **BagPlayer 里（CPU/PyAV）**，不在感知里
  （`build_map_node.py:764-774` `decoder.parse/decode` + `cv2.cvtColor`）。
- 感知用 `ApproximateTimeSynchronizer(slop=0.02)` 配对左右目（`perception_node.py:128`），
  再经 `stereo_queue(maxsize=1)` 交给**单个** `_process_stereo_worker` 线程（`:155,256`）。
- 感知后端优化是 gtsam `NonlinearFactorGraph` + `CombinedImuFactor`，标记 `[ISAM Processing]`（`:451-479`）。
- 建图后端用 SuperPoint/LightGlue/**Dinov2**（`build_map_node.py:809-811`）做特征+检索嵌入，
  `solve_pose_graph`（gtsam，≤1024 迭代）+ `find_loop`（numpy 余弦）+ `generate_occupancy_map`。

---

## 1.5 时序与并发模型（谁和谁并行 / 串行）

> §1 讲了"每步在哪跑"，但没讲"什么和什么同时发生"。这一节补上时序：
> **核心结论——节点内部基本串行，靠跨进程流水线并行；节拍由 perception 的逐帧串行处理决定。**
> 均据源码：`perception_node.py:156-157,256-268`（单 worker 线程）、`:709`（MultiThreadedExecutor）、
> process() 内全是顺序 `await`（无 `asyncio.gather`）；`build_map_node.py:1250-1267`（SingleThreadedExecutor 单线程 while 循环）、`:707-717`（`_pace_to_timestamp` 只减速不加速）。

| 层级 | 关系 | 依据 |
|---|---|---|
| 两腕（left / right） | **串行**（默认；`--build-parallel` 才并行） | 编排层 do_full |
| 单腕内 3 进程（BagPlayer‖perception‖build_map 后端） | **并行**（流水线，**唯一真并行**） | 3 个独立 OS 进程 |
| build_map 进程内 | **串行**（单线程，回放 ↔ 建图交替占用同一线程） | SingleThreadedExecutor |
| perception 立体流水 | **串行**（单 worker，一次一帧；帧内步骤顺序 await，GPU 步骤不重叠） | stereo_queue(maxsize=1) + 单线程 |
| perception IMU 摄取 | 与立体处理**并发**（但只加锁追加列表，极轻） | ReentrantCallbackGroup |
| GPU 使用 | 每进程各自**串行**发 kernel；同腕两进程的 kernel 在同卡上交替（→ §4 GPU-context 争用来源） | — |

<!-- TIMELINE_PLACEHOLDER -->

**单腕内的流水线时间线**（示意；`P`=感知处理一帧，`M`=建图处理一个关键帧，`▸`=回放发帧）：

```
时间 ──────────────────────────────────────────────────────────►

BagPlayer   │▸▸▸──▸▸────▸▸──────▸▸────  (发帧；被自己进程的 M 卡住时暂停，走走停停)
(build_map  │        └── play_rate=20 想快，但下游跟不上 → sleep_s<0 从不触发，实际=消费速度
 线程①)     │
            │        ┌─ 同进程单线程，M 跑时回放停 ─┐
BuildMapNode│                  █M1████    █M2███     █M3███   (GPU 特征/匹配/嵌入 + 写库 + 位姿图)
(build_map  │
 线程①，与▸交替)
────────────┼───────────────────────────────────────────────
perception  │  ██P1███████  ██P2███████  ██P3███████         (单 worker；一次一帧，帧内顺序:
(worker线程)│     └ 深度→SP×2→LG→IMU积分→PnP→ISAM 全部顺序 await，GPU 步骤不重叠 ┘
            │  ▲ 若 P 还在忙时新帧到达 → 排队帧被丢弃(§3) → 逐次丢不同帧 → 不可复现
perception  │  ·imu·imu·imu·imu·imu·imu·imu·  (IMU 线程与 P 并发，仅加锁追加，极轻)
(IMU线程)   │
```

**从这张图能读出的三件事：**

1. **真正的并行只有"跨进程"这一层**：BagPlayer、perception worker、build_map 后端是 3 个并发进程。但**每个进程内部都是单线程串行**——build_map 的回放和建图抢同一个线程（M 在跑时▸暂停），perception 一次只处理一帧且帧内 GPU 步骤顺序 await 不重叠。
2. **节拍 = perception 逐帧串行处理**：BagPlayer 想按 20× 发，但 perception 单 worker 消费不过来，`_pace_to_timestamp` 的 `sleep_s` 恒为负、从不生效 → 实际吞吐 = 最慢消费者。**play_rate=20 不是实际速度**；单腕 38.5s ≈ ~20s 固定启动 + ~16.6s 串行消费，与 play_rate 无关。
3. **不可复现就发生在 P 的串行节拍上**：P 忙时到达的帧被 `stereo_queue(maxsize=1)` 丢弃，丢哪几帧取决于每个 P 的墙钟时长（§3）——这是"节点内串行 + 跨进程异步"共同的产物。

**对优化的直接含义：**
- 单腕内几乎没有可挖的并行度（各进程已单线程串行、GPU 步骤已顺序）——**提速要么靠并发更多"腕/集"填满 GPU（受 §4 GPU-context 争用限制），要么缩短 P 的串行链（属改 VIO，风险高）**。
- 因此吞吐优化的正道仍是 §4/§5 的方向：**用 MPS 让多进程的 GPU kernel 真并发**（而非同卡时间片串行）、**横向加节点**、**存量走 BACKFILL**。

---

## 2. CPU vs GPU 逐环节拆解

> "算子在哪跑"直接决定瓶颈分析与优化手段。下表按流水线顺序，标注每个环节的**执行位置**。

### 2.1 perception_node（前端，占单腕耗时的主体）

| 环节 | 位置 | 说明 |
|---|---|---|
| 立体图像配对 `ApproximateTimeSynchronizer` | **CPU** | message_filters，时间戳配对 |
| `imgmsg_to_cv2`（ROS Image→ndarray） | **CPU** | cv_bridge，格式转换 |
| **立体深度 `stereo_engine.infer`（RetinifyTRT）** | **GPU** | TensorRT，视差→深度 |
| 视差滤波/掩码（numpy） | **CPU** | `disparity[...]=0` 等 |
| **特征提取 `superpoint.infer` ×2**（前一关键帧 + 当前帧） | **GPU** | TensorRT |
| **特征匹配 `light_glue.infer`** | **GPU** | TensorRT |
| IMU 预积分 `preintegrated_imu.integrateMeasurement` | **CPU** | gtsam，逐条 IMU |
| PnP 位姿估计 `estimate_pose` | **CPU** | numpy/cv2/gtsam（带 lru_cache） |
| **后端因子图 `[ISAM Processing]`** | **CPU** | gtsam 非线性优化（IMU+视觉因子） |
| 发布里程计/关键帧/可视化 | **CPU** | ROS 序列化 |

### 2.2 build_map_node（回放 + 后端）

| 环节 | 位置 | 说明 |
|---|---|---|
| **H.264 解码**（BagPlayer） | **CPU** | PyAV `decoder.decode` + `cv2.cvtColor` |
| 定速回放节流 `_pace_to_timestamp` | **CPU** | sleep 到目标时间戳 |
| 关键帧特征 `super_point_extractor` | **GPU** | TensorRT |
| 关键帧匹配 `light_glue_matcher` | **GPU** | TensorRT |
| **检索嵌入 `dinov2_model`** | **GPU** | TensorRT（回环检索用） |
| 建库写盘 `TinyNavDB`（features/embeddings/depths） | **CPU + 磁盘IO** | `IntKeyShelf`=shelve/pickle |
| 回环检索 `find_loop` | **CPU** | numpy 余弦相似度 |
| **位姿图优化 `solve_pose_graph`** | **CPU** | gtsam，≤1024 迭代 |
| 占据栅格 `generate_occupancy_map`（raycast+SDF） | **CPU** | numpy |
| 保存 `save_mapping`（np.save） | **CPU + 磁盘IO** | poses/内参/dbs |

### 2.3 一句话总结 CPU/GPU 分工

- **GPU 只做 5 类 TensorRT 推理**：立体深度(Retinify)、SuperPoint、LightGlue、Dinov2。都是**小模型、短 kernel**——这解释了实测 GPU 显存只用 2.4GB、单集 GPU 引擎活跃时间很短。
- **CPU 做其余一切**：H.264 解码、cv_bridge 转换、numpy 滤波、**IMU 预积分与 gtsam 因子图/位姿图优化（VIO 的数学核心）**、shelve 建库、占据栅格、ROS 序列化、以及回放节流。
- **磁盘 IO**：建库 + 保存，实测每集写 249MB / 读 88MB。

> 直觉修正：这是一个 **GPU 轻、CPU 重** 的流水线。神经网络部分（GPU）快而小；
> 真正吃时间的是 CPU 侧的经典 SLAM 计算（gtsam）+ 逐帧编排 + 固定启动开销。
> 这也是为何"常驻 worker 摊销模型加载"收益有限——瓶颈本就不在 GPU 模型加载。

---

## 3. 为什么逐次不可复现（精确机制，非"玄学抖动"）

实测：同一集、同一 GPU、全新进程连跑 3 次，right_wrist 得到 **3 个不同的 poses.npy，
关键帧数 37/37/38**（left_wrist 恰好稳定 39/39/39）。根因**不是**浮点/CUDA 非确定性，
而是**在线 SLAM 的"跟不上就丢帧"机制在离线回放下被时序放大**。逐层拆解：

**① 回放与消费解耦。** BagPlayer 按 `play_rate×实时`（20×）**主动**发布图像，
不管感知是否处理得过来（`build_map_node.py` play_next → `_pace_to_timestamp`）。

**② 感知端的单帧队列 + 丢帧。** `perception_node.py:248-255`：
```python
def _aligned_stereo_callback(self, stereo_pair_msg):
    if image_timestamp - self.last_processed_timestamp < 0.1333:   # ~7.5Hz 限频（确定性）
        return
    try:
        self.stereo_queue.put_nowait(stereo_pair_msg)              # 队列容量=1
    except Full:
        self.stereo_queue.get_nowait()                            # ← 丢掉排队中的旧帧
        self.stereo_queue.put_nowait(stereo_pair_msg)             #   换成新帧
```
只有**单个** `_process_stereo_worker` 线程在跑 `process()`（GPU 推理 + gtsam）。
**若新帧到达时 worker 还在忙，则队列里那帧被丢弃、替换成新帧**——到底丢哪几帧，
取决于每次 `process()` 花了多少墙钟时间 vs 帧到达时刻，**这是调度相关、逐次不同的**。

**③ 丢帧不同 → 关键帧不同 → 位姿图不同 → poses.npy 不同**（连关键帧数都变，37 vs 38）。

**④ 异步叠加。** MultiThreadedExecutor 下 IMU（ReentrantCallbackGroup）与立体
（MutuallyExclusiveCallbackGroup）回调并发，每个关键帧预积分到的 IMU 区间也受到达顺序影响。

**结论**：不可复现是**设计使然**（在线 SLAM 实时性优先、丢帧保延迟），不是 bug。
含义对 profiling 极重要：
- **任何耗时/占用测量都要多次取中位数**，不能单点。
- **"逐字节一致"不能作为任何优化的验收闸门**（这也是常驻 worker 方案验收失败的部分原因）。
- 若未来要"可复现转换"，需改成**不丢帧的离线模式**（队列不设上限或回放等待消费），
  但那会改变 VIO 输出、且降低吞吐——属于**改 VIO 行为**，超出"提速"范畴，需单独决策。

---

## 4. 这对性能分析意味着什么

结合已实测的数据（见 `提速测试报告.md` §2.5）：

| 事实 | 值 | 出处 |
|---|---|---|
| 单集（2 腕串行，1 GPU） | ~77s 墙钟 | 基线 |
| 每集 CPU 核-秒 C | **234.7** | cgroup `cpu.stat` |
| 平均占用核数 | 3.13 | C/wall |
| GPU 显存峰值 | 2.4 GB / 143 GB | — |
| 8 卡并行观测平台 | ~1190 集/时 | PAR 扫描 |
| CPU-计算上限 | 2945 集/时 | 192×3600/234.7 |
| GPU-计算上限 | ≥2292 集/时 | 由 PAR=8 反推 |

**推论（待 §5 证实）**：
- GPU 轻、CPU 重、且单集只占 3.13 核 → 单节点 192 核理论能塞下 ~60 路并发，
  但观测平台 1190 只到计算上限的 ~40%。**平台不是计算硬墙，是软瓶颈。**
- 结合 §3 的架构，软瓶颈的**头号嫌疑**是**多进程共享单卡时 GPU-context 的时间片串行化**
  （每卡多进程的小 kernel 挤过同一 context 串行执行），次要是每集**进程启动/DDS 发现/
  模型加载**（每腕 ~20s 固定开销）与**磁盘 IO 突发**（持续写 ~82MB/s @1190/时）。

---

## 5. 如何测这个瓶颈（profiling 方案）

> 原则：**先分层量出"时间花在哪、被什么挡住"，再谈优化。** 每步产出数据 + 图。
> 严格用 CPU-busy（cgroup/mpstat）与**真实 GPU 引擎活跃度**（DCGM/Nsight），
> **不用 loadavg / `nvidia-smi utilization.gpu` 判断饱和**（前者含 D 态等待，后者是 kernel 驻留率）。

### 5.1 单集阶段剖析（时间去向）
- 用日志时间戳 + 探针把单腕 38.5s 拆成：模型加载 / 首帧 / 逐帧感知稳态 / mapping 保存。
- 复用 `profile_one.sh`（cgroup `cpu.stat` 前后取差、pidstat、GPU 采样）。
- **每口径 ≥3 集 × ≥3 次取中位数**（§3 非确定性）。
- 产出：**单集阶段甘特图**（各阶段时间占比）。

### 5.2 CPU 剖析（定位 CPU 时间 + 是否串行）
- `mpstat -P ALL 1`：是"少数核 100%"（串行信号）还是多核均摊。
- `pidstat -t -u 1`：**per-thread** %CPU，找热线程（gtsam 优化？H.264 解码？shelve 写盘？）。
- **火焰图**：优先 `py-spy record`（采样、免改码）；退而 `perf record -g`+FlameGraph（能连 gtsam/native 栈）。
- 产出：**CPU 火焰图**，回答"逐帧感知/建图的 CPU 时间具体耗在哪个函数、是否卡单线程"。

### 5.3 GPU 剖析（**核心**：验证 GPU-context 串行化假设）
- 真实引擎活跃度取代 util：`dcgmi dmon -e 1002,1003,1005`（SM active/occupancy/mem BW），
  无 dcgmi 则 `nsys` GPU metrics 或 `nvidia-smi dmon -s um` 高频采样。
- **Nsight Systems 时间线（金标准）**：`nsys profile` 抓 CUDA API+kernel。
  **决定性实验**：同一张卡上**并发 2–4 个转换进程**，看不同进程的 kernel 在时间线上
  是**串行错开**（证实 context 时间片）还是并发重叠。产出**多进程 GPU 时间线截图**。
- 单进程 GPU 占空比：单集里 GPU 真算 vs 空等（等 CPU 喂数据）的时间——判断"喂不饱"还是"被串行化"。

### 5.4 饱和扫描（补全上限曲线）
- PAR = 已有 8/24/32/48/64 + **补 80/96**（确认平台/回退），需相对空闲节点。
- 每档同时记录：吞吐、真实 CPU %busy（mpstat `%idle`→0?）、真实 GPU 引擎活跃度（→100%?）、
  cgroup 核-秒、峰值 IO 带宽、失败数。
- 产出：**吞吐 vs PAR** + **真实利用率 vs PAR**（CPU 与 GPU 双线）+ **roofline 图**（CPU 2945 / GPU ≥2292 / IO 墙 / 观测 1190 四线对比）。

### 5.5 MPS A/B（头号优化的实测，必做）
- 开 NVIDIA MPS（`nvidia-cuda-mps-control -d`），多进程共享一个 GPU context。
- A/B：固定每卡 N 进程，MPS 关 vs 开，测吞吐 + GPU 引擎活跃度 + 单集延迟。
- 产出：**MPS A/B 柱状图** + 实测收益 %（能否把"聚合 GPU 计算 ~50%"推高、吞吐往 2292 推）。

### 5.6 IO 墙核对
- `iostat`/cgroup `io.stat` 在饱和档看写带宽是否逼近 `/data`（LVM）上限；
- 若 IO 成墙，考虑输出到更快盘或减少 shelve 写量（不改 VIO 数值）。

---

## 6. 硬约束（做 profiling 时必须遵守）
- **不改 VIO 计算**：`perception_node.py`/`build_map_node.py`/`models_trt.py` 只读，只做外部观测与编排。
- **绝不碰真实产物**：一律用 raw 集的 scratch 副本；5021 个 `*_with_pose.mcap` 及 `.pre_tactile.bak` 只读。
- **共享节点礼仪**：先 `nvidia-smi` 看占用，避开被占的卡（当前 GPU0 有他人 picpp 服务，且可能整机被占）；无空闲卡就等或降规模并注明。
- **超时兜底**：所有 build_map 调用包 `timeout`（`wait_for_perception_subscribers` 无超时，感知崩会无限占卡）。
- **多次取中位数**（§3 非确定性）；**不用 loadavg/util 判饱和**。
- 收尾清理进程/tmux/scratch/自开的 MPS daemon。

---

## 7. 预期结论形态（DoD）
- GPU-context 串行化假设被**多进程 GPU 时间线**证实或证伪，并给出定量占比。
- roofline 四条线（CPU/GPU/IO/观测）都有实测数，指明真正的绑定资源。
- CPU 火焰图指出 CPU 时间大头（预期在 gtsam 优化 / H.264 解码 / shelve 写盘之一）。
- MPS A/B 有实测吞吐差。
- 优化清单按收益/风险/工作量排序，每项带有依据的预期收益。

---

### 附：关键源码位置速查
- 立体同步/丢帧：`perception_node.py:126-129, 248-266`（`stereo_queue(maxsize=1)` + 驱逐）
- 感知 GPU 推理：`perception_node.py:333-360`（stereo/superpoint/lightglue infer）
- 感知 CPU 后端：`perception_node.py:451-479`（`[ISAM Processing]` gtsam）
- 回放 H.264 解码：`build_map_node.py:764-774`（PyAV，CPU）
- 建图 GPU 模型：`build_map_node.py:809-811`（SuperPoint/LightGlue/Dinov2）
- 建图 CPU 后端：`build_map_node.py:174`(`solve_pose_graph` gtsam)、`:190`(`find_loop`)、`:204`(`generate_occupancy_map`)
- 模型基类（TensorRT 加载）：`models_trt.py:40-49`（`TRTBase`，engine 反序列化 + capture_graph）
