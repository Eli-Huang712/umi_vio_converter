# P1 实现说明：VIO 转换的进程内同步喂帧（direct-feed）

> 对应任务 `P1_HANDOFF_PROMPT.md`。分支 `feature/p1-direct-feed`。
> 结论权威版是 `VIO转换瓶颈分析报告.md`；本文件记录 **P1 的实现、实测收益、验收数据**。
> 实测日期：2026-07-08 ｜ 节点：h2（jhhuang-h200-qinghua-2，8×H200 空闲）｜ 容器：`tinynav_flatbuffer`。

---

## 1. 改了什么（一句话）

把 `build_map` 的喂帧路径从"`BagPlayer` 定速回放 → 序列化成 `sensor_msgs/Image` → 过 DDS →
`perception` 反序列化 → `ApproximateTimeSynchronizer` 异步配对"改成
**一个进程内、把解码后的 numpy 帧点对点同步喂给 perception + build_map**，绕开火焰图里占
build_map ~63% 的 `sensor_msgs/_image.py` 逐字节 CDR (反)序列化和 rclpy executor spin。

**VIO 数学零改动**：`perception_node.py` / `build_map_node.py` / `models_trt.py` 一行未改，
以原样 import。只改"数据怎么进来"。

## 2. 实现形态（改动清单）

| 文件 | 改动 |
|---|---|
| `direct_feed_build_map.py` | **新增**：单进程 driver。见下。 |
| `_build_map_one_sensor.sh` | 加 `UMI_DIRECT_FEED=1` 分支：跑单进程 driver（不起 tmux/perception pane），包 `timeout` 看门狗（`UMI_BUILD_MAP_TIMEOUT_SEC`，默认 600s）；导出 `ROS_DOMAIN_ID` 隔离并行 DDS。stock 双进程路径仍是默认。 |
| `deploy.sh` | 把 `direct_feed_build_map.py` 加入 `NEWFILES` + 语法自检。 |
| profiling/scripts | 新增 `prof_compare_poses.py`（ATE/RPE）、`prof_validate_p1.sh`、`prof_footprint_ab.sh`、`prof_channel_parity.py`、`prof_run_parity.sh`、`prof_flamegraph_df.sh`。 |

`direct_feed_build_map.py` 核心组件：
- **`read_episode`**：用 stock 同款解码原语（`flatbuffer_codec.decode_*` + PyAV H.264）从 raw bag
  解出 numpy 帧（左 mono8+bgr8、右 mono8）、旋转后的 `Imu`、合成 `CameraInfo`——**全程不序列化**。
- **`ShimBridge` + `_NpImage`**：替换两个 node 的 `self.bridge`。`imgmsg_to_cv2` 直接返回 numpy
  数组（仅在请求 encoding 不同时做 cv2 转换），`cv2_to_imgmsg` 包回 `_NpImage`。**零 CDR、零逐字节拷贝。**
- **publisher 替换**：perception 的 keyframe_pose/image/depth pub 改成捕获到 holder；其余 pub
  与 TF broadcaster 换 no-op（消息的**构建**仍跑，VIO 行为不变；只砍 DDS 序列化）。
- **driver 主循环**：先喂一次 `CameraInfo`（设 K/baseline），再按**严格时间戳序**喂 IMU + stereo：
  - 左右目按最近邻配对（slop 0.02s；帧硬件同步到 ~33µs，确定性）。
  - 复刻 perception 的 7.5Hz 节流（`<0.1333s` 跳过）。
  - **IMU 1 步前瞻**：每帧 `process()` 前喂 `ts≤T` 的全部 IMU **加**第一条 `ts>T`，
    复刻 stock 异步投递下 deque 的状态（`process()` 里"specially process the last imu"那步需要 `ts>T` 的样本在场）。
  - 每帧 `perception._async_loop.run_until_complete(process(...))`，出关键帧就同步调
    `build_map.keyframe_callback(...)` 喂该帧的 bgr8。
- 跳过 `ImuPropagatorNode`（只产 `mapping_continuous_odom.npy`，merge 从不读；无关验收）。

## 3. 为什么这条路对（源码依据）

- `merge_sqlite_mcap` **只读** 每个 `*_db/poses.npy`；触觉/IMU/夹爪/camera_info 通道由 merge 从
  raw bag 原样透传，**与 VIO 路径无关**。所以正确性闸门 = 复现 perception 的关键帧位姿。
- `build_map.solve_pose_graph` 实为 **no-op**（真解算被注释，`build_map_node.py:186-187`），
  故 `poses.npy` = perception 关键帧里程计按时间戳收集排序。
- UMI flatbuffer bag **无 `/tf`**，merge 忽略 `T_rgb_to_infra1`/连续里程计/occupancy——最小正确路径很小。

---

## 4. 验收数据（h2，raw 集 scratch 副本，真实产物零改动）

### 4.1 吞吐（核心收益）——`prof_footprint_ab.sh`

CPU 核-秒 C 是单节点 roofline 的决定量（报告：`192×3600/C` 集/时）。同集 stock vs direct-feed、
空闲容器独占、cgroup `cpu.stat` 前后取差：

| 集（帧数） | | stock | direct-feed | 变化 |
|---|---|---:|---:|---:|
| ep0（134fr/腕） | 墙钟(双腕) | 67.9s | 20.9s | **−69%（3.25×）** |
| | CPU 核-秒 C | 113.9 | 73.4 | **−36%** |
| | 平均占用核 | 1.68 | 3.51 | 摆脱 GIL 单核 |
| | CPU roofline | 6068 集/时 | 9417 集/时 | **+55%** |
| ep2（327fr/腕） | 墙钟(双腕) | 140.6s | 38.0s | **−73%（3.7×）** |
| | CPU 核-秒 C | 271.9 | 180.1 | **−34%** |
| | 平均占用核 | 1.93 | 4.74 | 摆脱 GIL 单核 |
| | CPU roofline | 2542 集/时 | 3838 集/时 | **+51%** |

- 两集一致：**核-秒 −34~36% → CPU roofline +51~55%**——这是**下界**（只算 CPU 一堵墙移动）。
- ep2 的 C_stock=271.9 与报告 median ~235 同量级，具代表性。
- **`平均占用核 1.9→4.7` 是机理直证**：stock build_map 被 GIL 钉在 ~1 核（报告 top/sum=0.92）就是那堵软墙；
  direct-feed 单进程内解码/TRT/gtsam 真并行摊到多核，软墙消失。

### 4.1b PAR 满载稳态吞吐（帧/时，size-invariant）——`prof_steady_frames.sh`

在 128 集 scratch 池、8 卡空闲的 h2 上做**稳态**吞吐扫描（连续队列保持恰好 PAR 集在飞，
掐头去尾只取 [90s, 270s] 稳态窗口；单位 = **帧/时**，避开集大小干扰——帧数=双腕左相机
`left_h264/video` 消息数）。均 **0 失败**：

| PAR | 模式 | **帧/时** | 集/时 | CPU busy | GPU sm% | 失败 | vs stock@64 |
|---:|---|---:|---:|---:|---:|---:|---:|
| 64 | **stock** | **600,440** | 1,280 | 32.7% | 5.1% | 0 | 基准 |
| 32 | df | 1,270,020 | 2,660 | 44.9% | 49.0% | 0 | +112% |
| 48 | df | 1,533,500 | 3,160 | 55.5% | 60.2% | 0 | +155% |
| **64** | **df（峰值）** | **1,612,520** | 3,320 | 56.8% | 60.5% | 0 | **+169%（2.69×）** |
| 80 | df | 1,561,420 ↓ | 3,220 | 51.5% | 52.0% | 0 | +160% |
| 96 | df | 1,121,220 ↓ | 2,380 | 56.7% | 55.6% | 0 | +87% |
| 128 | df | 1,200,620 ↓ | 2,480 | 58.0% | 54.0% | 4 | +100% |

- **直接回答（PAR=64 同档对比）：600,440 → 1,612,520 帧/时 = +169%（2.69×）**，远超 P1 目标（+25–50%）。
- **为何 +169% ≫ footprint 预测的 +51~55%**：roofline 数只算"CPU 一堵墙右移"。但 direct-feed 还**顺带解放了 GPU**——
  stock 被 GIL 序列化钉住，喂不动 GPU（PAR=64 时 GPU sm 仅 5.1%、CPU 也仅 33%，**两种资源同时闲置**）；
  direct-feed 喂帧够快，把 GPU 拉到 60%、CPU 拉到 57%，**一次吃下两个轴的余量**。核-秒下降定下界，
  消除关键路径上的序列化停顿兑现其余。
- **df 的峰在 PAR=64**：32→48→64 单调上升（+112%→+155%→+169%），过 64 即回退（80 −3%、96/128 掉 ~25–30%，
  128 起开始出现失败）。与 stock 同形状（stock 也在 64 见顶回退，见报告），只是天花板高 2.69×。
  **加大 PAR 收益不再增大——PAR=64 是 df 的实测最优点**，且正是报告为 stock 建议锁定的同一 PAR，无需改编排。
- **峰值处 CPU/GPU 仍未打满（57%/60%）**：同报告的"软墙"特征，只是被抬到更高水平。绑定资源不是 CPU/GPU 总量，
  而是过 PAR=64 后每进程争用（CPU 超订 ~300 核需求 vs 192 核 + 同卡 GPU-context 交错）使单集延迟膨胀快过并行收益。
- stock 稳态 600K 帧/时与报告 545K 同量级（本池集大小分布略偏大/快，故略高），gpu_sm 5.1%≈报告 7%——
  口径自洽，证明测量环境可信。
- **测量陷阱**：整批 wall-time（xargs 两波）会被 drain 尾巴 + stock 无超时挂起（`wait_for_perception_subscribers`
  无 timeout，`build_map_node.py:610`，高 PAR DDS 握手偶发挂死）严重污染（首测得 219K 假值）。改用**连续队列 +
  稳态窗口 + 每单元 `timeout` 看门狗**（stock 也包，并按确定性会话名回收泄漏的 tmux pane）后干净复现。

### 4.2 flamegraph（"63% 序列化"去向）

py-spy 在本容器与离线节点均不可得（wheel 随旧 prof_scratch 删除，H200 无外网）。改用**更强的直证**：
cgroup 核-秒 + 每进程平均占用核。stock 的 63% 花在 `sensor_msgs/_image.py` 的逐字节 CDR + executor spin，
表现为 build_map 被 GIL 钉在 ~1 核；direct-feed **架构上删除了整条 `sensor_msgs/Image` 序列化 + rclpy
executor**，实测 `平均核 1.9→4.7`（不再单核串行）且 C 降 34–36%——正是火焰图会预测的结果。

### 4.3 正确性——轨迹级 ATE/RPE（`prof_validate_p1.sh` + `prof_compare_poses.py`）

3 集 × 2 腕，各 stock 3 次 + direct-feed 2 次。ATE = Umeyama 刚性对齐后平移 RMSE（common 关键帧）：

| 集/腕 | stock kf | df kf | stock 自抖动 | **df 逐次determinism** | **df vs stock ATE_rmse (max)** |
|---|---:|---:|---:|---:|---:|
| ep0 left | 27 | 29 | 0.000mm | **0.000mm** | **0.46mm** (1.4mm) |
| ep0 right | 28 | 30 | 0.000mm | **0.000mm** | **13.6mm** (41.6mm) |
| ep1 left | 44 | 46 | 0.000mm | **0.000mm** | **0.42mm** (1.5mm) |
| ep1 right | 44 | 45 | 0.000mm | **0.000mm** | **0.66mm** (2.9mm) |
| ep2 left | 72 | 74 | 0.000mm | **0.000mm** | **0.28mm** (1.6mm) |
| ep2 right | 72 | 74 | 0.49mm(3.2mm) | **0.000mm** | **13.1mm** (71.0mm) |

**读出三点 + 一个团队决策项：**
1. **direct-feed 逐次完全确定**（所有 2 次重跑 0.000mm）——顺带修了 stock 的时序非确定性（P1 红利）。
2. **4/6 亚毫米**（0.28–0.66mm），与 stock 不可区分——证明喂帧路径的 VIO 数学是忠实的
   （若有系统性 bug，6 个全会偏，不会只偏 right_wrist）。
3. **2/6（ep0/ep2 right_wrist）偏 ~13mm**（max 42–71mm）。机理已定位（见 §4.5），**非 bug**。
4. **决策项**：见 §4.5——需团队judgment 13mm 是否落在"VIO 自身精度/传感器噪声"可接受范围内。

### 4.4 通道 parity——`prof_channel_parity.py`（语义比对）

stock vs direct-feed 的 `*_with_pose.mcap`：**所有非位姿通道语义完全一致**——
触觉 ×4、IMU ×3、夹爪 ×2、camera_info ×6、video_encoded ×6 全部 `identical`，仅位姿话题按预期不同。
- **闸门自证**：stock-vs-stock（同位姿、同 raw、同 merge）对照跑 = `PARITY_OK`。
- 关键教训：**不能用逐字节 hash**——FastCDR 的对齐/padding 字节来自未清零缓冲，同一消息逐次序列化出
  不同字节（stock 对自己都 FAIL）。正确闸门是**反序列化后的字段值**（语义比对），对 padding/写序无关。

### 4.5 right_wrist ~13mm 偏差：机理与定性

**机理（源码 + 数据双证）**：stock 的 `perception_node.py:155` `stereo_queue(maxsize=1)` 在 worker
仍忙时**丢帧**（process ≈138ms/帧 ≈ 133ms 节流间隔，边缘态偶发丢弃）；direct-feed **不丢**——处理每
一个过节流的帧。所以 **df 处理的帧是 stock 的严格超集**——每个偏差集都显示 `a_only=2, b_only=0`
（stock 关键帧全在 df 内，df 多 2 个）。这 2 个多出的帧改变了滑窗 ISAM（`_N=10`）与 PnP 链，
使 common 时间戳的关键帧也重优化出不同解；平缓的 left_wrist 亚毫米，动态的 right_wrist 达 ~13mm。

**为什么这是允许的（handoff §4）**：任务明确松绑——"同步喂帧会得到一条*不同但确定*的轨迹，这是允许的"，
且团队已确认可接受 VIO 逐次抖动（等同传感器噪声）。direct-feed 甚至**更"对"**（不丢帧、用满信息）。

**与 §5 字面闸门的张力（需团队定）**：§5 要求偏差落在"stock 自身重跑抖动带"内。但**实测 stock 在空闲
GPU 上完全确定**（自抖动 0.000mm，甚至 6 路同卡共居下仍 0.000mm——见 `prof_validate_p1` 附带的
jitter probe），无法复现报告里"37/37/38"那种负载下非确定性。于是 §5 的字面带宽 ≈ 0，任何"不同但确定"的
轨迹都会名义上"超带"。**这是闸门口径问题，不是 P1 缺陷**：§5 的本意是"落在 VIO 精度内"，而 §4 明确允许不同轨迹。
→ **请团队判定**：13mm ATE（max 71mm）对下游训练是否在可接受精度内。若否，可选项见 §5。

## 4.9 当前阶段（df PAR=64 峰值）的制约因素分析——py-spy + 同卡扫描 + MPS A/B

> 疑问：df PAR=64 峰值处 CPU 仅 57%、聚合 GPU sm 仅 60%，两者皆不饱和，为何吞吐撞墙？
> 补装 py-spy（离线 wheel）+ 单卡 N 扫描 + MPS A/B 定位。完整证据见 `profiling/results/df_stage2_evidence.md`，
> 结论经 4 维独立分析 + 3 路对抗证伪（workflow）。**瓶颈已从 stock 的 CPU/GIL 单线程转移到 GPU 侧。**

**主约束（近因）= 单卡多进程 CUDA 上下文时间片串行化。** 决定性数字（同卡 N=8 MPS A/B）：
开 MPS 后每进程延迟 **30.4s→19.3s**（回落到孤立单进程的 19.7s，**共租惩罚被完全抹除**），同卡吞吐
**756→1147（+52%）**，而 sm **反降 73%→54%**。MPS 只改跨进程 GPU 上下文共享、不碰 CPU/解码/带宽——它
在降低占空比的同时提速 +52%，只有当"73% 是被串行化摊薄的低占用空转、而非算力饱和"时才可能。单卡 N 扫描
（N=1→8）佐证：边际吞吐 +165/+223/+135/+44 递减、N=8 墙钟 +56%，而 sm 只到 73% 就封顶——留有余量却死掉 =
调度串行化而非算力墙。对照 stock 同卡（报告）：墙钟平、吞吐线性、sm 2→7%（GPU 空闲，无上下文争用）。

**主约束（上游根因）= 整段前置解码造成的低 GPU 占空比。** flamegraph（单进程 on-CPU）：**53.6% = `read_episode`
在任何 VIO 前一次性解码整段 H.264（此间 GPU 全程空闲）**，仅 33% 是 VIO 喂入相位（且被 busy-wait 轮询占据）。
因每进程约半条命 GPU 空闲，才需 ~8 路共租才能喂饱一张卡——**这正是制造 8 路共租、触发上下文串行化的成因**。

**次约束 = 主机侧过订阅（决定峰值位置 + >64 塌陷）。** PAR>64 是**塌陷**而非平台（64=1.61M→96=1.12M<PAR48→
128 且 4 次失败）；纯每卡串行化只会缓降。~450 原生线程、瞬时 ~9-10 核/进程，高 PAR 下各进程 CPU 密集的解码相位
相互碰撞 → 主机过订阅。峰值 64 = "GPU 侧再塞进程填卡的收益" 与 "CPU 侧进程过多拖垮主机的代价" 的平衡点。

**三级约束**：(a) run_graph 的 `while cudaEventQuery: await asyncio.sleep(0)` **busy-wait 空转**（33% on-CPU，
烧核非有用功；故"57% CPU"高估了有用利用率）；(b) 每进程固定开销——numba JIT **7.7%** 每进程重付、TRT 加载、
以及**收尾 shutdown 挂死**（进程滞留 ~1.1GB GPU、D-state、扛过 SIGKILL；生产靠 `timeout` 看门狗掩盖，更可能经
显存压力贡献 >64 塌陷/失败）。非 GIL（450 原生线程释放 GIL，对比 stock top/sum=0.92）。

**为何两者都不饱和却撞墙**：GPU sm 60% 是"时间活动"代理非算力饱和——时间片下任一时刻只一个上下文在跑，空隙被
上下文切换气泡 + spin 停顿 + 解码期 GPU 全空闲填充；**MPS 让 sm 降到 54% 而吞吐升 +52% 是决定性反证**。
CPU 57% 含 33% 空转（高估有用率）且是相位平均（解码相位尖峰、GPU 相位空闲），非 GIL 约束。合起来即墙的形状：
低占空比 → 需多路共租填卡 → 共租在 GPU 相位碰撞 → sm~73% 即封单卡（N≈6-8）→ 节点 8×8=PAR64；再往上堆进程去追
未填满的余量就触发主机过订阅 → 回归+失败。**64 是两股力的平衡点，"都没打满"正是"复用效率+占空比联合封顶"的必然读数。**

**下一步杠杆（收益/风险/证实状态）：**

| # | 杠杆 | 预期收益 | 证实状态 | 风险 |
|---|---|---|---|---|
| 1 | **启用 MPS** | 单卡 **+52%**（实测）；**全节点 PAR=64 = 1.61M→1.86M 帧/时 = +15.4%（实测）** | **已证实（E6+E6b）** | 全节点增益远小于单卡（MPS 加速 GPU 相位后，根因 = 低占空比/主机协调 重新显形；节点 CPU 59%/GPU 50% 仍不饱和）。近零代码，建议直接开 |
| 2 | **流水化解码**（decode-ahead 双缓冲，边解码 N+1 边 VIO 消费 N） | 单进程 GPU 占空比 ~1/3→~2/3，更少进程即可填卡 → 把峰值移到 >64 塌陷悬崖之前；节点 ~+15~30%（假设） | **假设**（PyAV 释放 GIL，重叠可行；CPU 有余量） | 应叠加在 MPS 之后；直击根因，闭合 sm 缺口 |
| 3 | **常驻/热进程池**（N=PAR workers 跨 episode 复用） | 稳态 ~3-8% + 打包摊销 JIT/TRT加载 + 消除 shutdown 挂死 → 延展可用 PAR 越过 64、收回塌陷与失败 | **假设**（各部件成因已测，整体未测） | 重构成本最高，但一次解决 3 个固定开销问题 |
| 4 | **replace spin-poll + 修 shutdown 挂死** | 释放 ~0.75 核/进程（PAR64 ≈48 核）、右移 >64 塌陷悬崖；numba AOT/cache 去掉 7.7% | 成因已测（E2/E7），净收益假设 | 单独收益小（GPU 仍是墙），作为廉价附带项 |

**推荐顺序**：MPS 先行（实测 +15% 节点、近零代码）→ 解码流水（推卡向饱和，把峰值移到塌陷前）→ 常驻池（打包固定开销、
延展 PAR 上限）→ spin/挂死修复（廉价附带）。**关键缺测**：全节点 MPS 已补（+15%）；仍缺 Nsight 时间线确认
"GPU 端 kernel 串行 vs CPU 端 launch/query 串行"、及流水化解码变体的单卡 N 扫描（验证根因）。

## 5. 若 13mm 需收紧（可选，未实施）

- **最忠实**：让 df 复刻 stock 的丢帧——但丢帧取决于 `process()` 墙钟，跨硬件不确定，且会**重新引入**
  P1 要消除的非确定性，得不偿失。
- **务实**：接受 df"不丢帧"的确定轨迹为新基准（更信息完整），用 ATE 对齐后偏差 <VIO 精度 论证等价。
- 建议取务实项：df 的确定性 + 不丢帧本身是质量提升，13mm 在 UMI 手腕 ~0.5m 级轨迹上属 VIO 精度类。

## 6. 复现命令（h2 容器内）

```bash
# 部署（本地 → h2 容器）：见 deploy.sh；或直接 docker cp direct_feed_build_map.py + 改后的 _build_map_one_sensor.sh
# 单集 direct-feed：
UMI_DIRECT_FEED=1 CUDA_VISIBLE_DEVICES=0 bash /tinynav/tool/umi/_build_map_one_sensor.sh <ep.mcap> left_wrist 150 20
# 正确性：bash prof_validate_p1.sh <hash> <gpu>   (DBASE 唯一/并行)
# 吞吐：   bash prof_footprint_ab.sh <ep.mcap> <gpu>
# parity： bash prof_run_parity.sh <hash>
```

## 7. DoD 勾稽

- [x] 喂帧改为进程内同步、绕开 `sensor_msgs/Image` CDR + rclpy executor（核-秒 −34~36%、平均核 1.9→4.7 直证）
- [~] 轨迹级容差：4/6 亚毫米；2/6 right_wrist ~13mm，机理=帧超集（§4 允许），**待团队判定是否收紧**
- [x] 触觉/IMU/夹爪/camera_info 通道数量与内容不变（语义 parity `PARITY_OK`，对照自证）
- [x] 单节点吞吐提升：**PAR=64 稳态 600K→1.61M 帧/时 = +169%（2.69×）**（远超 +25–50% 目标）；
      核-秒 −34~36%（roofline 下界 +51~55%）；wall 3.25~3.7×；df 在 PAR=64 仍未饱和
- [x] 不改 VIO 数学；真实产物零改动（全用 scratch 副本）；环境清理（见收尾）
- [x] 记录改动/实测/验收数据（本文件）

