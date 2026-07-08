# 任务：P1 —— 削减 VIO 转换的 ROS2 图像消息序列化开销（允许大改）

你要在 `feature/p1-direct-feed` 分支上，动手实现一版更快的 tinynav UMI VIO 转换。这不是探索性任务——**瓶颈已经被上一轮系统性剖析精确定位、真凶明确、方向清晰**。你的活是**把它改快**，并用轨迹级容差证明没改坏。

先读两份东西，再动手：① 仓库根 `VIO转换瓶颈分析报告.md`（权威结论）；② `profiling/docs/VIO流程与瓶颈测试设计.md`（基于源码的流水线逐步解剖）。本 prompt 是它们的行动版摘要。

---

## 1. 一句话背景

这条流水线把 raw 机器人数据集（`.mcap`）转换成带位姿轨迹的 `*_with_pose.mcap`：为**左右两只手腕**各跑一遍 VIO（`perception_node` + `build_map_node` 两个 ROS 进程，经 ROS topic 通信、由 `BagPlayer` 定速回放驱动），再 merge。批量转换的单节点吞吐卡在 **PAR=64 ≈ 1196 集/时（≈54.5 万帧/时）**。

## 2. 已定位的瓶颈（不要重新论证，直接在此之上动手）

**真凶 = `build_map_node` 进程被单线程 ROS2 图像消息序列化钉在 ~1 个 CPU 核上。** 证据链（全部实测，见报告）：

- **不是计算硬墙**：峰值处真实 CPU-busy 仅 50%、GPU 引擎活跃度 7%、显存/IO 余量 20×——没有任何资源被撞到。
- **不是 GPU-context 串行化**（上一轮的头号假设已被**证伪**）：同卡 8 进程吞吐线性涨、GPU 仅 7%；**NVIDIA MPS A/B 收益 0%**。别再往 GPU/MPS 方向想。
- **火焰图直指**：`build_map` 里 **63%** 的时间花在 `sensor_msgs/msg/_image.py:267-268` 的 `Image.data` getter + 生成器表达式（**每帧图像在纯 Python 里逐字节拷贝一遍 CDR (反)序列化**）+ `rclpy` 事件循环 spin。
- **GIL 证据**：`build_map` 每线程 %CPU 的 top/sum = 0.92（一个线程干了几乎所有活，钉在 ~1 核）；`perception` 是真多线程（top/sum=0.06，不是瓶颈）。

**根因诊断**：这是把一个**在线实时** VIO 系统拿来做**离线批量转换**——它用 `BagPlayer` 定速回放 + ROS topic **异步喂帧** + `ApproximateTimeSynchronizer` 近似配对。对离线任务，这条路径既**慢**（每帧的 ROS 序列化 + executor spin）又**不确定**（时序敏感的帧配对，导致逐次不可复现）。**慢和不确定同源。**

## 3. 你要做的（P1 的具体形态）

**把 `build_map` 喂帧路径从"经 ROS topic 异步序列化"改成"进程内直接同步喂帧"。**

核心思路（允许大改，鼓励重写喂帧/回放层）：
- 现状：`BagPlayer` 定速回放 → 序列化成 ROS Image 消息 → 过 DDS → `perception` 反序列化 → 异步配对。
- 目标：直接从 bag reader 取出解码后的帧（numpy 数组），**在一个进程内点对点、同步**地喂给感知/建图逻辑，**绕开 `sensor_msgs/Image` 的逐字节 CDR 序列化和 rclpy executor**。
- 可选更激进：把 perception + build_map 从两进程合成一进程（消除 DDS 往返），但**保持 SLAM 计算逻辑本身不变**。

**不要动的**：`perception_node` / `build_map_node` / `models_trt.py` 里的**VIO 数学**（特征提取、匹配、位姿图、回环、occupancy）。你改的是**数据怎么进来**，不是**算法怎么算**。

## 4. 一个关键的松绑约束（务必利用）

**下游是训练模型，团队已确认可以接受"VIO 逐次抖动"**（抖动在 VIO 自身精度内，等同于传感器噪声）。这意味着：

- **验收标准不是"逐字节一致"**（旧的常驻 worker 就是卡在这个不可能的闸门上才被否决的）。
- **你可以改 VIO 的时序行为**：同步喂帧会得到一条*不同但确定*的轨迹（而非复刻在线版），这**是允许的**。
- 直接同步喂帧顺带把**非确定性也修了**（确定性配对）——这是 P1 的额外红利，但不是硬要求。

## 5. 验收方式（轨迹级容差，不是 hash）

VIO 本就逐次不可复现，所以验收用**轨迹级偏差**，不是逐字节比对：

1. **正确性闸门**：取 N 个代表性集（含 partial-gripper 的），P1 版 vs stock 版各跑，算轨迹 **ATE/RPE**（绝对/相对位姿误差）。判据：P1 与 stock 的偏差落在 **stock 自身逐次重跑的抖动范围内**（先测 stock 重跑 3 次的自抖动作为基线容差）。
2. **触觉/其他通道**：`*_with_pose.mcap` 的触觉、IMU、夹爪、camera_info 通道数量与内容应与 stock 一致（触觉是确定性逐字节插入，必须不变）。
3. **吞吐闸门**：在空闲节点跑 PAR 扫描，确认单节点吞吐较 stock 提升（目标 +25–50%，即 ~1500–1800 集/时）。复用 `profiling/scripts/prof_run_par_sweep.sh` 与 `prof_footprint.sh`。

## 6. 环境与访问

- **节点**：`ssh jhhuang-h200-qinghua-1`（h1）/ `-2`（h2）。**用前先 `nvidia-smi` 看占用，避开别人在跑的卡；饱和扫描需多卡空闲，没条件就等或降规模。**
- **代码在容器里**：`docker exec tinynav_flatbuffer bash -lc "..."`（镜像 `tinynav_flatbuffer_saved`）。工具在 `/tinynav/tool/umi/`。
- **python**：容器内 `python3` 需先 source ROS 才有 rclpy/gtsam/mcap：
  ```bash
  for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
           /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
    [ -f "$f" ] && source "$f"; done
  export PYTHONPATH=/tinynav:$PYTHONPATH
  ```
- **本地→容器部署**：本仓库是本地源码，改完 `docker cp` 进容器验证（见 `deploy.sh`）。VIO 需 GPU，只能在容器里跑。
- **数据**：raw 集在 h1 容器 `/data/plant_collection/raw/26-06-23/dataloop-umi/184/<hash>/episode.mcap`（5021 个）。**务必 `cp` 到 scratch 再跑**，别碰真实产物 `*_with_pose.mcap` 及 `*.pre_tactile.bak` 备份（只读）。
- **无外网**：H200 节点无外网，装包用预下载 wheel + `pip install --no-index --find-links`。

## 7. 起手点（源码入口）

- `_build_map_one_sensor.sh`：单腕入口（起 perception + build_map 两个 tmux pane）。
- `build_map_node.py`（容器 `/tinynav/tinynav/core/`）：`BagPlayer` 回放 + `wait_for_perception_subscribers`（L610，注意它**无超时**）+ `play_next`（L724）+ 喂帧循环。**这是 P1 主战场。**
- `perception_node.py`：`_process_stereo_worker`（L266）、`process`（L504）、主 spin（L712）。
- `flatbuffer_reader.py`（`profiling/` 分析显示 `_generate`/`has_next`/`decode_compressed_video` 占 ~11%）：bag 解码路径，同步喂帧时可直接复用它拿解码后的帧。
- 火焰图热点文件/行号见 `profiling/results/flamegraph_analysis.txt`。

## 8. 硬约束（别踩）

- **不改 VIO 数学**：只改喂帧/回放/进程编排，不动特征/匹配/位姿图/occupancy 算法。
- **绝不碰真实产物**：测试一律用 raw 集的 scratch 副本。5021 个 `*_with_pose.mcap` + `.pre_tactile.bak` 是珍贵数据，只读。
- **共享节点礼仪**：先看 GPU 占用，避开被占的卡，跑完清理（kill 进程/tmux、删 scratch、关自己开的 MPS/容器）。
- **build_map 若保留等待逻辑，必须包超时**（`wait_for_perception_subscribers` 无内置超时，感知一崩就无限占卡；stock `_build_map_one_sensor.sh` 的 Phase-1 版有 `UMI_BUILD_MAP_TIMEOUT_SEC` 看门狗可参考——但本分支从 main 起、不含它，需要就自己加）。
- **提交纪律**：只在被要求时 commit；push 到本分支，别直接推 main。

## 9. 参考物料（都在本分支）

- `VIO转换瓶颈分析报告.md`（根）——权威结论，先读。
- `profiling/docs/VIO流程与瓶颈测试设计.md`——源码级流水线解剖 + 时序模型，P1 必读。
- `profiling/results/flamegraph_analysis.txt`——精确到文件行号的热点。
- `profiling/scripts/`——18 个剖析脚本，可直接复用来验收 P1 的吞吐/足迹（`prof_footprint.sh`、`prof_run_par_sweep.sh`、`prof_flamegraph.sh` 前后对比）。
- `profiling/docs/提速测试报告.md`——**负结果参考**：常驻 worker 为什么失败（native 状态泄漏 + 只摊了启动开销、没碰主导项）。别重蹈覆辙——P1 要直击 63% 的序列化主导项，不是摊销那 ~2s 启动。

## 10. 完成定义（DoD）

- [ ] `build_map` 喂帧路径改为进程内直接同步喂帧，绕开 `sensor_msgs/Image` 逐字节 CDR 序列化（火焰图里那 63% 显著下降，用 `prof_flamegraph.sh` 前后对比证明）。
- [ ] 轨迹级容差闸门通过：P1 vs stock 的 ATE/RPE 落在 stock 自抖动范围内（含 partial-gripper 集）。
- [ ] 触觉/IMU/夹爪/camera_info 通道数量与内容不变。
- [ ] 空闲节点 PAR 扫描证明单节点吞吐提升（目标 +25–50%）。
- [ ] 不改 VIO 数学；真实产物零改动；环境清理干净。
- [ ] 更新 `VIO转换瓶颈分析报告.md` 或新增 P1 实现说明，记录改动、实测收益、验收数据。

---

### 自检（验证环境搭对）
先在容器里跑一集 `profiling/scripts/profile_one.sh <scratch副本> <空闲GPU>`，应得 wall≈77s、C≈235 核-秒。对不上先排查环境（多半是 ROS 没 source 或用错 GPU），再动手。
