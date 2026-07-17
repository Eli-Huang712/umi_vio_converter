# 任务：将 VIO `*_with_pose.mcap` 转换成 LeRobot v3 数据集（保真 + 尽量提速）

> 你是这条数据流水线的下一棒。上一棒把 raw UMI `.mcap` 转成了带位姿轨迹的 `*_with_pose.mcap`（VIO 提速见
> [`VIO转换加速技术报告.md`](VIO转换加速技术报告.md)，600K→1.95M 帧/时 = 3.25×）。**你的活**：把这 5021 个
> `*_with_pose.mcap` 进一步转成 **LeRobot v3.0** 数据集，用于训练。**要求和标准与上一棒一致**：保真、不碰真实产物、
> 共享节点礼仪、用真实利用率量吞吐、正确性优先于速度、在此之上**尽可能优化提速**。

---

## 0. 先做的一件事（step 0，别跳过）

**LeRobot v3.0 的磁盘格式与 API 近期一直在变——不要凭记忆写，先对着当前 `lerobot` 源码/文档核实**再动手：
- v3 结构：`meta/info.json`（`codebase_version="v3.0"`、`fps`、`features` schema、`chunks_size`、`data/`+`video/` 路径模板）、
  `data/`（parquet，**分块多集**，v3 相对 v2 把每集小文件合并成 chunk）、`videos/`（mp4，分块）、`meta/{episodes,tasks,stats}`。
- 创建 API：`LeRobotDataset.create(...)` → 逐帧 `add_frame(frame_dict)` → `save_episode()`；视频在 save 时由
  `encode_episode_videos(...)` 编码；统计由 `compute_episode_stats(...)` 逐帧算。
- 权威来源：`huggingface/lerobot` GitHub（源码为准）、博客 `huggingface.co/blog/lerobot-datasets-v3`、
  `huggingface.co/blog/video-encoding`、DeepWiki `deepwiki.com/huggingface/lerobot`。
- **核实清单**：v3 的 parquet 到底存什么（下面 §2 说存 state/action + VideoFrame 引用、不存像素——确认之）；
  默认视频编码器（下面 §2 说是 libsvtav1/AV1——确认之）；`add_frame` 收 numpy 还是路径；多相机多 video feature 怎么表达。

---

## 1. 输入长什么样（已实测，可直接用）

5021 个产物在 h1 容器：`/data/plant_collection/raw/26-06-23/dataloop-umi/184/<hash>/episode/episode_with_pose.mcap`
（**rosbag2 目录包**：内含 `episode_with_pose.mcap_0.mcap` + `metadata.yaml`，~35MB/集，视频是 **H.264**）。

一集的通道清单（实测，每集 ~11s、~5900 条消息）：

| 通道 | 数量/集 | 类型 | 说明 |
|---|---:|---|---|
| `/robot/camera/{head,left_wrist,right_wrist}/{left,right}/video_encoded` | 147–170 | `foxglove_msgs/CompressedVideo`（H.264） | **6 路立体视频**，~30Hz，640×480 |
| `/robot/camera/.../{left,right}/camera_info` | 6 | `sensor_msgs/CameraInfo` | ~1Hz，内参 |
| `/robot/camera/{left_wrist,right_wrist}/left/pose` | **37–39** | `geometry_msgs/PoseStamped` | **VIO 输出 = world_T_tcp**，**稀疏**（关键帧 ~7.5Hz，只有两只手腕，无 head） |
| `/robot/imu/{head,left_wrist,right_wrist}/data` | ~561 | `sensor_msgs/Imu` | ~96Hz |
| `/robot/{left,right}_gripper/joint_state` | ~582 | `sensor_msgs/JointState` | ~100Hz，夹爪开合 |
| `/observation/tactile_*_gripper_finger{0,1}/...tactile_point_cloud2` | 485 | `sensor_msgs/PointCloud2` | 6×float32 `x,y,z,fx,fy,fz`，触觉 |

**关键设计点（不是简单换容器，是要做时序重采样）**：各流频率不同（视频 30Hz、位姿 **稀疏 7.5Hz**、IMU 96Hz、夹爪 100Hz、
触觉），LeRobot 通常**一帧一行**、其它信号对齐/插值到帧时间戳。**位姿（action/state 的核心信号）比视频帧稀疏得多**——
怎么把它铺到帧时间线上（最近邻？线性插值 SE(3)？）是你要定的保真决策，会直接影响下游训练。想清楚
`observation.state` / `action` 到底放什么（大概率 = 两腕 TCP world 位姿 + 夹爪开合；IMU/触觉作附加 observation feature）。

---

## 2. 已定位的瓶颈（不要从零试，直接在此之上动手）

**真凶 = 视频编码（AV1 re-encode），约占单集处理时间 75%。** 证据链（社区实测）：

- LeRobot issue **#1434**：存一个 15s 集要 30–40s 处理，`encode_episode_videos` **~75%** / `compute_episode_stats` **~25%** /
  parquet 可忽略。
- **默认编码器 = `libsvtav1`（AV1）**，是**故意选的最慢编码**（为训练时解码速度 + 文件小，官方"从未测过编码耗时"）。
- 规模痛点：DROID **92,233 集单机转换 ~7 天**——几乎全花在 AV1 编码上。
- v3 parquet **只存 state/action + VideoFrame 引用（字节/时间戳区间），不存像素** → parquet 极小；stats 是 #2
  （~25%，要逐帧解码采样算 min/max/mean/std）。

**你环境的致命 gotcha（必须知道，否则白忙）**：
- **H100/H200/A100 没有 NVENC 编码硅**（只有解码 NVDEC）。容器 ffmpeg 里 **列出** `h264_nvenc` 不代表能用——
  在 H200 上运行时会失败。所以**在 H200 上视频编码逃不掉是 CPU-bound**。
- 容器现状（实测）：有 `av`(PyAV 17.0)、`numpy`、`huggingface_hub`；**`lerobot`/`datasets`/`pyarrow`/`torch`/
  `torchvision`/`pandas` 全部缺失**（无外网，要离线 wheel 装整套栈——这是第一个工作量）。ffmpeg 有 `libaom-av1`（软件、慢）
  但**没有 `libsvtav1`、没有 `av1_nvenc`**。
- **换到 Blackwell（RTX 5090/B200 等）才有 `av1_nvenc` 硬件编码**——见上一棒报告 §9 的换卡结论，这是弱卡/新卡上编码提速的关键分水岭。

---

## 3. 你要做的（具体形态）

写一个 `mcap → LeRobot v3` 转换器 + 批处理编排：
1. **读 mcap**：复用上一棒的解码原语（`tool/umi/flatbuffer_codec.decode_compressed_video` + PyAV H.264 解码，见
   `direct_feed_build_map.py` 的 `StreamingDecoder`）从 `video_encoded` 拿解码后的 numpy 帧；CDR 反序列化 pose/imu/gripper/tactile。
2. **重采样对齐**：以视频帧为主时间线，把 pose/gripper/imu/tactile 对齐/插值到每帧（§1 的保真决策）。
3. **写 LeRobot v3**：按核实后的 API（`create`/`add_frame`/`save_episode`）落盘；6 路视频 → 多个 video feature；
   state/action/observations 落 parquet。
4. **提速**（核心 KPI）：见 §4 的杠杆。先把 baseline（默认 AV1）测出来，再逐个杠杆上、每步量收益（方法学同上一棒）。

**不要动**：`*_with_pose.mcap` 里的**数据语义**——这是"换格式 + 重采样"，不是"重算"。位姿/触觉/夹爪的**值**要保真透传
（重采样引入的插值误差要可控、可论证）。

---

## 4. 关键杠杆（务必利用，按收益排序）

1. **避免 re-encode（最大一枪）**：源视频**已经是 H.264**。若 LeRobot 允许（配置 `vcodec`/编码参数匹配、或直接 remux
   把 H.264 流**拷进 mp4 容器**不重编码），就能**整块干掉那 75% 的编码开销**。**先验证 LeRobot v3 是否强制 re-encode /
   能否 stream-copy 或接受 h264 源**——这是能否 10× 的分水岭。若不能纯拷贝，退而求其次：**编码器从 AV1 换成 h264**
   （CPU 编码便宜得多；训练侧解码略慢但可接受）。
2. **硬件编码**：H200 无 NVENC（§2）→ 该机只能 CPU 编码；但 **NVDEC 解码可用**（源 H.264 解码走 GPU）。**换 Blackwell
   卡则开 `av1_nvenc`/`h264_nvenc`**，编码搬上 GPU。按报告 §9 的换卡调参法定参。
3. **per-episode 并行**：5021 集天然可并行。用报告已确立的方法找单卡/单机的最优并发度 `N*`——但注意这次多半是
   **CPU-bound（编码）或 IO-bound**，不是 GPU；`N*` 大概率 = `min(核数/每进程核数, 编码线程预算)`。ffmpeg 自身线程数也要调。
4. **流水化**：解码→（重采样）→编码→parquet/stats 分阶段重叠（同上一棒 pipeline 思路），让 CPU 编码期间别闲着解码/IO。
5. **stats 提速**（#2 项，~25%）：若逐帧解码算 stats 太贵，考虑采样帧 / 复用解码结果 / 并行。

**方法学（照搬上一棒，别退化）**：吞吐用 **帧/时**（size-invariant，别用集/时）；利用率用 **cgroup 核-秒 + mpstat %idle +
10Hz `util.gpu`**（不用 loadavg / `utilization.gpu`）；吞吐用**稳态窗口 + 每单元 `timeout` 看门狗**测；**瓶颈会迁移**——
干掉编码后重测，瓶颈会移到解码/重采样/parquet/stats，再攻。

---

## 5. 验收方式（保真闸门，不是逐字节 hash）

参考上一棒的 ATE/parity 双闸门思路，为本任务定义"忠实"：
1. **结构/可加载**：产物能被 `LeRobotDataset(...)` 正常 load + 逐帧迭代；`info.json`/`stats`/`episodes`/`tasks` 齐全、
   `codebase_version=="v3.0"`。
2. **帧计数**：每集每相机的视频帧数 == 源 mcap 对应 `video_encoded` 消息数（或按明确规则重采样后的目标帧数，需可论证）。
3. **值保真**：state/action/pose/gripper/imu/tactile 的值，与源 mcap 反序列化后**在容差内一致**（重采样插值误差要
   量化、可接受）；位姿别丢别错序。
4. **视频保真**：若 stream-copy → 逐帧**像素无损**；若 re-encode → 抽样帧 PSNR/SSIM 在阈值内（且记录 crf/编码器）。
5. **抽样人工核对**：随机几集在 LeRobot 里可视化/播放正常，动作与视频对得上。
6. **成功率**：批量 0 失败（上一棒 P2 教训：别用会静默损坏/丢整批的方案换速度）。

---

## 6. 环境与访问

- **节点**：`ssh jhhuang-h200-qinghua-1`（h1）/ `-2`（h2）。**用前 `nvidia-smi` 看占用**，避开别人在跑的卡。
  （注意：跨天/换 session 后 SSH agent 的 key 可能掉，需重新加载；跳板机 `183.242.150.33` 偶发 banner-exchange 超时，重试即可。）
- **代码在容器**：`docker exec tinynav_flatbuffer bash -lc "..."`（h1/h2 都有该容器）。上一棒工具在 `/tinynav/tool/umi/`。
  **坑**：`cat > /tmp/x.sh` 写在**宿主**上，`docker exec bash -lc "bash /tmp/x.sh"` 找的是**容器内**——脚本要 `docker cp` 进容器或 stdin 灌进去。
- **无外网**：H200 节点无外网。**LeRobot 整套栈（lerobot/datasets/pyarrow/torch/torchvision/pandas）在容器里全缺**，
  要用预下载 wheel 离线装（`pip install --no-index --find-links`）。torch/torchvision 体积大，提前备好对应 CUDA 版本。
- **本机→容器部署**：本机源码，`docker cp` 进容器验证；GPU 只能在容器里用。
- **h2 现状（上一棒留的暖机）**：py-spy 已装（离线 wheel），有 128 集 scratch 池 `/data/p1_pool/raw`（raw 集，不是 with_pose），
  MPS 可用（`prof_mps.sh`）。上一棒的 `prof_*` 剖析脚本在 `feature/p1-direct-feed` / `feature/p2-max-throughput` 分支
  （`delivery` 分支已 ignore 掉）——测吞吐/util 直接复用（`prof_steady_frames.sh` 稳态、`prof_effective_util.sh` 10Hz util 等）。

---

## 7. 起手点（源码入口 + 可复用件）

- **读视频的现成路径**：`tool/umi/flatbuffer_codec.py` 的 `decode_compressed_video`（拿 H.264 bytes + 时间戳）+
  `direct_feed_build_map.py` 的 `StreamingDecoder`（PyAV 后台线程解码，GIL-friendly，可直接搬来做"解码流水"）。
- **读 CDR 消息**：容器内 source ROS 后 `rclpy.serialization.deserialize_message` + `sensor_msgs` 类型；mcap 用 `mcap` 库
  （或 `rosbag2_py.SequentialReader`）。
- **LeRobot 入口**：核实后用 `LeRobotDataset.create/add_frame/save_episode`；关注 `encode_episode_videos`（改编码器/参数的入口）
  与 `compute_episode_stats`。
- **提速方法与调参**：`VIO转换加速技术报告.md`（§2 测量方法学、§9 换卡调参流程 `N* = min(N_gpu, N_cpu, N_vram)`）。

---

## 8. 硬约束（别踩）

- **绝不碰真实产物**：5021 个 `*_with_pose.mcap` + raw 集 + `*.pre_tactile.bak` 全部**只读**，测试一律用 scratch 副本。
- **不改数据语义**：只换格式 + 重采样；位姿/触觉/夹爪的**值**保真，插值误差要可论证。
- **共享节点礼仪**：先看 GPU，避开被占卡，跑完清理（kill 进程/tmux、删 scratch、关自己开的 MPS/容器）。
- **成功率 > 速度**：任何提速方案若引入失败/丢批/静默损坏，一律不采纳（P2 常驻池就是因此被否）。
- **量吞吐用真实指标**：帧/时 + cgroup/mpstat/10Hz util.gpu，别用 loadavg / `utilization.gpu`（不能证明饱和）。
- **H200 无 NVENC 编码**：别指望 `h264_nvenc` 在 H200 上能跑（会失败）；编码在 H200 上就是 CPU-bound。
- **提交纪律**：只在被要求时 commit；push 到自己的分支，别推 main / 别污染 `delivery`。

---

## 9. 参考物料

- [`VIO转换加速技术报告.md`](VIO转换加速技术报告.md)——上一棒的完整加速方案 + **测量方法学（§2）** + **换卡调参法（§9）**，直接复用。
- `direct_feed_build_map.py`（`feature/p2-max-throughput` 分支）——`StreamingDecoder` / `flatbuffer_codec` 解码 H.264 的可复用范式。
- `feature/p1-direct-feed` / `feature/p2-max-throughput` 分支的 `profiling/scripts/prof_*`——吞吐/util/正确性闸门脚本，改一改就能量本任务。
- LeRobot：GitHub `huggingface/lerobot`（源码为准）、issue #1434（编码 75% 实测）、博客 `lerobot-datasets-v3` + `video-encoding`、DeepWiki。

---

## 10. 完成定义（DoD）

- [ ] `*_with_pose.mcap → LeRobot v3.0` 转换器跑通，产物能被 `LeRobotDataset` load + 迭代（§5-1）。
- [ ] 保真闸门通过：帧计数、值容差、视频保真、抽样人工核对（§5）。
- [ ] 时序重采样方案定型并论证（位姿稀疏→帧时间线的对齐/插值，误差可接受）。
- [ ] 提速：给出 baseline（默认 AV1）→ 优化后（首选避免 re-encode / 换 h264 / 硬件编码 / per-episode 并行）的**帧/时**对比，
      标注瓶颈迁移路径；空闲节点 0 失败。
- [ ] 不改数据语义；真实产物零改动；环境清理干净。
- [ ] 写一份实现说明（改动、实测收益、验收数据、换卡调参建议），并给下一棒留接力文档。

---

### 自检（先确认环境搭对，再动手优化）
1. 离线装好 LeRobot 栈，能 `import lerobot`。
2. 挑 1 个 with_pose scratch 副本，转成 1 集 LeRobot v3，`LeRobotDataset(...)` load 回来、迭代无误、视频能播。
3. 抽几帧核对：state/action/pose/gripper 值与源 mcap 反序列化一致（容差内）。
4. 量一集的 baseline 处理耗时并 attribution（编码 vs 解码 vs stats vs parquet），确认编码是否真的主导——**对不上先排查**
   （多半是编码器没按预期用 AV1、或 NVENC 在 H200 上悄悄 fallback/失败），再动手做 §4 的优化。
