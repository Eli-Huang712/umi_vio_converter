# UMI VIO Converter

把 UMI 原始 `episode.mcap` 中的双目视频、IMU 和标定信息送入 TinyNav VIO，分别计算
左右手腕轨迹，再把末端工具中心点（TCP）位姿与原始视频、IMU、夹爪、触觉和任务
sidecar 合并成一个可直接被 ROS 2 / 下游训练读取的 MCAP bag。

```text
raw FlatBuffer MCAP
  ├─ left wrist:  stereo + IMU ──> VIO ──> world_T_camera
  ├─ right wrist: stereo + IMU ──> VIO ──> world_T_camera
  ├─ hand-eye: world_T_camera @ camera_T_tcp ──> world_T_tcp
  └─ video / IMU / gripper / tactile / camera_info / sidecar
                                      │
                                      ▼
                         episode_with_pose.mcap
```

## 1. 它计算的是什么

### 1.1 双目视觉给出几何尺度

对一个传感器在时刻 $t$ 的左右图像 $I_t^L,I_t^R$，先按照时间戳配成立体帧。当前
实现为每个左帧寻找时间差不超过 20 ms 的最近右帧：

$$
j^*(i)=\arg\min_j |t_i^L-t_j^R|,
\qquad |t_i^L-t_{j^*}^R|\le 0.02\ \mathrm{s}.
$$

对应代码是
[`_nearest_right()` / `pair_stereo()`](src/umi_vio_converter/direct_feed_build_map.py)。
TinyNav 的立体网络输出视差 $d$，由焦距 $f$ 和基线 $b$ 恢复深度：

$$
z=\frac{fb}{d},\qquad d>0.
$$

深度把图像特征变成带真实尺度的三维约束；SuperPoint/LightGlue、立体深度网络及其
TensorRT engine 属于外部 TinyNav runtime，本仓库不复制这些模型。

### 1.2 IMU 约束帧间运动

原始 IMU 首先从 IMU 坐标系旋转到相机坐标系。代码使用固定旋转

$$
R_{C\leftarrow I}=
\begin{bmatrix}
0&1&0\\
0&0&-1\\
-1&0&0
\end{bmatrix},
$$

并对角速度、线加速度以及 $3\times3$ covariance 同时应用该旋转，见
[`rotate_imu_inplace()`](src/umi_vio_converter/direct_feed_build_map.py)。

处理时刻为 $T$ 的双目帧前，调度器会依次送入所有 $t_{imu}\le T$ 的测量，再送入第一条
$t_{imu}>T$ 的测量。这个 1-step look-ahead 让 TinyNav 能把最后一段 IMU 积分精确截到
图像时间，而不是因缺少跨越 $T$ 的采样而少积分一段。

双目 VIO 调用按代码中的 0.1333 s 间隔节流，约为 7.5 Hz；IMU 不随图像节流，仍按
时间顺序连续送入。因此视觉状态是稀疏关键帧轨迹，而不是每个视频帧都有一条 pose。

### 1.3 VIO 状态与优化目标

TinyNav 先在相邻关键帧提取并匹配特征，再利用当前帧深度进行几何位姿估计，得到
相对运动 ${}^{C_{k-1}}T_{C_k}$。它给新状态提供初值：

$$
{}^{W}T_{C_k}^{init}={}^{W}T_{C_{k-1}}\,{}^{C_{k-1}}T_{C_k}.
$$

每个关键帧的核心状态可以写成

$$
\mathcal X_k=\left({}^{W}T_{C_k},\ v_k,\ b_k\right),
$$

其中 ${}^{W}T_{C_k}\in SE(3)$ 是相机在 VIO 世界坐标系中的位姿，$v_k$ 是速度，$b_k$
是 IMU bias。兼容 TinyNav core 将帧间 IMU 预积分、双目特征投影约束和先验放进因子图，
求解形式可概括为

$$
\min_{\{T_k,v_k,b_k\}}
\sum_k\|r^{imu}_{k,k+1}\|^2_{\Sigma_{imu}^{-1}}
+\sum_j\|r^{stereo}_j\|^2_{\Sigma_{pixel}^{-1}}
+\sum_k\|r^{prior}_k\|^2.
$$

具体残差、噪声参数和优化器实现在外部
`/tinynav/tinynav/core/perception_node.py` 与 `build_map_node.py`。本仓库不修改 VIO
数学，而是通过
[`direct_feed_build_map.py`](src/umi_vio_converter/direct_feed_build_map.py) 按确定的
帧、IMU 和标定顺序调用这两个 node，最后取得按纳秒时间戳保存的 `poses.npy`。
这里的 `world` 是每次 VIO 自己建立的局部参考系，不应直接解释成机器人 base、场地
地图或跨 episode 共享的绝对世界坐标。

### 1.4 从相机位姿变成机器人 TCP 位姿

TinyNav 输出的是 ${}^{W}T_C$。对左右手腕，本仓库在
[`merge_sqlite_mcap.py`](src/umi_vio_converter/merge_sqlite_mcap.py) 中右乘 UMI 手眼
外参：

$$
{}^{W}T_{TCP}={}^{W}T_C\,{}^{C}T_{TCP},
$$

$$
{}^{C}T_{TCP}=
\begin{bmatrix}
0&-1&0&0.0275\\
0&0&-1&0.036\\
1&0&0&0.05657\\
0&0&0&1
\end{bmatrix}.
$$

平移与旋转矩阵被写成 `geometry_msgs/PoseStamped`，四元数顺序为 `xyzw`，
`frame_id="world"`。默认输出话题为：

```text
/robot/camera/left_wrist/left/pose
/robot/camera/right_wrist/left/pose
```

如需相机位姿而非 TCP 位姿，可对单集入口传 `--no-tcp-transform`。`head` 没有 TCP
变换，但当前 head VIO 未验收，默认拒绝运行。

### 1.5 触觉为什么不参与 VIO

触觉不是本 VIO 状态的一部分。原始 `discover.TactileData` 中每个点已经是连续的
6 个 little-endian `float32`：

```text
[x, y, z, fx, fy, fz]   # 24 bytes / point
```

[`flatbuffer_codec.py`](src/umi_vio_converter/flatbuffer_codec.py) 将这段内存无重排地
写成 `sensor_msgs/PointCloud2`；merge 再按原 `log_time` 把四路触觉插回输出。也就是说，
VIO 只产生 pose，触觉、视频、IMU、夹爪和 camera info 都沿各自原时间线进入结果。

## 2. 代码如何完成一集转换

```text
scripts/convert.sh
  └─ scripts/vio_batch_dispatch.sh             批量队列 / GPU / ROS domain
      └─ scripts/umi_vio_converter.sh          加载 ROS 环境
          └─ umi_vio_converter.py              FULL / BACKFILL / SKIP
              ├─ _build_map_one_sensor.sh      每个手腕运行一次
              │   └─ direct_feed_build_map.py  stereo + IMU -> TinyNav -> poses.npy
              ├─ merge_sqlite_mcap.py          pose + 原始通道 -> 新 MCAP
              └─ backfill_tactile.py           已有 pose 时只补触觉
```

单集编排器根据输出状态选择动作：

| 输出状态 | 动作 | 是否运行 GPU VIO |
|---|---|---:|
| 没有 `metadata.yaml` | `FULL`：双腕 VIO，然后合并 | 是 |
| 已有 pose、没有触觉 | `BACKFILL`：只插入触觉 | 否 |
| pose 和触觉都存在 | `SKIP` | 否 |

因此批量任务可以安全重跑：完成的 episode 会跳过，旧产物缺触觉时不会浪费 GPU
重新计算轨迹。

## 3. 性能优化做了什么

### 3.1 原始路径的瓶颈

原始 TinyNav 路径是两个 ROS 进程：BagPlayer 解码图像，再把 `sensor_msgs/Image`
通过 CDR/DDS 发给 perception，perception 的关键帧又通过 ROS topic 发给 build-map。
大量时间消耗在 Python 图像逐字节搬运、序列化、executor 和 DDS，同一帧被多次复制，
GPU 经常等待上游喂数据。

### 3.2 Direct feed：删除无意义的数据搬运

[`ShimBridge`](src/umi_vio_converter/direct_feed_build_map.py) 用 `_NpImage` 直接持有
NumPy array：

- `imgmsg_to_cv2()` 直接返回原数组；
- perception 的关键帧 publisher 改为进程内捕获；
- 不影响求解的 publisher/TF 改为 no-op；
- `PerceptionNode`、`BuildMapNode` 和模型仍从 TinyNav 原样 import。

优化只改变数据如何到达估计器，不改变视觉、IMU 因子或优化器。

### 3.3 Pipeline decode：让 CPU 解码和 GPU VIO 重叠

原 direct-feed 会先解完整段 H.264，再开始 VIO，解码期间 GPU 空闲。
[`StreamingDecoder`](src/umi_vio_converter/direct_feed_build_map.py) 改为：

```text
后台线程：MCAP -> H.264 / IMU decode ───────────────┐
                                                     ├─ 同时进行
主线程：等待可判定的 stereo/IMU 前缀 -> TRT/GTSAM ─┘
```

在线与离线路径共用同一个最近邻 stereo 配对和 IMU look-ahead；历史正确性闸门中，
pipeline 与 decode-all 路径在 6 个“episode × wrist”样本上 `poses.npy` 逐位一致。

### 3.4 节点级并发：持续队列而不是一次性起一批进程

[`vio_batch_dispatch.sh`](scripts/vio_batch_dispatch.sh) 保持恰好 `PAR` 个 episode
在飞。每个 slot 获得：

$$
GPU(slot)=GPUS[slot\bmod N_{gpu}],\qquad
ROS\_DOMAIN\_ID=DBASE+slot.
$$

它还提供单集 timeout、失败重试、独立日志、源目录 scratch 清理和最终非零退出码。
`DBASE + PAR` 必须不超过 232。

### 3.5 存量数据只做 BACKFILL

已经有 pose 的包如果只缺触觉，会走纯 CPU 的时间序合并，不重新加载模型、不运行
VIO。这不是微优化，而是直接消除不必要的 GPU 工作。

### 3.6 历史 H200 基准

以下数字来自 2026-07-08 的 8×H200、128 episode 稳态测试；“帧”按双腕左目视频
消息计数。它们解释优化方向，不是对其他 GPU 的性能承诺：

| 路径 | 最优 PAR | 帧/时 | 相对 stock |
|---|---:|---:|---:|
| stock ROS/DDS | 64 | 0.60M | 1.00× |
| direct-feed | 64 | 1.61M | 2.69× |
| direct-feed + pipeline | 64 | 1.86M | 3.09× |
| direct-feed + pipeline + NVIDIA MPS | 48 | 1.95M | 3.25× |

MPS 不是本仓库自动管理的功能，也不是所有硬件都应开启：stock 阶段 GPU 尚未成为
瓶颈时 MPS 没有收益；direct-feed 把瓶颈推到 GPU 后才出现收益。换机器必须重新扫描
单卡并发、显存和 `PAR`，不能照搬 H200 的 48/64。

Direct-feed 避免了 stock `stereo_queue(maxsize=1)` 的忙时丢帧，因此它与 stock 不保证
字节级相同。历史轨迹闸门中 4/6 手腕轨迹为亚毫米差异，2 个动态右腕约 13 mm；目标
数据仍应使用真实 FULL canary 做轨迹、topic 和时间戳验收。

## 4. 输入、输出与数据合同

输入树：

```text
RAW_ROOT/<subset>/<episode-id>/episode.mcap
RAW_ROOT/<subset>/<episode-id>/<episode-id>.json   # optional
```

输出树：

```text
OUTPUT_ROOT/<subset>/<episode-id>/episode_with_pose.mcap/
OUTPUT_ROOT/<subset>/<episode-id>/<episode-id>.json
OUTPUT_ROOT/_vio_logs/results.tsv
OUTPUT_ROOT/_vio_logs/<episode-tag>.log
```

输入与输出必须是不同目录。批处理会把 raw episode 同目录下的 `episode/` 视为保留的
转换 scratch，并在每次尝试后清理；不要把其他数据放进这个目录名。验收时仍应比较
raw 文件的大小、mtime 或 SHA，并检查输出 pose/触觉数量及 sidecar。

## 5. 安装与使用

### 5.1 环境边界

本仓库不是独立 VIO 实现。运行环境必须提供：

```text
/tinynav/tinynav/core/perception_node.py
/tinynav/tinynav/core/build_map_node.py
/tinynav/tinynav/core/models_trt.py
```

还需要 ROS 2 Humble、匹配目标 GPU 的 TensorRT engine，以及 `mcap`、`numpy`、`av`、
OpenCV、`rclpy`、`rosbag2_py`、`cv_bridge` 和对应 ROS message。

没有现成镜像时，请从 [`docs/DOCKER.md`](docs/DOCKER.md) 开始配置。

### 5.2 安装到 TinyNav runtime

```bash
bash scripts/install.sh /tinynav/tool/umi
```

安装器严格按照 [`runtime_files.tsv`](runtime_files.tsv) 复制 15 个运行文件并检查
Shell/Python 语法，不会安装 CUDA、ROS、模型或 Python 依赖。

### 5.3 先只读检查一个 episode

```bash
bash /tinynav/tool/umi/umi_vio_converter.sh \
  /data/raw/<subset>/<episode-id>/episode.mcap \
  --mode check \
  --pose-sensors left_wrist,right_wrist
```

这只判断将执行 `FULL`、`BACKFILL` 还是 `SKIP`，不会证明 GPU VIO 能跑通。

### 5.4 转换目录

```bash
PAR=32 \
GPUS=0,1,2,3,4,5,6,7 \
DBASE=100 \
bash /tinynav/tool/umi/convert.sh /data/raw /data/vio
```

- `GPUS` 是逗号分隔的 GPU 编号，不是字符串 `all`；
- `PAR` 是整台节点的 episode 并发数，不是每卡并发数；
- 默认计算 `left_wrist,right_wrist`；
- 批处理默认启用 direct-feed 和 pipeline；
- 任一 episode 在重试后仍失败，批处理最终退出码非零。

Docker 调用示例：

```bash
docker exec \
  -e PAR=32 \
  -e GPUS=0,1,2,3,4,5,6,7 \
  umi-vio \
  bash /tinynav/tool/umi/convert.sh /data/raw /data/vio
```

### 5.5 验收

至少同时检查：

1. 外层退出码；
2. `_vio_logs/results.tsv` 和每集日志；
3. 输出 MCAP 中左右腕 pose topic、四路触觉、原始视频/IMU/夹爪通道；
4. sidecar 是否一致；
5. raw 输入是否没有变化或残留 scratch。

触觉与 pose 数量快速检查：

```bash
python3 /tinynav/tool/umi/verify_tactile.py \
  /data/raw/<subset>/<episode-id>/episode.mcap \
  /data/vio/<subset>/<episode-id>/episode_with_pose.mcap/episode_with_pose.mcap_0.mcap
```

## 6. 仓库结构

```text
scripts/                  Shell 入口、批量调度和维护工具
src/umi_vio_converter/    VIO 数据适配、direct-feed、merge 和触觉代码
docker/Dockerfile         基于 TinyNav base 的通用 overlay 镜像
docs/                     Docker、使用和架构文档
runtime_files.tsv         源码到容器运行目录的安装清单
```

更细的执行边界见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。
