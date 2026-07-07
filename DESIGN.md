# umi_vio_converter — 设计方案

单集 UMI VIO 转换器。对现有 tinynav 工作流做**手术刀级别**改动，达成两个目标：

1. **保留触觉**：`*_with_pose.mcap` 原生带上 4 路触觉（保持原始 topic 名）。
2. **位姿开关**：可按 `{left_wrist, right_wrist, head}` 逐传感器开/关位姿转换。
3. **幂等 / 补差**：已有 with_pose 产物**不重复转换、不重跑 VIO**，只把缺的触觉补进去；已带触觉的直接跳过。

> 决策已定：触觉保持原 topic 名；范围 = 整条单集流水线（编排 build_map VIO + 转换合并）；head 只留开关、默认关。

---

## 1. 已核实的关键事实（设计前提）

| 事项 | 结论 |
|---|---|
| 真实转换路径 | flatbuffer→CDR **在 `flatbuffer_reader.py`（`FlatbufferBagReader`）就地完成**，被 build_map / convert_mcap_sqlite / merge 三处共用。 |
| `convert_flatbuffer_mcap_to_cdr.py` | 全仓库无人引用 = **死代码**，不在流水线上。改它无效。 |
| `merge_sqlite_mcap.py` | 逐条**原样拷贝** reader 输出 + 按时间插入位姿；本身不解码、不过滤。输入是 raw flatbuffer（被 reader 伪装成 CDR）。 |
| 触觉被丢的位置 | `flatbuffer_reader.canonical_topic()` 对触觉返回 `None`，`open()`/`_generate()` 两处跳过。 |
| 触觉话题 | `/observation/tactile_{left,right}_gripper_finger{0,1}/tactile/tactile_point_cloud2`，schema `discover.TactileData`，flatbuffer，4 路，~492–494 msg/集。 |
| head 可行性 | `make_sensor_topic_map()` 纯字符串参数化，reader 已支持 head 相机/IMU 规范化；仅被 4 处 wrist-only 硬门卡住（build_map_node `:53`/`:466`、convert_mcap_sqlite `:50`、merge `SUPPORTED_SENSORS`）。 |

### TactileData flatbuffer 布局（实测一条真实消息）

- root table 3 字段：`[0] timestamp`（`foxglove.Time` 内联 struct：uint32 sec+nsec，实测 == log_time）、`[1] frame_id`（string，值即完整话题路径）、`[2] points`（**内联 struct 向量**）。
- `TactilePoint` = 24 字节 = 6×float32 `[x, y, z, fx, fy, fz]`（taxel 位置 + 力）。固定 **25 点/条**（5×5 网格），小端连续。
- ⚠️ 现有 codec `vector_bytes()` 把向量头 u32 当**字节长度**读，但 points 是**结构体向量**，头 u32 是**元素个数**(25) → 必须按 `count*24` 切片，不能直接用 `vector_bytes`。

**CDR 类型选定：`sensor_msgs/msg/PointCloud2`**（话题名即 `..._point_cloud2`，数据即带力分量的 3D 点集）。字段 x/y/z/fx/fy/fz 六个 FLOAT32、`point_step=24`、`width=25`、`height=1`；points 的原始字节可**零重排直接拷进** `data`。

---

## 2. 架构与交付物

**部署模型**（与团队既有习惯一致）：源码放本机项目 `datasets/umi_vio_converter/` 版本管理 → 部署进容器 `/tinynav/tool/umi/` 运行（VIO 需 GPU/容器）→ 验证通过后 `docker commit` 持久化。整条流水线在容器内跑（`docker exec`），本机只是源与部署脚本。

```
datasets/umi_vio_converter/
├── DESIGN.md                     # 本文件
├── umi_vio_converter.py          # 主编排 CLI（单集）：三态分派 → build_map ×N / 补触觉 / 跳过
├── backfill_tactile.py           # 补差：把 raw 触觉按 log_time 补进已有 with_pose（纯 CPU、免 GPU/VIO）
├── patched/                      # 对原文件的“手术”拷贝（带清晰改动块标记）
│   ├── flatbuffer_codec.py       #   + TACTILE_TYPE / decode_tactile / vector_struct_bytes
│   ├── flatbuffer_reader.py      #   + include_tactile 开关 + 触觉透传（原名，默认关）
│   └── merge_sqlite_mcap.py      #   + include_tactile=True + 传感器列表化 + head 免 TCP
├── _build_map_one_sensor.sh      # 单传感器 VIO（从 container_all_in_one_parallel.sh 泛化）
├── deploy.sh                     # 备份原文件 → 拷 patched 进容器 →（可选）commit
└── README.md
```

改动**全部向后兼容**：所有新参数默认值 = 今天的行为，build_map / convert_mcap_sqlite 调用点零改动。

---

## 3. 手术改动清单（逐文件、精确到最小编辑）

### 3.1 `flatbuffer_codec.py`（新增，不动老逻辑）

```python
TACTILE_TYPE = "sensor_msgs/msg/PointCloud2"
_TACTILE_POINT_STEP = 24  # 6 x float32

# FlatTable 新增：结构体向量按 count*stride 切片（vector_bytes 只适用于 ubyte 向量）
def vector_struct_bytes(self, index: int, stride: int) -> bytes:
    start = self.vector_start(index)
    if start is None:
        return b""
    count = struct.unpack_from("<I", self.data, start)[0]
    return self.data[start + 4 : start + 4 + count * stride]

def decode_tactile(data: bytes):
    from sensor_msgs.msg import PointCloud2, PointField
    t = FlatTable(data)
    msg = PointCloud2()
    t.read_time(msg.header.stamp, 0)          # 字段0：时间戳
    msg.header.frame_id = t.string(1)          # 字段1：frame_id（原值）
    raw = t.vector_struct_bytes(2, _TACTILE_POINT_STEP)  # 字段2：points
    count = len(raw) // _TACTILE_POINT_STEP
    msg.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z", "fx", "fy", "fz"))]
    msg.height, msg.width = 1, count
    msg.is_bigendian, msg.is_dense = False, True
    msg.point_step, msg.row_step = _TACTILE_POINT_STEP, _TACTILE_POINT_STEP * count
    msg.data = raw
    return msg
```
（顺手把 `"discover.TactileData"` 加进 `TOPIC_TYPE_MAP`/`DECODERS`，让那个离线 converter 保持一致——非必需，仅对齐。）

### 3.2 `flatbuffer_reader.py`（触觉透传，用开关**隔离 VIO**）

关键取舍：**触觉透传只在 merge 需要，build_map/convert 不需要**。若无脑给共享 reader 加触觉，build_map 会白白解码 ~2000 条触觉/集并引入 PointCloud2 依赖。故加 `include_tactile` 开关（默认 `False` = 今天行为），只有 merge 传 `True`：

```python
from tool.umi.flatbuffer_codec import (... , TACTILE_TYPE, decode_tactile)
_TACT_RE = re.compile(r"^/observation/tactile_\w+/tactile/tactile_point_cloud2$")

def canonical_topic(recorded_topic, include_tactile=False):
    ...  # video/imu/joint 原样
    if include_tactile:
        m = _TACT_RE.match(recorded_topic)
        if m:
            return recorded_topic, TACTILE_TYPE, decode_tactile   # 保持原名，不重映射
    return None

class FlatbufferBagReader:
    def __init__(self, include_tactile=False): self._include_tactile = include_tactile; ...
    def open(...):  ... canonical_topic(channel.topic, self._include_tactile) ...

def open_sequential_reader(storage_options, converter_options=None, include_tactile=False):
    ... reader = FlatbufferBagReader(include_tactile) if flatbuffer else rosbag2_py.SequentialReader()
```
- 缺通道自动安全：reader 只遍历包内**实际存在**的 channel，缺的触觉不映射、不报错（覆盖 ~9.6% 残缺集）。
- 触觉走**原名** `/observation/...`，其余仍是 `/robot/...` 规范名（按你的选择，允许这点不一致）。

### 3.3 `merge_sqlite_mcap.py`（1 行触觉 + 位姿列表化）

- reader 打开处 `open_sequential_reader(..., include_tactile=True)` ← **触觉全靠这 1 行接通**。
- 位姿泛化为“传感器列表”，每传感器带一条 TCP 规则：**wrist → 乘 `CAMERA_T_TCP`，head → 原始相机位姿（免 TCP）**。
- `SUPPORTED_SENSORS += "head"`；`build_pose_events` 已是列表循环、`db_dir=None` 自动跳过 → 天然支持“关掉某只手腕”。
- CLI 加 `--pose-sensors lw,rw`（默认）→ 决定读哪些 `_db`、建哪些 `/robot/camera/{s}/left/pose`。空集 = 只转换（带触觉）不加位姿。

### 3.4 `build_map_node.py` —— **默认零改动**

默认只有双腕，build_map 已支持，不碰。head 的 `Literal` 门仅在**显式开 head** 时才需放开，列为**实验性可选补丁**（见 §6），主线不触碰 → 满足“默认关”。

### 3.5 `umi_vio_converter.py`（新主编排，取代 container_all_in_one_parallel 的单集逻辑）

```
umi_vio_converter.py <input.mcap>
    --pose-sensors left_wrist,right_wrist   # 默认；可 "left_wrist" 或 "" 或含 head
    --play-rate 20  --ros-domain-id N  --output <path>  [--no-tcp-transform]
    --mode auto        # auto(默认) | full | backfill | check
    [--force]          # 忽略已有产物，强制全量重转（会覆盖现有 VIO 结果，慎用）
```

**三态自动分派**（`--mode auto`，逐集判定 `wp = <stem>_with_pose.mcap`）：

| 条件 | 动作 | 代价 |
|---|---|---|
| `wp/metadata.yaml` 不存在 | **FULL**：对启用传感器跑 build_map → merge(`include_tactile=True`) | GPU（VIO） |
| `wp` 存在但**无触觉话题** | **BACKFILL**：`backfill_tactile.py`，只补触觉，**不碰 VIO/位姿/其它话题** | 纯 CPU、秒~十几秒/集 |
| `wp` 存在且**已有触觉话题** | **SKIP**：幂等，直接跳过 | 0 |

- 判定“有无触觉”：读 `wp` 的 summary，任一 channel 命中 `_TACT_RE` 即视为已带触觉。
- `--mode check`：只打印每集会走哪条分支、不动数据（先看清全量清单里各多少集要 FULL / BACKFILL / SKIP）。
- `--force` / `--mode full`：无视已有产物重跑全量（会覆盖你现有 VIO 结果，先备份）。
- **FULL 流程**：① source ROS overlays；② 对每个启用传感器调 `_build_map_one_sensor.sh`（复用现有 tmux：perception + build_map 双 pane、wait-for 信号、status 校验）→ 出 `<s>_db/poses.npy`；③ 调 patched merge（`include_tactile=True` + 启用传感器）→ `*_with_pose.mcap`。输出路径/形态与现状完全一致（rosbag2 目录 + `metadata.yaml`），不破坏批处理完成判定与下游读法。

### 3.6 `backfill_tactile.py`（补差：不重转、只补触觉）—— 覆盖你现有的无触觉产物

已有的 `*_with_pose.mcap` 已是 **CDR**（有位姿、无触觉）。补差 = **原样拷贝已有 with_pose + 从 raw 解码触觉、按 log_time 插入**，全程不解码视频、不跑 VIO、不动位姿。

```
backfill_tactile.py --with-pose <stem>_with_pose.mcap --raw <stem>.mcap [--output <tmp>] [--keep-backup]
```

流程：
1. **读已有 with_pose**：用真正的 `rosbag2_py.SequentialReader`（它是 CDR，不是 flatbuffer；⚠️ **不能**走 `open_sequential_reader`——`is_flatbuffer_mcap` 会对 rosbag2 目录 `open()` 报错）。逐条**字节级原样**写进输出（话题/类型/时间戳全不变）。
2. **从 raw 取触觉**：`mcap.reader` 遍历 raw，命中 `_TACT_RE` 的 channel → 复用 §3.1 `decode_tactile()` → `serialize_message` → `(log_time, 原topic, data)`。**只处理 raw 里实际存在的触觉通道**（残缺集自动少补、不报错）。
3. **按时间归并**：把触觉事件按 `log_time` 插进拷贝流（与 merge 的插入逻辑同构），建立触觉 topic（`PointCloud2`，原名）。
4. **安全落地**（产物珍贵，务必非破坏）：先写临时目录 → 校验计数 → 原 `wp` 改名为 `<stem>_with_pose.mcap.pre_tactile.bak` 备份 → 临时目录改名到原位。默认保留备份，人工确认后再删。产物名不变 → 下游/批处理完成判定不受影响。

- **时钟**：raw 与 with_pose 同一 `log_time`，触觉直接对齐、无需重定时。
- **幂等**：`wp` 已含触觉时编排器根本不调用它（SKIP）；单独直调时它也先自检，已存在则拒绝重复插入。
- **可选纯本机变体**：此步无 GPU/VIO 依赖，也可用 `rosbags`(serialize_cdr)+`mcap` 在 macOS 本机批量补差；默认仍走容器内以复用同一份 `decode_tactile`/`sensor_msgs`，与 FULL 路径共享解码代码。

### 3.7 `_build_map_one_sensor.sh` / `deploy.sh`
- 前者 = 把 `run_build_map_for_sensor` 抽成“任意传感器名”的单函数脚本（逻辑与现有逐行等价，只是参数化传感器）。
- 后者 = 先 `docker commit ... backup-$(date +%Y%m%d)` 备份 → `docker cp` patched/* 进 `/tinynav/tool/umi/` → 单集验证 → 通过后更新 latest。

---

## 4. 数据流 / 时序

```
raw.mcap(flatbuffer) ──► FlatbufferBagReader(include_tactile=True)
    ├ video/imu/joint → 解码+重映射 /robot/... (原逻辑)
    ├ camera_info     → 由 MCAP metadata 合成 ~1Hz (原逻辑)
    └ tactile ×4      → decode_tactile→PointCloud2, 保持 /observation/... 原名  [新]
                              │  (每条 log_time / header.stamp 同一时钟，直接对齐)
merge: 逐条原样写 + 按 timestamp 插入 /robot/camera/{s}/left/pose
                              ▼
              <stem>_with_pose.mcap  (视频/IMU/夹爪/camera_info/位姿 不变 + 触觉4路)
```
时间戳一律沿用，不重定时。

---

## 5. head 处理与限制（默认关）
- 结构上 head 已是一等公民（merge/topic-map/reader 字符串通用），位姿走原始相机位姿（无 TCP）。
- 默认 `--pose-sensors` 不含 head → 主线永不触发。
- ⚠️ 开 head 需额外放开 build_map `Literal` 门（实验补丁），且 **head VIO 精度未验证**（perception 外参/内参可能假定手腕几何）——启用会打印 experimental 警告，需实测后才可信。

---

## 6. 验证方案

**A. FULL 路径**（取一个 4 路触觉齐全的 raw 集，如 `.../vio_todo/00e4beb44c8740531f7c0953b93a5064`）跑 patched 流水线，用公共 venv 读 `with_pose`：
- `s.statistics.channel_message_counts`：4 路触觉话题出现，计数与 raw 对应通道一致（492/494/493/493）；抽查 stamp 落原时间窗；随机取一条反序列化 PointCloud2，`width==25`、6 字段、点值与 raw 一致。
- 视频/IMU/夹爪/位姿/camera_info 的话题与计数**与改前的 with_pose 完全一致**（回归）。
- 残缺集（只有 2–3 路）：只写实到的通道、不报错。
- `--pose-sensors left_wrist`：只出左腕位姿；`--pose-sensors ""`：无位姿但有触觉。

**B. BACKFILL 路径**（拿一个**已有的无触觉 with_pose** + 对应 raw）：
- 补差后：触觉 4 路计数与 raw 一致；**位姿话题、位姿逐条内容、视频/IMU/夹爪/camera_info 与补差前逐字节不变**（这是补差不能破坏的核心——可对非触觉话题做逐条 `(topic,timestamp,data)` 哈希比对）。
- 备份 `<stem>_with_pose.mcap.pre_tactile.bak` 存在且可读。
- **幂等**：对已补过触觉的 with_pose 再跑一次 → 判定 SKIP、不重复插入、原文件不变。
- `--mode check` 在全量目录上打印 FULL/BACKFILL/SKIP 计数，与实际磁盘状态吻合。

读法：`/data/shared/tools/ngad_viz/venv/bin/python` + `from mcap.reader import make_reader`。

---

## 7. 集成 / 持久化 / 开放项
- **批处理集成**：现有 `run_vio_batch_retry_parallel.sh` 把 `VIO_RUNNER_IN_CONTAINER` 指向 `umi_vio_converter.py`（或等价 wrapper）即可；`--mode auto` 让它对已完成集自动走 BACKFILL/SKIP，并行/重试/GPU 分配层不动。注意批处理原完成判定是 `wp/metadata.yaml` 存在 → 对旧产物它会“已完成”而跳过；补差要么用编排器的 auto 分派覆盖此判定，要么单独对存量目录跑一轮 BACKFILL。
- **持久化**：`deploy.sh` 备份 commit → 更新 latest（团队习惯）。
- **可视化（可选后续）**：ngad_viz 的 mcap reader 目前分类 `_RAW_TACT` 但无 CDR 触觉解码分支 → with_pose 的触觉暂不显示，不会报错；如需展示另开工单。
- **风险**：head VIO 精度未验证（已默认关）；改 reader 影响所有 consumer——用 `include_tactile` 默认关严格隔离，build_map 行为按位不变（回归项覆盖）；BACKFILL 会重写整包（含视频，纯拷贝）——用“临时目录 + 校验 + 改名 + 保留备份”确保不破坏珍贵 VIO 产物。

---

## 8. 分阶段实施
1. **P1（核心，覆盖需求）**：codec 触觉解码 + reader `include_tactile` 透传 + merge 接线与传感器列表化 + `backfill_tactile.py` + 编排器三态分派 + deploy。默认双腕、带触觉、对存量产物自动补差。跑 §6 A+B 验证。
2. **P2（可选）**：放开 head 实验补丁 + head 位姿实测。
3. **P3（可选）**：ngad_viz 触觉可视化。

**本方案不含任何代码改动**（仅本设计文档）。确认后从 P1 开始实现。
```
