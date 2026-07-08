# umi_vio_converter

单集 UMI VIO 转换器：对 tinynav 工作流做**手术刀级别**改动，让 `*_with_pose.mcap`
原生带上触觉，位姿按传感器可开关，且对已有产物**只补差、不重跑 VIO**。

设计细节见 [DESIGN.md](DESIGN.md)。

> **日常操作请看 [使用指南.md](使用指南.md)**（部署/单集/批量/验证/排错）；速度数据见 [BENCHMARK.md](BENCHMARK.md)。

> **性能/吞吐**：瓶颈剖析结论见 [VIO转换瓶颈分析报告.md](VIO转换瓶颈分析报告.md)（真瓶颈 = build_map 单线程 ROS 图像序列化，非 GPU）；
> 剖析全部脚本/图/数据在 [profiling/](profiling/)。当前 `feature/p1-direct-feed` 分支正实现 P1 提速，交接见 [P1_HANDOFF_PROMPT.md](P1_HANDOFF_PROMPT.md)。

## 做了什么（三处外科改动 + 新工具）
- `patched/flatbuffer_codec.py` — 新增 `decode_tactile()` → `sensor_msgs/PointCloud2`
  （6×float32 `x,y,z,fx,fy,fz`，`width=25`，点字节零重排直拷）+ `vector_struct_bytes` 助手。
- `patched/flatbuffer_reader.py` — 新增 `include_tactile` 开关（默认 **False**，build_map/convert
  行为按位不变），开时把触觉按**原 topic 名**透传。
- `patched/merge_sqlite_mcap.py` — reader 打开处 `include_tactile=True`（触觉全靠这一行接通）+
  位姿改成传感器列表（wrist 乘 `CAMERA_T_TCP`，head 出原始相机位姿）+ `--pose-sensors`。
- `backfill_tactile.py` — 已有 with_pose 只补触觉：真 `rosbag2_py.SequentialReader` 原样拷贝 +
  从 raw 解码触觉按 `log_time` 插入；临时目录→校验→改名→保留 `*.pre_tactile.bak` 备份。
- `umi_vio_converter.py` / `.sh` — 三态分派编排；`_build_map_one_sensor.sh` — 单传感器 VIO。

## 三态自动分派（`--mode auto`）
| 产物状态 | 动作 | 代价 |
|---|---|---|
| 无 `wp/metadata.yaml` | FULL：build_map(启用传感器) → merge(带触觉) | GPU |
| `wp` 无触觉话题 | BACKFILL：只补触觉 | 纯 CPU |
| `wp` 已有触觉 | SKIP | 0 |

## 部署（在 H200 host 上）
```bash
rsync -av umi_vio_converter/ h1:/data/shared/tools/umi_vio_converter/
ssh h1
cd /data/shared/tools/umi_vio_converter
bash deploy.sh deploy tinynav_flatbuffer          # 备份镜像 + 备份原文件 + 拷入，不提交 latest
# ... 验证一集（见下）...
bash deploy.sh finalize tinynav_flatbuffer        # 通过后再提交 :latest
bash deploy.sh rollback tinynav_flatbuffer        # 需要时一键回滚
```

## 用法（容器内）
```bash
# 单集：自动决定 FULL / BACKFILL / SKIP
docker exec tinynav_flatbuffer bash tool/umi/umi_vio_converter.sh /data/<hash>.mcap

# 只看会走哪条分支（不动数据）
... umi_vio_converter.sh /data/<hash>.mcap --mode check

# 对已有无触觉产物补差
... umi_vio_converter.sh /data/<hash>.mcap --mode backfill

# 只出左腕位姿 / 只转换不加位姿
... umi_vio_converter.sh /data/<hash>.mcap --pose-sensors left_wrist
... umi_vio_converter.sh /data/<hash>.mcap --pose-sensors ""
```
批处理：把 `run_vio_batch_retry_parallel.sh` 的 `VIO_RUNNER_IN_CONTAINER` 指向
`/tinynav/tool/umi/umi_vio_converter.sh`（`--mode auto` 让存量集自动走 BACKFILL/SKIP）。

## 验证（公共 venv 有 mcap）
```bash
/data/shared/tools/ngad_viz/venv/bin/python - <<'PY'
from mcap.reader import make_reader
s = make_reader(open("<wp>/<...>.mcap","rb")).get_summary()
print(s.statistics.channel_message_counts)   # 应见 4 路触觉，计数与 raw 一致
PY
```
- FULL：触觉 4 路计数 == raw 对应通道；PointCloud2 `width==25`、6 字段；视频/IMU/夹爪/位姿/camera_info 与改前一致。
- BACKFILL：非触觉话题逐条 `(topic,timestamp,data)` 与备份一致；幂等复跑判 SKIP。

## 约束
- head 位姿默认关；显式启用需 `--allow-experimental-head`（且 build_map 的 head 门待放开，P2）。
- 残缺集（缺整只夹爪/单指）只补实际存在的触觉通道、不报错。
- BACKFILL 会重写整包（含视频，纯拷贝）；全程非破坏、保留备份。
