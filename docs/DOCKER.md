# Docker 环境配置

## 1. 先明确镜像分层

完整环境由两层组成：

```text
TinyNav 基础镜像
  CUDA / TensorRT / ROS 2 Humble / Python 依赖
  TinyNav core / 模型 / 对应 GPU 的 TensorRT engine
        +
本仓库 overlay
  UMI 解码 / VIO 调度 / MCAP 合并 / 触觉回填
```

本仓库不包含 TinyNav core、模型或 TensorRT engine，因此不能单独从空白 Ubuntu
镜像构建完整 VIO 环境。没有现成镜像时，应先使用有授权的 TinyNav 源码和模型资产
制作基础镜像，再安装本仓库。

## 2. 基础镜像最低要求

在安装本仓库前，基础镜像应满足：

- 宿主机已安装 Docker Engine、NVIDIA 驱动和 NVIDIA Container Toolkit；
- 执行用户可以直接运行 `docker info`，无需临时切换到个人管理员账户；
- 容器内 `nvidia-smi -L` 能看到目标 GPU；
- `/tinynav/tinynav/core/` 中存在 `perception_node.py`、`build_map_node.py` 和
  `models_trt.py`；
- ROS 2 Humble及项目使用的 ROS overlay 已安装；
- Python 能导入 `mcap`、`numpy`、`av`、`cv2`、`rclpy`、`rosbag2_py`、
  `cv_bridge` 及所需 ROS message；
- `tmux`、`timeout`、`find`、`awk`、`md5sum` 等命令可用；
- TensorRT engine 与目标 GPU、TensorRT/CUDA 版本匹配，并且能实际反序列化。

不要仅因为两张卡的计算架构相同就直接认定 TensorRT engine 可复用；必须在目标机器
实际加载并运行样本。

## 3. 将转换器加入基础镜像

仓库提供 [`docker/Dockerfile`](../docker/Dockerfile)。它使用多阶段构建，只把
manifest 中的 15 个运行文件放进最终镜像，不会把 Git 历史、文档或本地数据留在
镜像层中。

使用固定基础镜像 digest 构建，避免 `latest` 漂移：

```bash
BASE_IMAGE='tinynav-runtime@sha256:<base-image-digest>'
IMAGE="umi-vio:$(git rev-parse --short=12 HEAD)"

docker image inspect "$BASE_IMAGE"
docker run --rm --gpus all --entrypoint nvidia-smi "$BASE_IMAGE" -L

docker build \
  --build-arg TINYNAV_BASE="$BASE_IMAGE" \
  -f docker/Dockerfile \
  -t "$IMAGE" .
```

## 4. 创建容器并挂载数据

Docker bind mount 只能访问创建容器时明确挂载的宿主目录。先选择一个用户都有权限的
绝对路径，再将它挂到容器 `/data`：

```bash
HOST_DATA=/absolute/path/to/shared-data
IMAGE=umi-vio:<git-sha>

test -d "$HOST_DATA"
docker run -d \
  --name umi-vio \
  --gpus all \
  --shm-size=16g \
  --restart unless-stopped \
  --mount type=bind,src="$HOST_DATA",dst=/data \
  --workdir /tinynav \
  --entrypoint tail \
  "$IMAGE" -f /dev/null
```

`HOST_DATA` 的读取和写入权限仍由宿主文件系统控制。需要共享使用时，应先配置好该目录
的用户组和权限；Docker 不会自动绕过宿主权限，也不能访问未挂载的其他宿主路径。

## 5. 验证容器

先检查静态环境：

```bash
docker ps --filter name=umi-vio
docker exec umi-vio nvidia-smi -L
docker exec umi-vio bash -lc '
  test -f /tinynav/tool/umi/convert.sh
  test -f /tinynav/tinynav/core/perception_node.py
  python3 -c "import mcap, numpy, av, cv2, rclpy, rosbag2_py, cv_bridge"
'
```

然后用一个独立的小样本目录运行 FULL 转换：

```bash
docker exec \
  -e PAR=1 \
  -e GPUS=0 \
  umi-vio \
  bash /tinynav/tool/umi/convert.sh /data/canary_raw /data/canary_vio
```

验收时同时检查退出码、`_vio_logs/results.tsv`、每集日志、输出 MCAP 的 pose/触觉
topic 和 JSON sidecar。`docker ps` 显示 `Up` 或 `--mode check` 成功，都不能替代真实
GPU FULL 样本验证。

## 6. 日常使用

```bash
docker exec \
  -e PAR=32 \
  -e GPUS=0,1,2,3,4,5,6,7 \
  umi-vio \
  bash /tinynav/tool/umi/convert.sh /data/my_raw /data/my_vio
```

`GPUS` 是逗号分隔的 GPU 编号，不是字符串 `all`。输入和输出必须使用不同目录。
