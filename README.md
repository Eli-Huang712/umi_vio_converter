# UMI VIO Converter

Convert raw UMI `episode.mcap` recordings into ROS 2 MCAP bags containing:

- left/right wrist VIO poses;
- the original four tactile channels;
- camera information and the original non-pose streams;
- the per-episode JSON sidecar when one is present.

The converter supports resumable directory conversion and three per-episode actions:

| Existing output | Action |
|-|-|
| No output | `FULL`: run wrist VIO, then merge poses and tactile data |
| Pose output without tactile | `BACKFILL`: add tactile data without rerunning VIO |
| Complete output | `SKIP` |

## Important scope

This repository is the UMI decoding, orchestration and MCAP merge layer. It is **not a
standalone VIO implementation**. A compatible TinyNav runtime must already provide:

```text
/tinynav/tinynav/core/perception_node.py
/tinynav/tinynav/core/build_map_node.py
/tinynav/tinynav/core/models_trt.py
```

It must also contain the matching TensorRT engines, ROS 2 Humble overlays and Python
packages used by TinyNav (`mcap`, `numpy`, `av`, OpenCV, `rclpy`, `rosbag2_py`,
`cv_bridge`, and the required ROS message packages). GPU-specific engines and models are
deliberately not distributed here.

## Install into a compatible runtime

Run this inside the TinyNav environment, or copy the repository into the container first:

```bash
bash scripts/install.sh /tinynav/tool/umi
```

The installer copies exactly the files listed in [`runtime_files.tsv`](runtime_files.tsv),
sets deterministic permissions, and performs Shell/Python syntax validation. It does not
install system, ROS, CUDA or Python dependencies.

If no ready-to-use image is available, follow
[`docs/DOCKER.md`](docs/DOCKER.md) to prepare a compatible TinyNav base image, add this
repository as an overlay, mount data, and validate the resulting container.

`/tinynav` is the default TinyNav root. Set `TINYNAV_ROOT` only when the compatible
runtime is installed elsewhere. Runtime helpers call each other relative to their
installed directory, so the tool directory itself may be relocated.

## Directory conversion

Input and output must be different directories:

```text
RAW_ROOT/<subset>/<episode-id>/episode.mcap
RAW_ROOT/<subset>/<episode-id>/<episode-id>.json   # optional sidecar
```

Run:

```bash
PAR=48 \
GPUS=0,1,2,3,4,5,6,7 \
bash /tinynav/tool/umi/convert.sh RAW_ROOT OUTPUT_ROOT
```

`GPUS` is a comma-separated list of visible GPU indices; it is not the Docker value
`all`. Start with roughly 4–6 workers per GPU and tune `PAR` for the target machine.
Use only GPUs assigned to your job.

The output mirrors the input tree:

```text
OUTPUT_ROOT/<subset>/<episode-id>/episode_with_pose.mcap/
OUTPUT_ROOT/<subset>/<episode-id>/<episode-id>.json
OUTPUT_ROOT/_vio_logs/results.tsv
OUTPUT_ROOT/_vio_logs/<episode-tag>.log
```

The batch command exits nonzero if any episode still fails after the retry round. Always
inspect `results.tsv` and the final artifacts; a running container or a zero instantaneous
GPU-utilization sample is not an acceptance check.

## Run through Docker

If the tool is installed in an existing GPU container:

```bash
docker exec \
  -e PAR=48 \
  -e GPUS=0,1,2,3,4,5,6,7 \
  YOUR_CONTAINER \
  bash /tinynav/tool/umi/convert.sh /data/raw /data/vio
```

Mounts, container lifecycle, GPU allocation and permissions are deployment concerns and
are intentionally not encoded in this repository.

## Single-episode inspection

Read-only decision check:

```bash
bash /tinynav/tool/umi/umi_vio_converter.sh \
  /data/raw/<subset>/<episode-id>/episode.mcap \
  --mode check \
  --pose-sensors left_wrist,right_wrist
```

See [docs/USAGE.md](docs/USAGE.md) for operational details and
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for code ownership and external boundaries.

## Repository layout

```text
scripts/
  install.sh                  install the runtime files
  convert.sh                  directory conversion entrypoint
  vio_batch_dispatch.sh       concurrency, GPU and ROS-domain dispatcher
  umi_vio_converter.sh        ROS environment wrapper
  _build_map_one_sensor.sh    one-sensor VIO launcher
  *.sh                        batch and maintenance helpers
src/umi_vio_converter/
  umi_vio_converter.py        FULL/BACKFILL/SKIP orchestrator
  direct_feed_build_map.py    in-process video/IMU feed into TinyNav
  flatbuffer_*.py             UMI FlatBuffer decoders
  merge_sqlite_mcap.py        pose/tactile MCAP merge
  backfill_tactile.py         tactile-only backfill
  verify_tactile.py           raw/output tactile inspection
docker/Dockerfile             generic TinyNav-base overlay image
docs/                         usage and architecture documentation
runtime_files.tsv             source-to-runtime installation manifest
```
