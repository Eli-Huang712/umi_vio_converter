# Usage

## 1. Check the runtime

Before conversion, confirm that the TinyNav core, ROS overlays and Python dependencies
described in the repository README are available. For a GPU run, also check that the
selected GPU indices are visible and idle.

## 2. Install

```bash
bash install.sh /tinynav/tool/umi
```

All runtime files are installed into one directory because the converter resolves its
helper scripts and Python modules relative to its own location. If TinyNav itself is not
installed at `/tinynav`, export `TINYNAV_ROOT=/absolute/path/to/tinynav` when running the
converter.

## 3. Inspect one episode without writing

```bash
bash /tinynav/tool/umi/umi_vio_converter.sh \
  /data/raw/subset/episode-id/episode.mcap \
  --mode check \
  --pose-sensors left_wrist,right_wrist
```

The reported action is one of `FULL`, `BACKFILL`, or `SKIP`.

## 4. Convert a directory

```bash
PAR=24 \
GPUS=0,1,2,3 \
DBASE=100 \
bash /tinynav/tool/umi/convert.sh /data/raw /data/vio
```

Options are passed through environment variables:

| Variable | Meaning | Default |
|-|-|-|
| `PAR` | Total concurrent episodes | `32` |
| `GPUS` | Comma-separated GPU indices | `0,1,2,3,4,5,6,7` |
| `DBASE` | First ROS domain ID | `100` |
| `POSE_SENSORS` | Pose sensors | `left_wrist,right_wrist` |
| `TIMEOUT` | First-pass per-episode timeout in seconds | `900` |
| `RETRY_TIMEOUT` | Retry timeout in seconds | `1800` |

The dispatcher requires `DBASE + PAR <= 232`.

## 5. Verify results

Check all three layers:

1. process exit code;
2. `_vio_logs/results.tsv` and per-episode logs;
3. output MCAP topics/counts and the copied JSON sidecar.

For tactile counts:

```bash
python3 /tinynav/tool/umi/verify_tactile.py \
  /data/raw/subset/episode-id/episode.mcap \
  /data/vio/subset/episode-id/episode_with_pose.mcap/episode_with_pose.mcap_0.mcap
```

Raw inputs should remain unchanged. The dispatcher removes only the per-episode scratch
directory it creates next to `episode.mcap`.

## Maintenance helpers

- `failed_manifest.sh`: summarize episodes without products.
- `backfill_sidecar_json.sh`: copy existing JSON sidecars into an output tree.
- `batch_backfill.sh`: tactile-only backfill for existing pose bags.
- `clean_raw_scratch.sh`: explicitly remove converter scratch directories; review its
  target root and subset list before use.
