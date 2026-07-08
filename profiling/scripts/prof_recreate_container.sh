#!/usr/bin/env bash
# prof_recreate_container.sh — recreate the h2 tinynav container WITH SYS_PTRACE
# (needed for py-spy flamegraph) and redeploy the umi tool + py-spy. RUN ON h2 HOST.
# The image :latest is the pre-deploy state, so the tool must be re-copied in.
set -euo pipefail
PS=/data/shared/datasets/prof_scratch
DST=/tinynav/tool/umi

docker rm -f tinynav_flatbuffer 2>/dev/null || true
# --shm-size=16g: FastDDS (rmw_fastrtps_cpp) uses a shared-memory transport that
# allocates /dev/shm segments PER participant. Docker's default 64MB /dev/shm is
# exhausted at high PAR (~64+ DDS participants) -> "RTPS_TRANSPORT_SHM Failed to
# create segment" and participant init fails. The stock batch driver hides this
# by RETRYING (wasted GPU/CPU). For clean saturation measurement we give a
# generous /dev/shm (host has 2TB RAM). Pure orchestration change; no VIO edit.
# --cap-add SYS_PTRACE: lets py-spy attach for the flamegraph.
docker run -d --name tinynav_flatbuffer --gpus all --cap-add SYS_PTRACE --shm-size=16g \
  -v /data/shared/datasets:/data -w /tinynav \
  tinynav_flatbuffer_saved:latest tail -f /dev/null
sleep 2

# redeploy tool files (patched overwrite target names + new files)
docker cp "$PS/uvc_deploy/patched/flatbuffer_codec.py"  tinynav_flatbuffer:$DST/flatbuffer_codec.py
docker cp "$PS/uvc_deploy/patched/flatbuffer_reader.py" tinynav_flatbuffer:$DST/flatbuffer_reader.py
docker cp "$PS/uvc_deploy/patched/merge_sqlite_mcap.py" tinynav_flatbuffer:$DST/merge_sqlite_mcap.py
for f in backfill_tactile.py umi_vio_converter.py umi_vio_converter.sh _build_map_one_sensor.sh; do
  docker cp "$PS/uvc_deploy/$f" tinynav_flatbuffer:$DST/$f
done
docker exec tinynav_flatbuffer bash -lc "chmod +x $DST/umi_vio_converter.sh $DST/_build_map_one_sensor.sh"
# reinstall py-spy offline. NB: pip runs INSIDE the container, so use the
# CONTAINER path (/data/prof_scratch = host /data/shared/datasets/prof_scratch
# via the bind mount), NOT the host $PS. docker cp above uses host $PS correctly.
CPS=/data/prof_scratch
docker exec tinynav_flatbuffer bash -lc "/opt/venv/bin/python3 -m pip install --no-index --find-links $CPS $CPS/py_spy-0.4.2-py2.py3-none-manylinux_2_5_x86_64.manylinux1_x86_64.whl 2>&1 | tail -2; which py-spy"
# verify ptrace now works
docker exec tinynav_flatbuffer bash -lc '/opt/venv/bin/python3 -c "import time;[time.sleep(0.05) for _ in range(100)]" & P=$!; sleep 0.5; /opt/venv/bin/py-spy dump --pid $P >/dev/null 2>&1 && echo PTRACE_OK || echo PTRACE_FAIL; kill $P 2>/dev/null'
echo RECREATE_DONE
