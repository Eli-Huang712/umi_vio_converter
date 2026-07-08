#!/usr/bin/env bash
# =============================================================================
# prof_env_probe.sh — read-only environment + tooling probe for the VIO
# profiling task. Run INSIDE the tinynav container. Prints a compact report:
#   - node identity, container, cgroup v2 paths
#   - GPU inventory + current occupancy (who is using what, memory, procs)
#   - CPU inventory + current mpstat %idle (real busy, not loadavg)
#   - profiling tool availability (nsys ncu dcgmi py-spy perf austin mpstat
#     pidstat nvidia-smi dmon) so we know which 4B/4C paths are live
#   - python module availability (psutil, py_spy)
# Writes nothing except stdout. Safe to run on a shared node.
# =============================================================================
set +u
echo "############## prof_env_probe @ $(date '+%F %T %Z') ##############"
echo "== identity =="
echo "hostname=$(hostname)"
echo "uname=$(uname -a)"
echo "in_container=$( [ -f /.dockerenv ] && echo yes || echo unknown )"

echo; echo "== cgroup v2 (for real CPU-busy via cpu.stat) =="
for f in /sys/fs/cgroup/cpu.stat /sys/fs/cgroup/io.stat /sys/fs/cgroup/memory.current /sys/fs/cgroup/cpu.max; do
  if [ -r "$f" ]; then echo "OK   $f"; else echo "MISS $f"; fi
done
echo "-- cpu.max (quota?): $(cat /sys/fs/cgroup/cpu.max 2>/dev/null)"
echo "-- cpu.stat usage_usec: $(awk '/usage_usec/{print $2}' /sys/fs/cgroup/cpu.stat 2>/dev/null)"

echo; echo "== CPU =="
echo "nproc=$(nproc)"
echo "-- mpstat 1s (%idle = real free; loadavg is NOT this) --"
if command -v mpstat >/dev/null 2>&1; then
  mpstat 1 1 2>/dev/null | tail -n 3
else
  echo "mpstat: MISSING"
fi
echo "-- loadavg (record-only, do NOT conclude saturation): $(cat /proc/loadavg)"

echo; echo "== GPU inventory + occupancy =="
nvidia-smi --query-gpu=index,name,memory.total,memory.used,utilization.gpu --format=csv,noheader 2>/dev/null
echo "-- per-process GPU usage (who owns which card / how much mem) --"
nvidia-smi --query-compute-apps=gpu_uuid,pid,used_memory,process_name --format=csv,noheader 2>/dev/null | head -60
echo "-- map uuid->index --"
nvidia-smi --query-gpu=index,uuid --format=csv,noheader 2>/dev/null

echo; echo "== profiling tools availability =="
for t in nsys ncu dcgmi py-spy austin perf mpstat pidstat nvidia-smi tmux; do
  p=$(command -v "$t" 2>/dev/null)
  if [ -n "$p" ]; then echo "HAVE $t -> $p"; else echo "MISS $t"; fi
done
echo "-- nvidia-smi dmon capability (sm/mem via -s ucm) --"
nvidia-smi dmon -c 1 -s ucm 2>&1 | head -4
echo "-- DCGM engine-active fields (1002 gr-active,1003 sm-active,1005 mem-bw) --"
if command -v dcgmi >/dev/null 2>&1; then dcgmi discovery -l 2>&1 | head -4; else echo "dcgmi: MISSING (use nvidia-smi dmon / nsys instead)"; fi
echo "-- perf_event_paranoid (perf record -g needs <=1 or CAP_PERFMON) --"
cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo "n/a"

echo; echo "== python modules =="
PY=/opt/venv/bin/python3
[ -x "$PY" ] || PY=python3
"$PY" - <<'PYEOF' 2>&1
mods = ["psutil", "py_spy", "numpy", "matplotlib"]
for m in mods:
    try:
        __import__(m); print(f"HAVE {m}")
    except Exception as e:
        print(f"MISS {m} ({type(e).__name__})")
PYEOF

echo; echo "== MPS state =="
echo "MPS pipe dir: ${CUDA_MPS_PIPE_DIRECTORY:-<unset>}"
ls -d /tmp/nvidia-mps 2>/dev/null && echo "(an MPS control pipe dir exists)" || echo "(no /tmp/nvidia-mps)"
pgrep -a nvidia-cuda-mps-control 2>/dev/null || echo "no mps-control daemon running"

echo; echo "== scratch/data sanity (read-only) =="
echo "raw root: /data/plant_collection/raw/26-06-23/dataloop-umi/184"
ls /data/plant_collection/raw/26-06-23/dataloop-umi/184 2>/dev/null | head -3
echo "n_raw_hash_dirs=$(ls /data/plant_collection/raw/26-06-23/dataloop-umi/184 2>/dev/null | wc -l)"
df -h /data 2>/dev/null | tail -2
echo "############## PROBE_DONE ##############"
