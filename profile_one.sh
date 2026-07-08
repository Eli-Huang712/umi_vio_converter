#!/usr/bin/env bash
# =============================================================================
# profile_one.sh — rigorous per-episode resource footprint for the roofline.
#
# Runs ONE episode FULL conversion (sequential, 1 GPU) and measures:
#   C  = CPU core-seconds (cgroup v2 cpu.stat usage_usec delta) — the DECIDING
#        number: C≈581 => CPU hard wall at 192 cores/1190 ep/hr; C<<581 => soft
#        bottleneck (headroom exists).
#   per-process CPU% over time (pidstat) — distinguishes single-thread serial
#        bottleneck (soft) from genuinely parallel CPU work.
#   GPU util/mem sampled on the target GPU (integrate -> GPU-seconds G). NOTE:
#        only clean if the target GPU is otherwise idle; under co-tenant load
#        this is contaminated (report as such).
#   IO bytes (container cgroup io.stat delta) and input/output sizes.
#   phase wall times (perception boot / build / mapping) from logs.
#
# Usage:  profile_one.sh <raw episode.mcap> <gpu_id> [out_prefix]
# Env:    UMI_PERCEPTION_WARMUP_SEC (default 3)
# Runs inside the tinynav container. Source ROS is done here.
# =============================================================================
set +u

RAW="${1:?usage: profile_one.sh <episode.mcap> <gpu_id> [prefix]}"
GPU="${2:?need gpu_id}"
PREFIX="${3:-/data/prof}"
WARMUP="${UMI_PERCEPTION_WARMUP_SEC:-3}"
CG=/sys/fs/cgroup            # cgroup v2 root for this container
OUT="${PREFIX}"; mkdir -p "$OUT"

for f in /opt/ros/humble/setup.bash /3rdparty/ros2_ws/install/local_setup.bash \
         /3rdparty/plotjuggler_ws/install/local_setup.bash /3rdparty/message_filters_ws/install/local_setup.bash; do
  [ -f "$f" ] && source "$f"
done
export PYTHONPATH=/tinynav:${PYTHONPATH:-}

read_cpu_usec() { awk '/usage_usec/{print $2}' "$CG/cpu.stat"; }
read_io_bytes() { awk -F'[ =]' '{for(i=1;i<=NF;i++){if($i=="rbytes")r+=$(i+1); if($i=="wbytes")w+=$(i+1)}} END{print r+0, w+0}' "$CG/io.stat" 2>/dev/null; }

echo "=== profiling episode: $RAW on GPU $GPU ==="
echo "raw_size_bytes=$(stat -c %s "$RAW")"

# GPU sampler (background): index,util,mem every 0.5s
( while :; do
    ts=$(date +%s.%N)
    line=$(nvidia-smi --id="$GPU" --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits 2>/dev/null | tr -d ' ')
    echo "$ts $line"
    sleep 0.5
  done ) > "$OUT/gpu_samples.txt" 2>/dev/null &
GPU_SAMPLER=$!

# pidstat: per-process %CPU every 1s for perception+build_map (background)
pidstat -u -h -C "perception_node|build_map_node" 1 > "$OUT/pidstat.txt" 2>/dev/null &
PIDSTAT=$!

# ---- bracket the whole 2-wrist episode ----
CPU0=$(read_cpu_usec); read IOR0 IOW0 < <(read_io_bytes); T0=$(date +%s.%N)

for sensor in left_wrist right_wrist; do
  CUDA_VISIBLE_DEVICES="$GPU" UMI_PERCEPTION_WARMUP_SEC="$WARMUP" \
    bash /tinynav/tool/umi/_build_map_one_sensor.sh "$RAW" "$sensor" 140 20 > "$OUT/${sensor}_run.log" 2>&1
done

T1=$(date +%s.%N); CPU1=$(read_cpu_usec); read IOR1 IOW1 < <(read_io_bytes)

kill "$GPU_SAMPLER" "$PIDSTAT" 2>/dev/null

# ---- results ----
WALL=$(awk -v a=$T0 -v b=$T1 'BEGIN{printf "%.1f", b-a}')
CPU_S=$(awk -v a=$CPU0 -v b=$CPU1 'BEGIN{printf "%.1f", (b-a)/1e6}')
echo "=================== RESULTS ==================="
echo "wall_s          = $WALL      (2 wrists sequential)"
echo "cpu_core_seconds= $CPU_S     <-- C (deciding: 581=CPU wall @192core/1190eph)"
awk -v c=$CPU_S -v w=$WALL 'BEGIN{printf "avg_cores_used  = %.2f      (C/wall; >1 = multi-core)\n", c/w}'
awk -v c=$CPU_S 'BEGIN{printf "CPU-roofline    = 192/C = %.0f ep/hr (if CPU-bound this = ceiling)\n", 192*3600/c}'
echo "io_read_MB      = $(awk -v a=$IOR0 -v b=$IOR1 'BEGIN{printf "%.0f",(b-a)/1048576}')"
echo "io_write_MB     = $(awk -v a=$IOW0 -v b=$IOW1 'BEGIN{printf "%.0f",(b-a)/1048576}')"
# GPU-seconds: integrate util fraction over samples (dt=0.5)
awk 'NF>=2{split($2,a,","); u=a[1]; s+=u/100*0.5; n++} END{printf "gpu_busy_seconds= %.1f      (integral util*dt; CONTAMINATED if GPU shared)\n", s}' "$OUT/gpu_samples.txt"
awk 'NF>=2{split($2,a,","); if(a[1]+0>mx)mx=a[1]; if(a[2]+0>mm)mm=a[2]} END{printf "gpu_util_peak   = %s%%   gpu_mem_peak= %s MiB\n", mx, mm}' "$OUT/gpu_samples.txt"
# per-process peak CPU%
echo "--- per-process peak %CPU (pidstat; >100 = multi-thread) ---"
awk 'NR>3 && $0 ~ /(perception_node|build_map_node)/{cpu=$8; cmd=$NF; if(cpu>mx[cmd])mx[cmd]=cpu} END{for(c in mx) printf "  %-20s peak %%CPU = %s\n", c, mx[c]}' "$OUT/pidstat.txt" 2>/dev/null || echo "  (pidstat parse: see $OUT/pidstat.txt)"
echo "PROFILE_DONE"
