#!/usr/bin/env python3
"""prof_steadystate.py — recompute steady-state CPU-busy and GPU sm% from the
raw per-PAR mpstat.txt / dmon.txt, trimming the ramp-up and drain tail.

Why: the sweep's whole-run means are dragged down by (a) startup ramp and
(b) a long drain tail — the episode pool has ~4x size variance, so the final
wave leaves most GPUs idle while a few large episodes finish. For a SATURATION
argument we want utilization during the window when the node is actually full.
Heuristic: keep the middle window, trimming the first FRAC_HEAD and last
FRAC_TAIL of the timeline (tail longer than head because large episodes drain
slowly). Emit both whole-run and steady-state so the report can show both.

Usage: prof_steadystate.py <sweep_out_dir> [frac_head=0.15] [frac_tail=0.30]
Prints one CSV row per par dir: par, cpu_busy_whole, cpu_busy_steady,
gpu_sm_whole, gpu_sm_steady, n_cpu_samples, n_gpu_ticks.
"""
import sys, os, glob, re

def trimmed_mean(vals, fh, ft):
    if not vals: return 0.0
    n = len(vals)
    lo = int(n * fh); hi = int(n * (1 - ft))
    seg = vals[lo:hi] if hi > lo else vals
    return sum(seg) / len(seg) if seg else 0.0

def cpu_busy_series(mpstat_path):
    """ordered list of (100-%idle) from mpstat 'all' rows (%idle = last col)."""
    out = []
    if not os.path.isfile(mpstat_path): return out
    for ln in open(mpstat_path, errors="replace"):
        if re.search(r"\ball\b", ln):
            toks = ln.split()
            try:
                idle = float(toks[-1])
                if 0 <= idle <= 100: out.append(100 - idle)
            except ValueError:
                pass
    return out

def gpu_sm_ticks(dmon_path):
    """mean sm% per timestamp-tick (avg over the 8 GPU rows sharing a tick),
    ordered chronologically. dmon -s u -o T cols: Time gpu sm mem ..."""
    if not os.path.isfile(dmon_path): return []
    per_tick = {}
    order = []
    for ln in open(dmon_path, errors="replace"):
        if ln.lstrip().startswith("#"): continue
        toks = ln.split()
        if len(toks) < 4: continue
        t = toks[0]
        try: sm = float(toks[2])
        except ValueError: continue
        if t not in per_tick:
            per_tick[t] = []; order.append(t)
        per_tick[t].append(sm)
    return [sum(per_tick[t]) / len(per_tick[t]) for t in order]

def main():
    d = sys.argv[1]
    fh = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    ft = float(sys.argv[3]) if len(sys.argv) > 3 else 0.30
    print("par,cpu_busy_whole,cpu_busy_steady,gpu_sm_whole,gpu_sm_steady,n_cpu,n_gpu_ticks")
    for pd in sorted(glob.glob(os.path.join(d, "par*"))):
        m = re.search(r"par(\d+)", os.path.basename(pd))
        if not m: continue
        par = int(m.group(1))
        cpu = cpu_busy_series(os.path.join(pd, "mpstat.txt"))
        gpu = gpu_sm_ticks(os.path.join(pd, "dmon.txt"))
        cw = sum(cpu)/len(cpu) if cpu else 0
        gw = sum(gpu)/len(gpu) if gpu else 0
        cs = trimmed_mean(cpu, fh, ft)
        gs = trimmed_mean(gpu, fh, ft)
        print(f"{par},{cw:.1f},{cs:.1f},{gw:.1f},{gs:.1f},{len(cpu)},{len(gpu)}")

if __name__ == "__main__":
    main()
