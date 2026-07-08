#!/usr/bin/env python3
"""prof_plots.py — render all 7 profiling charts from the collected CSVs/JSON.

Runs INSIDE the container (matplotlib present). Reads whatever inputs exist
under --indir and writes PNGs to --outdir. Missing inputs -> that chart is
skipped with a note (so it degrades gracefully if an experiment was cut short).

Inputs (all optional):
  footprint.csv        (prof_footprint.sh)   -> feeds roofline + stage gantt fallbacks
  sweep_results.csv    (prof_par_sweep.sh)   -> throughput/util vs PAR (+ historical)
  conc_out/results.csv (prof_gpu_concurrency.sh) -> concurrency signature + MPS A/B
  stages.json          (prof_parse_stages.py)-> single-episode stage gantt
  roofline.json        (hand/derived)        -> ceiling values

Usage: prof_plots.py --indir /data/prof_scratch --outdir /data/prof_scratch/profiling_out
"""
import argparse, csv, json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- historical PAR sweep from 提速测试报告.md §2 (validated, h1) ----
HIST_PAR = [
    # PAR, throughput ep/hr, per-ep latency s, loadavg/192, nvidia-smi util%
    (1,   46,   78,  None, None),
    (8,   275,  99,  0.17, 12),
    (24,  714,  120, 0.52, 61),
    (32,  883,  126, 0.38, 43),
    (48,  1077, 158, 0.56, 96),
    (64,  1188, 193, 0.92, 80),
]

def read_csv(path):
    if not os.path.isfile(path): return []
    with open(path) as f:
        return list(csv.reader(f))

def parse_kv_rows(path, tag):
    """rows like 'SWEEP_ROW,par=80,...' -> list of dict."""
    out = []
    if not os.path.isfile(path): return out
    for line in open(path):
        line = line.strip()
        if not line.startswith(tag): continue
        d = {}
        for tok in line.split(",")[1:]:
            if "=" in tok:
                k, v = tok.split("=", 1)
                d[k] = v
        out.append(d)
    return out

def savefig(fig, outdir, name):
    p = os.path.join(outdir, name)
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"WROTE {p}")

# placeholder — chart functions appended below

def chart_roofline(indir, outdir, sweep_rows):
    rf = os.path.join(indir, "roofline.json")
    if not os.path.isfile(rf):
        print("skip roofline: no roofline.json"); return
    R = json.load(open(rf))
    fig, ax = plt.subplots(figsize=(9, 5.2))
    walls = {
        "CPU-compute ceiling": (R.get("cpu_ceiling_eph"), "#2ca02c"),
        "GPU-compute ceiling": (R.get("gpu_ceiling_eph"), "#1f77b4"),
        "IO-bandwidth wall":   (R.get("io_ceiling_eph"),   "#9467bd"),
    }
    labels, vals, colors = [], [], []
    for k, (v, c) in walls.items():
        if v: labels.append(k); vals.append(v); colors.append(c)
    y = list(range(len(labels)))
    ax.barh(y, vals, color=colors, alpha=0.85)
    for yi, v in zip(y, vals):
        ax.text(v, yi, f" {v:.0f}", va="center", fontsize=11, fontweight="bold")
    obs = R.get("observed_plateau_eph")
    if obs:
        ax.axvline(obs, color="crimson", ls="--", lw=2)
        ax.text(obs, len(labels)-0.35, f"observed plateau {obs:.0f} ep/hr",
                color="crimson", rotation=90, va="top", ha="right", fontsize=10, fontweight="bold")
    ax.set_yticks(y); ax.set_yticklabels(labels)
    ax.set_xlabel("throughput ceiling (episodes / hour)")
    ax.set_title("Roofline: where the 1190 ep/hr plateau sits vs each resource ceiling")
    ax.grid(axis="x", alpha=0.3)
    savefig(fig, outdir, "1_roofline.png")

def _sweep_points(sweep_rows):
    """merge historical + newly measured PAR points -> sorted [(par,thr,cpu%,gpu%)]."""
    pts = {}
    for p, thr, lat, load, util in HIST_PAR:
        pts[p] = {"thr": thr, "cpu": None, "gpu": None, "lat": lat, "src": "hist"}
    for d in sweep_rows:
        try: p = int(d["par"])
        except: continue
        pts[p] = {
            "thr": float(d.get("throughput_eph", 0)),
            "cpu": float(d["cpu_busy_pct"]) if d.get("cpu_busy_pct") else None,
            "gpu": float(d["gpu_sm_pct"]) if d.get("gpu_sm_pct") else None,
            "lat": None, "src": "measured",
        }
    return [(k, pts[k]) for k in sorted(pts)]

def chart_throughput_vs_par(indir, outdir, sweep_rows):
    pts = _sweep_points(sweep_rows)
    if not pts: print("skip throughput: no data"); return
    par = [k for k, _ in pts]
    thr = [v["thr"] for _, v in pts]
    fig, ax = plt.subplots(figsize=(9, 5.2))
    ax.plot(par, thr, "o-", color="#1f77b4", lw=2, label="throughput (ep/hr)")
    for _, v in pts:
        pass
    # marginal gain per added worker on secondary axis
    ax2 = ax.twinx()
    mg_x, mg_y = [], []
    for i in range(1, len(pts)):
        p0, p1 = par[i-1], par[i]
        if p1 > p0:
            mg_x.append((p0+p1)/2); mg_y.append((thr[i]-thr[i-1])/(p1-p0))
    ax2.plot(mg_x, mg_y, "s--", color="#ff7f0e", alpha=0.7, label="marginal ep/hr per +worker")
    ax2.axhline(0, color="grey", lw=0.8, ls=":")
    ax2.set_ylabel("marginal gain (ep/hr per added worker)", color="#ff7f0e")
    ax.set_xlabel("PAR (concurrent episodes across 8 GPUs)")
    ax.set_ylabel("throughput (episodes / hour)", color="#1f77b4")
    ax.set_title("Throughput vs PAR (+ marginal gain): where the plateau saturates")
    ax.grid(alpha=0.3)
    # mark measured (new) points
    for k, v in pts:
        if v["src"] == "measured":
            ax.annotate("measured", (k, v["thr"]), textcoords="offset points",
                        xytext=(0, 10), fontsize=8, color="green")
    savefig(fig, outdir, "2_throughput_vs_par.png")

def _read_steadystate(indir):
    """optional steadystate.csv from prof_steadystate.py -> {par: {...}}."""
    import csv as _csv
    p = os.path.join(indir, "steadystate.csv")
    out = {}
    if not os.path.isfile(p): return out
    for row in _csv.DictReader(open(p)):
        try: out[int(row["par"])] = {k: float(row[k]) for k in
              ("cpu_busy_whole","cpu_busy_steady","gpu_sm_whole","gpu_sm_steady")}
        except (KeyError, ValueError): pass
    return out

def chart_util_vs_par(indir, outdir, sweep_rows):
    pts = [(k, v) for k, v in _sweep_points(sweep_rows) if v["cpu"] is not None or v["gpu"] is not None]
    if not pts: print("skip util_vs_par: no measured real-utilization points"); return
    ss = _read_steadystate(indir)
    par = [k for k, _ in pts]
    # whole-run (from sweep rows) and steady-state (from steadystate.csv if present)
    cpu_w = [v["cpu"] for _, v in pts]
    gpu_w = [v["gpu"] for _, v in pts]
    cpu_s = [ss.get(k, {}).get("cpu_busy_steady") for k in par]
    gpu_s = [ss.get(k, {}).get("gpu_sm_steady")  for k in par]
    have_ss = any(x is not None for x in cpu_s)
    fig, ax = plt.subplots(figsize=(9.2, 5.4))
    if have_ss:
        ax.plot(par, cpu_s, "o-", color="#2ca02c", lw=2.4, label="CPU %busy — steady-state")
        ax.plot(par, gpu_s, "s-", color="#1f77b4", lw=2.4, label="GPU sm% — steady-state")
        ax.plot(par, cpu_w, "o:", color="#2ca02c", lw=1, alpha=0.5, label="CPU %busy — whole-run (ramp-diluted)")
        ax.plot(par, gpu_w, "s:", color="#1f77b4", lw=1, alpha=0.5, label="GPU sm% — whole-run (ramp-diluted)")
    else:
        ax.plot(par, cpu_w, "o-", color="#2ca02c", lw=2, label="real CPU %busy (100-mpstat %idle)")
        ax.plot(par, gpu_w, "s-", color="#1f77b4", lw=2, label="real GPU engine active (dmon sm%)")
    ax.axhline(100, color="grey", ls=":", lw=0.8)
    ax.set_ylim(0, 105)
    ax.set_xlabel("PAR (concurrent episodes across 8 GPUs)")
    ax.set_ylabel("real utilization (%)")
    ax.set_title("Real CPU-busy vs GPU-engine-active vs PAR\n(neither is loadavg/util; steady-state trims ramp+drain)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    savefig(fig, outdir, "3_util_vs_par.png")

def chart_stage_gantt(indir, outdir):
    sf = os.path.join(indir, "stages.json")
    if not os.path.isfile(sf): print("skip gantt: no stages.json"); return
    S = json.load(open(sf))
    # ordered non-overlapping stages for ONE wrist (seconds)
    seq = []
    if "spawn_to_playback_s" in S:
        # split spawn_to_playback into (spawn+init) and (model_load) if we have model_load
        ml = S.get("model_load_s", 0)
        boot = max(S["spawn_to_playback_s"] - ml, 0)
        seq.append(("spawn + DDS discovery + CUDA/ROS init", boot, "#8c564b"))
        if ml: seq.append(("TRT model load (x3 .plan)", ml, "#e377c2"))
    if "per_frame_perception_s" in S:
        seq.append(("per-frame perception (paced playback)", S["per_frame_perception_s"], "#ff7f0e"))
    fig, ax = plt.subplots(figsize=(10, 3.4))
    start = 0.0
    for name, dur, col in seq:
        ax.barh(0, dur, left=start, color=col, edgecolor="white", label=f"{name} ({dur:.1f}s)")
        if dur > 1.5:
            ax.text(start + dur/2, 0, f"{dur:.1f}s", ha="center", va="center", color="white", fontsize=9, fontweight="bold")
        start += dur
    # overlay mapping-math (subset of per-frame tail)
    gt = S.get("mapping_grand_total_s")
    ann = f"   (of which real mapping math = {gt:.1f}s)" if gt else ""
    ax.set_yticks([]); ax.set_xlabel("seconds (one wrist, sequential)")
    ax.set_title("Single-wrist stage breakdown" + ann)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.25), ncol=2, fontsize=8)
    ax.set_xlim(0, start*1.02)
    savefig(fig, outdir, "4_stage_gantt.png")

def chart_concurrency(indir, outdir, conc_rows):
    """chart #5: same-GPU concurrency signature — per-proc latency + sm% + throughput vs N."""
    base = [d for d in conc_rows if d.get("mps") != "on"]
    if not base: print("skip concurrency: no conc results"); return
    def num(d, k):
        try: return float(d[k])
        except: return None
    base.sort(key=lambda d: int(d["N"].split("=")[-1]) if "=" in d.get("N","") else int(d.get("N",0)))
    N   = [int(d["N"].split("=")[-1]) if "=" in d["N"] else int(d["N"]) for d in base]
    lat = [num(d, "med_proc_wall_s") for d in base]
    sm  = [num(d, "sm_avg_pct") for d in base]
    thr = [num(d, "wrists_per_hr") for d in base]
    fig, ax = plt.subplots(figsize=(9.5, 5.4))
    ax.plot(N, sm, "s-", color="#1f77b4", lw=2, label="aggregate GPU sm% (engine active)")
    ax.set_ylim(0, 105); ax.set_ylabel("GPU sm% / (latency & thr scaled)", color="#1f77b4")
    # latency inflation vs ideal (flat) and throughput, normalized to right axis
    ax2 = ax.twinx()
    ax2.plot(N, lat, "o--", color="#d62728", lw=2, label="per-proc latency (s)")
    ax2.plot(N, thr, "^--", color="#2ca02c", lw=2, label="throughput (wrists/hr)")
    ax2.set_ylabel("per-proc latency (s)  /  throughput (wrists/hr)")
    ax.set_xlabel("N concurrent build_map processes on ONE GPU")
    ax.set_title("Same-GPU concurrency: sm% vs per-proc latency vs throughput\n"
                 "(serialization ⇒ flat sm% + linear latency + flat throughput)")
    l1, la1 = ax.get_legend_handles_labels(); l2, la2 = ax2.get_legend_handles_labels()
    ax.legend(l1+l2, la1+la2, loc="upper left", fontsize=8)
    ax.grid(alpha=0.3)
    savefig(fig, outdir, "5_gpu_concurrency_signature.png")

def chart_mps_ab(indir, outdir, conc_rows):
    """chart #7: MPS A/B bars at fixed N — throughput + sm% off vs on."""
    def key(d):
        n = d["N"].split("=")[-1] if "=" in d.get("N","") else d.get("N","")
        return (int(n), d.get("mps")=="on")
    pairs = {}
    for d in conc_rows:
        try: n = int(d["N"].split("=")[-1]) if "=" in d["N"] else int(d["N"])
        except: continue
        pairs.setdefault(n, {})[d.get("mps")=="on"] = d
    ready = {n: v for n, v in pairs.items() if True in v and False in v}
    if not ready: print("skip mps: need both off+on at same N"); return
    n = sorted(ready)[-1]  # highest N with a pair
    off, on = ready[n][False], ready[n][True]
    def f(d, k):
        try: return float(d[k])
        except: return 0
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.5, 4.6))
    xs = ["MPS off", "MPS on"]
    thr = [f(off, "wrists_per_hr"), f(on, "wrists_per_hr")]
    sm  = [f(off, "sm_avg_pct"), f(on, "sm_avg_pct")]
    a1.bar(xs, thr, color=["#999999", "#2ca02c"])
    for i, v in enumerate(thr): a1.text(i, v, f"{v:.0f}", ha="center", va="bottom", fontweight="bold")
    a1.set_title(f"throughput (wrists/hr), N={n}/GPU"); a1.set_ylabel("wrists/hr")
    if thr[0] > 0:
        a1.text(0.5, max(thr)*0.5, f"{(thr[1]/thr[0]-1)*100:+.0f}%", ha="center", color="crimson", fontsize=14, fontweight="bold")
    a2.bar(xs, sm, color=["#999999", "#1f77b4"])
    for i, v in enumerate(sm): a2.text(i, v, f"{v:.0f}%", ha="center", va="bottom", fontweight="bold")
    a2.set_title("GPU sm% (engine active)"); a2.set_ylabel("sm%"); a2.set_ylim(0, 105)
    fig.suptitle("NVIDIA MPS A/B — does sharing one GPU context beat time-slicing?")
    savefig(fig, outdir, "7_mps_ab.png")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--indir", default="/data/prof_scratch")
    ap.add_argument("--outdir", default="/data/prof_scratch/profiling_out")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    sweep_rows = parse_kv_rows(os.path.join(a.indir, "sweep_out", "sweep_results.csv"), "SWEEP_ROW")
    conc_rows  = parse_kv_rows(os.path.join(a.indir, "conc_out", "results.csv"), "RESULT_ROW")
    chart_roofline(a.indir, a.outdir, sweep_rows)
    chart_throughput_vs_par(a.indir, a.outdir, sweep_rows)
    chart_util_vs_par(a.indir, a.outdir, sweep_rows)
    chart_stage_gantt(a.indir, a.outdir)
    chart_concurrency(a.indir, a.outdir, conc_rows)
    chart_mps_ab(a.indir, a.outdir, conc_rows)
    print("PLOTS_DONE (flamegraph #6 is py-spy SVG, copied separately)")

if __name__ == "__main__":
    main()

