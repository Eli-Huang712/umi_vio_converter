#!/usr/bin/env python3
"""prof_parse_stages.py — extract single-episode stage wall-times from the
build/perception logs of ONE wrist run, for the stage gantt (chart #4).

Stages (from log markers verified on this pipeline):
  spawn+init  : run start -> first model 'load ...plan done!'
  model_load  : first -> last 'load ...plan done!' (TRT deserialize x3)
  boot_wait   : last model load -> 'Perception input subscribers are ready'
  per_frame   : subscribers ready -> 'Full mapping data saved successfully'
                (this is the paced playback + per-frame perception, the ~15s)
  mapping     : build_map profiler 'Grand total: X s' (overlaps per_frame tail;
                reported separately as the true mapping-math cost ~1.6s)
Emits JSON to stdout: {stage: seconds, ...} + absolute markers.

Usage: prof_parse_stages.py <build_log> [perception_log]
"""
import sys, re, json, datetime

def ros_ts(line):
    m = re.search(r"\[(\d+\.\d+)\]", line)
    return float(m.group(1)) if m else None

def wall_ts(line):
    m = re.match(r"(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)", line)
    if m:
        return datetime.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S").timestamp()
    return None

def main():
    build = open(sys.argv[1], errors="replace").read().splitlines()
    markers = {}
    model_loads = []      # wall ts of 'load ...plan done!'
    first_ros = last_ros = None
    subs_ready = None
    saved = None
    grand_total = None
    for ln in build:
        r = ros_ts(ln)
        if r is not None:
            if first_ros is None: first_ros = r
            last_ros = r
        if "plan done!" in ln:
            w = wall_ts(ln)
            if w: model_loads.append(w)
        if "subscribers are ready" in ln.lower() or "starting bag playback" in ln.lower():
            subs_ready = subs_ready or ros_ts(ln)
        if "Full mapping data saved successfully" in ln:
            saved = ros_ts(ln)
        m = re.search(r"Grand total:\s*([\d.]+)\s*s", ln)
        if m: grand_total = float(m.group(1))

    out = {}
    if model_loads:
        out["model_load_s"] = round(model_loads[-1] - model_loads[0], 2)
    if subs_ready and saved:
        out["per_frame_perception_s"] = round(saved - subs_ready, 2)
    if grand_total is not None:
        out["mapping_grand_total_s"] = grand_total
    # boot: first ros log -> subscribers ready (spawn + model load + DDS discovery)
    if first_ros and subs_ready:
        out["spawn_to_playback_s"] = round(subs_ready - first_ros, 2)
    out["_markers"] = {
        "n_model_loads": len(model_loads),
        "first_ros": first_ros, "subs_ready": subs_ready, "saved": saved,
    }
    print(json.dumps(out, indent=2))

if __name__ == "__main__":
    main()
