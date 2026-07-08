#!/usr/bin/env python3
"""prof_analyze_flame.py — summarize a py-spy flamegraph SVG: top frames by
sample count (the hot spots), so the per-process critical path can be named
without eyeballing the SVG. py-spy embeds samples in <title> like:
  funcname (file.py:line) (N samples, X.XX%)

Usage: prof_analyze_flame.py <flame.svg>
"""
import sys, re
svg = open(sys.argv[1], errors="replace").read()
titles = re.findall(r"<title>([^<]+)</title>", svg)
rows = []
for t in titles:
    m = re.search(r"\((\d+) samples,\s*([\d.]+)%\)", t)
    if not m:
        continue
    label = re.sub(r"\s*\(\d+ samples,[^)]*\)", "", t).strip()
    rows.append((int(m.group(1)), float(m.group(2)), label))
rows.sort(reverse=True)
print(f"# {sys.argv[1]}  total_frames={len(rows)}")
seen = set()
shown = 0
for n, pct, label in rows:
    if label in seen:
        continue
    seen.add(label)
    print(f"{pct:6.2f}%  {n:6d}  {label}")
    shown += 1
    if shown >= 30:
        break
