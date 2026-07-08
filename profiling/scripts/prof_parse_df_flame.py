#!/usr/bin/env python3
"""Parse a py-spy flamegraph SVG into ranked frames (inclusive%) + estimated self%."""
import re, sys, collections

for tag in ("flame_df", "flame_df_idle"):
    path = f"/data/p1_pool/df_flame/{tag}.svg"
    try:
        txt = open(path).read()
    except FileNotFoundError:
        continue
    titles = re.findall(r"<title>([^<]+)</title>", txt)
    incl = collections.OrderedDict()
    for t in titles:
        m = re.search(r"^(.*?)\s\(([\d,]+) samples?,\s([\d.]+)%\)", t)
        if not m:
            continue
        name = m.group(1).strip()
        n = int(m.group(2).replace(",", ""))
        pct = float(m.group(3))
        # keep max (a frame can appear at several stack positions; title is per-rect inclusive)
        if name not in incl or n > incl[name][0]:
            incl[name] = (n, pct)
    rows = sorted(incl.items(), key=lambda kv: -kv[1][0])
    print(f"===== {tag}: top frames by inclusive samples =====")
    for name, (n, pct) in rows[:40]:
        short = name.split("/")[-1][:95]
        print(f"{pct:5.1f}%  {n:6d}  {short}")
    print()
