#!/usr/bin/env python3
"""Verify tactile channels survived FULL conversion (raw vs with_pose)."""
import sys
from mcap.reader import make_reader


def chans(p):
    s = make_reader(open(p, "rb")).get_summary()
    cc = {c.id: c.topic for c in s.channels.values()}
    return {cc[k]: v for k, v in s.statistics.channel_message_counts.items()}


def tac(d):
    return {k: v for k, v in d.items()
            if any(t in k.lower() for t in ("tactile", "touch", "force"))}


raw, wp = chans(sys.argv[1]), chans(sys.argv[2])
rt, wt = tac(raw), tac(wp)

print("=== RAW tactile channels ===")
for k in sorted(rt):
    print(f"  {k} = {rt[k]}")
print("=== WITH_POSE tactile channels ===")
for k in sorted(wt):
    print(f"  {k} = {wt[k]}")
match = all(rt.get(k) == wt.get(k) for k in rt) and len(rt) == len(wt)
print(f"--- tactile: raw={len(rt)} ch, wp={len(wt)} ch, counts_match={match} ---")

print("=== WITH_POSE pose channels ===")
for k in sorted(wp):
    if "pose" in k.lower():
        print(f"  {k} = {wp[k]}")
print(f"--- total channels: raw={len(raw)} wp={len(wp)} ---")
