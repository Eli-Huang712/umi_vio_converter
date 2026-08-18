#!/usr/bin/env python3
"""Compare tactile and pose channel counts in a raw MCAP and converted MCAP."""
import argparse
from pathlib import Path

from mcap.reader import make_reader


def chans(p):
    with p.open("rb") as stream:
        s = make_reader(stream).get_summary()
    if s is None or s.statistics is None:
        raise SystemExit(f"MCAP summary/statistics missing: {p}")
    cc = {c.id: c.topic for c in s.channels.values()}
    return {cc[k]: v for k, v in s.statistics.channel_message_counts.items()}


def tac(d):
    return {k: v for k, v in d.items()
            if any(t in k.lower() for t in ("tactile", "touch", "force"))}


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("raw", type=Path)
parser.add_argument("converted", type=Path)
args = parser.parse_args()

for path in (args.raw, args.converted):
    if not path.is_file():
        parser.error(f"not a file: {path}")

raw, wp = chans(args.raw), chans(args.converted)
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
