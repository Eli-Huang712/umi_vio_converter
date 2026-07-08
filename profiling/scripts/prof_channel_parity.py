#!/usr/bin/env python3
"""prof_channel_parity.py — verify *_with_pose.mcap non-pose channels are unchanged.

The P1 correctness contract (handoff §5.2): tactile / IMU / gripper / camera_info
channels in ``*_with_pose.mcap`` must match stock in count AND content. merge only
inserts pose topics (from poses.npy) and passes everything else through verbatim
from the raw bag via flatbuffer_reader, so those channels should be byte-identical
regardless of which build_map path produced poses.npy. This checks that empirically.

For every channel: report per-run message count. For NON-pose channels, also hash
the concatenated message payloads (sorted by log_time) and compare. Pose channels
(``/robot/camera/*/left/pose``) are expected to differ (that IS the VIO output) and
are reported count-only.

Usage: prof_channel_parity.py <stock_with_pose.mcap> <df_with_pose.mcap>
"""
from __future__ import annotations

import argparse
import glob
import hashlib
import os
import sys
from collections import defaultdict

from mcap.reader import make_reader

POSE_SUFFIX = "/left/pose"


def _resolve_mcaps(path):
    """A with_pose 'mcap' is a rosbag2 DIRECTORY bag (metadata.yaml + inner *.mcap).
    Accept either a directory bag (read all inner *.mcap) or a bare .mcap file."""
    if os.path.isdir(path):
        inner = sorted(glob.glob(os.path.join(path, "*.mcap")))
        if not inner:
            raise FileNotFoundError(f"No inner *.mcap in bag dir {path}")
        return inner
    return [path]


def scan(path):
    """Per topic: (count, ORDER-INDEPENDENT multiset digest of SEMANTIC content).

    We deserialize each message to its ROS type and hash ``(log_time, str(msg))``,
    then digest the sorted multiset. Comparing DESERIALIZED field values (not raw
    CDR bytes) is immune to two nondeterminisms that make a raw-byte hash useless
    here: (1) FastCDR alignment/padding bytes come from non-zeroed buffers, so the
    SAME message content serializes to slightly different bytes run-to-run (this
    fails stock against itself); (2) mcap writer chunk/tie ordering. Sorting the
    multiset also removes reader ordering. So a difference here means real field
    values changed.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    counts = defaultdict(int)
    hashes = {}
    per_topic = defaultdict(list)
    type_cache = {}
    for mcap_path in _resolve_mcaps(path):
        with open(mcap_path, "rb") as f:
            for schema, channel, message in make_reader(f).iter_messages():
                type_name = schema.name.replace("/msg/", "/").replace("/", "/msg/", 1) \
                    if "/msg/" not in schema.name else schema.name
                cls = type_cache.get(type_name)
                if cls is None:
                    cls = get_message(type_name)
                    type_cache[type_name] = cls
                msg = deserialize_message(message.data, cls)
                sig = hashlib.sha1(str(msg).encode()).hexdigest()
                per_topic[channel.topic].append((int(message.log_time), sig))
    for topic, items in per_topic.items():
        counts[topic] = len(items)
        items.sort()  # (log_time, semantic sig) — order-independent multiset
        h = hashlib.sha1()
        for lt, s in items:
            h.update(str(lt).encode())
            h.update(s.encode())
        hashes[topic] = h.hexdigest()
    return counts, hashes


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("stock")
    ap.add_argument("df")
    args = ap.parse_args()

    cs, hs = scan(args.stock)
    cd, hd = scan(args.df)
    topics = sorted(set(cs) | set(cd))

    print(f"{'topic':60s} {'stock':>7} {'df':>7}  status")
    ok = True
    for t in topics:
        sc, dc = cs.get(t, 0), cd.get(t, 0)
        is_pose = t.endswith(POSE_SUFFIX)
        if is_pose:
            status = "POSE (count-only; expected to differ)"
        elif sc != dc:
            status = "COUNT_MISMATCH"
            ok = False
        elif hs.get(t) != hd.get(t):
            status = "CONTENT_MISMATCH"
            ok = False
        else:
            status = "identical"
        print(f"{t:60s} {sc:>7} {dc:>7}  {status}")

    print()
    print("PARITY_OK" if ok else "PARITY_FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
