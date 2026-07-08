#!/usr/bin/env python3
"""prof_count_frames.py — count stereo camera frames per raw episode (pure
CPU/IO, no GPU, no VIO) so throughput can be expressed per-FRAME, not just
per-episode (episodes vary ~4x in size). Uses the mcap summary statistics
(channel_message_counts) — no full decode needed. Reports, per episode:
  size_MB, n_frames (per stereo camera), duration_s, and derived fps.

A UMI episode's real work scales with FRAMES (per-frame perception dominates),
so frames is the size-invariant unit. Frame count = messages on ONE wrist
camera's video_encoded channel (one message = one encoded frame).

Usage: prof_count_frames.py <raw_root> [n_episodes]
Run inside the tinynav container (needs python-mcap; source ROS first).
"""
import sys, os, glob
from mcap.reader import make_reader

root = sys.argv[1]
N = int(sys.argv[2]) if len(sys.argv) > 2 else 30
eps = sorted(glob.glob(os.path.join(root, "*", "episode.mcap")))
# spread across size range
eps_by_size = sorted(eps, key=lambda p: os.path.getsize(p))
if len(eps_by_size) > N:
    step = len(eps_by_size) / N
    eps = [eps_by_size[int(i*step)] for i in range(N)]
else:
    eps = eps_by_size

# which channel is a per-frame stereo camera stream (one msg per frame).
# Raw flatbuffer topic for the left-wrist left camera H.264 stream:
CAM_HINT = "coracam_lefthand/left_h264/video"
print("size_MB,n_frames,duration_s,fps")
rows = []
for ep in eps:
    try:
        with open(ep, "rb") as f:
            summ = make_reader(f).get_summary()
        if summ is None:
            continue
        # map channel_id -> topic
        topic_by_ch = {cid: ch.topic for cid, ch in summ.channels.items()}
        stats = summ.statistics
        counts = stats.channel_message_counts if stats else {}
        # frame count = messages on the left_wrist left camera channel
        nframes = 0
        for cid, c in counts.items():
            if CAM_HINT in topic_by_ch.get(cid, ""):
                nframes = c; break
        dur = (stats.message_end_time - stats.message_start_time) / 1e9 if stats else 0
        mb = os.path.getsize(ep) / 1e6
        fps = nframes / dur if dur else 0
        print(f"{mb:.1f},{nframes},{dur:.1f},{fps:.1f}")
        rows.append((mb, nframes, dur))
    except Exception as e:
        print(f"# {os.path.basename(os.path.dirname(ep))}: {type(e).__name__} {e}")

if rows:
    import statistics as st
    mbs=[r[0] for r in rows]; fr=[r[1] for r in rows]
    print(f"# n={len(rows)} size_MB[min/med/max]={min(mbs):.0f}/{st.median(mbs):.0f}/{max(mbs):.0f}"
          f" frames[min/med/max]={min(fr)}/{int(st.median(fr))}/{max(fr)}"
          f" frames_per_MB_median={st.median([r[1]/r[0] for r in rows if r[0]]):.1f}")
