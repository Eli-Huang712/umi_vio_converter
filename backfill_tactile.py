#!/usr/bin/env python3
"""Backfill tactile into an existing ``*_with_pose.mcap`` — no VIO rerun.

An existing ``*_with_pose.mcap`` is already a ROS 2 CDR rosbag2 bag with poses
but no tactile. This tool copies it verbatim and inserts the 4 tactile channels
decoded from the matching raw flatbuffer bag, aligned by ``log_time`` (raw and
with_pose share one clock). It never decodes video, never touches poses, and
never runs build_map — pure CPU, seconds per episode.

Non-destructive: writes a fresh bag in a temp dir, verifies, renames the
original aside as ``*.pre_tactile.bak``, then swaps the new bag into place. The
output keeps the exact same name/shape so downstream and batch completion checks
are unaffected.

IMPORTANT: the existing with_pose is CDR, NOT flatbuffer. It is read with a real
``rosbag2_py.SequentialReader`` — do NOT use ``open_sequential_reader`` here, as
``is_flatbuffer_mcap`` would try to ``open()`` the rosbag2 directory and crash.
"""
from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path


TACTILE_TYPE = "sensor_msgs/msg/PointCloud2"
_TACT_RE = re.compile(r"^/observation/tactile_\w+/tactile/tactile_point_cloud2$")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Insert tactile (from raw flatbuffer bag) into an existing *_with_pose.mcap without rerunning VIO."
    )
    parser.add_argument("--with-pose", required=True, type=Path, help="Existing *_with_pose.mcap rosbag2 bag (dir).")
    parser.add_argument("--raw", required=True, type=Path, help="Matching raw flatbuffer .mcap.")
    parser.add_argument("--keep-backup", action="store_true", default=True,
                        help="Keep the *.pre_tactile.bak backup (default: keep).")
    parser.add_argument("--no-keep-backup", dest="keep_backup", action="store_false",
                        help="Delete the backup after a verified swap.")
    parser.add_argument("--force", action="store_true",
                        help="Proceed even if the with_pose already has tactile (rebuilds from raw).")
    return parser.parse_args()


def make_topic_metadata(name: str, msg_type: str):
    import rosbag2_py

    return rosbag2_py.TopicMetadata(
        name=name,
        type=msg_type,
        serialization_format="cdr",
        offered_qos_profiles="",
    )


def existing_tactile_topics(with_pose: Path):
    """Topic names in the with_pose bag that look like tactile channels."""
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(with_pose), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return sorted(t.name for t in reader.get_all_topics_and_types() if _TACT_RE.match(t.name))


def decode_raw_tactile_events(raw_mcap: Path):
    """Return (events, per_topic_counts).

    events: sorted list of (log_time_ns, topic, serialized_cdr_bytes).
    Only tactile channels actually present in the raw bag are decoded, so
    partial episodes (missing a finger/gripper) simply produce fewer topics.
    """
    from mcap.reader import make_reader
    from rclpy.serialization import serialize_message
    from tool.umi.flatbuffer_codec import decode_tactile

    events = []
    counts = {}
    with open(raw_mcap, "rb") as f:
        for _schema, channel, message in make_reader(f).iter_messages():
            if not _TACT_RE.match(channel.topic):
                continue
            msg = decode_tactile(message.data)
            events.append((int(message.log_time), channel.topic, serialize_message(msg)))
            counts[channel.topic] = counts.get(channel.topic, 0) + 1
    events.sort(key=lambda e: (e[0], e[1]))
    return events, counts


def write_merged(with_pose: Path, out_bag: Path, tactile_events, tactile_counts):
    """Copy with_pose verbatim + insert tactile_events by timestamp, into out_bag."""
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(with_pose), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(out_bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    # Recreate every original topic with its original type (verbatim passthrough).
    existing_names = set()
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(make_topic_metadata(topic.name, topic.type))
        existing_names.add(topic.name)

    # Create tactile topics (skip any that somehow already exist under --force).
    for topic in sorted(tactile_counts):
        if topic in existing_names:
            continue
        writer.create_topic(make_topic_metadata(topic, TACTILE_TYPE))

    tactile_index = 0
    copied = 0
    inserted = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        while tactile_index < len(tactile_events) and tactile_events[tactile_index][0] <= timestamp:
            t_ns, t_topic, t_data = tactile_events[tactile_index]
            writer.write(t_topic, t_data, t_ns)
            tactile_index += 1
            inserted += 1
        writer.write(topic, data, timestamp)
        copied += 1

    while tactile_index < len(tactile_events):
        t_ns, t_topic, t_data = tactile_events[tactile_index]
        writer.write(t_topic, t_data, t_ns)
        tactile_index += 1
        inserted += 1

    return copied, inserted


def swap_in(with_pose: Path, out_bag: Path, keep_backup: bool) -> Path:
    """Atomically-ish replace with_pose by out_bag, keeping a .pre_tactile.bak."""
    import os

    backup = with_pose.with_name(with_pose.name + ".pre_tactile.bak")
    if backup.exists():
        raise FileExistsError(f"Backup already exists, refusing to overwrite: {backup}")
    os.replace(with_pose, backup)          # original -> backup
    os.replace(out_bag, with_pose)         # new bag  -> original name
    if not keep_backup:
        shutil.rmtree(backup)
        return None
    return backup


def main():
    args = parse_args()
    with_pose: Path = args.with_pose
    raw: Path = args.raw

    if not (with_pose / "metadata.yaml").exists():
        raise SystemExit(f"Not a rosbag2 bag (no metadata.yaml): {with_pose}")
    if not raw.exists():
        raise SystemExit(f"Raw bag not found: {raw}")

    present = existing_tactile_topics(with_pose)
    if present and not args.force:
        print(f"already_has_tactile={','.join(present)}")
        print("action=skip (idempotent; pass --force to rebuild)")
        return

    tactile_events, tactile_counts = decode_raw_tactile_events(raw)
    if not tactile_counts:
        print("raw_tactile_channels=0")
        print("action=skip (raw bag has no tactile to backfill)")
        return

    # Write into a sibling temp dir so the new bag carries the final name.
    tmp_parent = with_pose.with_name(f".backfill_tmp_{with_pose.name}")
    if tmp_parent.exists():
        shutil.rmtree(tmp_parent)
    tmp_parent.mkdir(parents=True)
    out_bag = tmp_parent / with_pose.name

    try:
        copied, inserted = write_merged(with_pose, out_bag, tactile_events, tactile_counts)
        backup = swap_in(with_pose, out_bag, args.keep_backup)
    finally:
        if tmp_parent.exists():
            shutil.rmtree(tmp_parent, ignore_errors=True)

    print(f"copied_messages={copied}")
    for topic in sorted(tactile_counts):
        print(f"tactile_topic={topic} count={tactile_counts[topic]}")
    print(f"inserted_tactile_messages={inserted}")
    print(f"backup={'(deleted)' if backup is None else backup}")
    print(f"output={with_pose}")


if __name__ == "__main__":
    main()
