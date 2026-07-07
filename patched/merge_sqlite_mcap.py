#!/usr/bin/env python3
"""Copy an input MCAP and add wrist/head camera pose topics from tinynav *_db folders.

umi_vio_converter surgery (all backward compatible; defaults reproduce the
original behaviour):
  * the input reader is opened with ``include_tactile=True`` so the 4 tactile
    channels flow through to the output on their ORIGINAL topic names — this one
    kwarg is what wires tactile into ``*_with_pose.mcap``;
  * pose sensors are a configurable list via ``--pose-sensors`` (default the two
    wrists), each with its own hand-eye rule: wrists are multiplied by the
    ``CAMERA_T_TCP`` hand-eye matrix, ``head`` has no TCP so it emits the raw
    camera pose.
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import numpy as np


POSE_TOPIC_TYPE = "geometry_msgs/msg/PoseStamped"
SUPPORTED_SENSORS = ("left_wrist", "right_wrist", "head")
# sensor name -> CLI flag overriding its tinynav *_db directory
DB_FLAG = {"left_wrist": "left_db", "right_wrist": "right_db", "head": "head_db"}
# head has no tool-center-point; its pose stays the raw camera pose.
NO_TCP_SENSORS = ("head",)

# cam2tcp hand-eye extrinsic (mirrors cam2imu hardcoded in perception_node.py).
# Output pose becomes TCP in the VSLAM world frame:
#     world_T_tcp = world_T_camera @ CAMERA_T_TCP
# Matches umi_replay convert_rosbag_to_replay_npz.py DEFAULT_CAMERA_T_TCP
# (right-multiplied directly, NOT inverted) so replay output is unchanged.
CAMERA_T_TCP = np.asarray(
    (
        (0.0, -1.0, 0.0, 0.0275),
        (0.0, 0.0, -1.0, 0.036),
        (1.0, 0.0, 0.0, 0.05657),
        (0.0, 0.0, 0.0, 1.0),
    ),
    dtype=float,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Copy an input MCAP and add camera pose topics from tinynav *_db folders (tactile preserved)."
    )
    parser.add_argument("--input_mcap", required=True, type=Path, help="Input MCAP bag path.")
    parser.add_argument(
        "--pose-sensors",
        default="left_wrist,right_wrist",
        help="Comma list of pose sensors to merge (subset of left_wrist,right_wrist,head). "
        "Empty string = convert only (tactile preserved, no pose topics).",
    )
    parser.add_argument("--left_db", type=Path, help="tinynav *_left_wrist_db directory.")
    parser.add_argument("--right_db", type=Path, help="tinynav *_right_wrist_db directory.")
    parser.add_argument("--head_db", type=Path, help="tinynav *_head_db directory.")
    parser.add_argument("--output_mcap", type=Path, help="Output MCAP bag path.")
    parser.add_argument(
        "--no-overwrite",
        dest="overwrite",
        action="store_false",
        help="Fail if the derived output MCAP already exists.",
    )
    parser.add_argument(
        "--camera-tcp-matrix",
        nargs=16,
        type=float,
        metavar="V",
        help="Row-major 4x4 camera_T_tcp matrix. Defaults to the built-in UMI hand-eye matrix.",
    )
    parser.add_argument(
        "--no-tcp-transform",
        dest="tcp_transform",
        action="store_false",
        help="Write raw camera poses (world_T_camera) instead of TCP poses (world_T_tcp).",
    )
    parser.set_defaults(overwrite=True, tcp_transform=True)
    return parser.parse_args()


def resolve_camera_tcp(args) -> np.ndarray | None:
    if not args.tcp_transform:
        return None
    if args.camera_tcp_matrix is not None:
        matrix = np.asarray(args.camera_tcp_matrix, dtype=float).reshape(4, 4)
        if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
            raise ValueError(f"camera_T_tcp last row must be [0,0,0,1], got {matrix[3]}")
        return matrix
    return CAMERA_T_TCP


def default_work_dir(input_mcap: Path):
    return input_mcap.with_suffix("")


def default_output_mcap(input_mcap: Path):
    return default_work_dir(input_mcap) / f"{input_mcap.stem}_with_pose.mcap"


def parse_pose_sensors(value: str):
    sensors = [s.strip() for s in value.split(",")] if value else []
    sensors = [s for s in sensors if s]
    for s in sensors:
        if s not in SUPPORTED_SENSORS:
            raise ValueError(f"Unknown pose sensor {s!r}; supported: {', '.join(SUPPORTED_SENSORS)}")
    return sensors


def resolve_sensor_db(args, work_dir: Path, sensor: str) -> Path:
    override = getattr(args, DB_FLAG[sensor], None)
    return override if override is not None else work_dir / f"{sensor}_db"


def tcp_for_sensor(sensor: str, camera_T_tcp):
    """Per-sensor hand-eye rule: wrists use camera_T_tcp; head stays raw camera pose."""
    if sensor in NO_TCP_SENSORS:
        return None
    return camera_T_tcp


def storage_id_for(path: Path):
    if path.suffix == ".mcap":
        return "mcap"
    return "sqlite3"


def make_topic_metadata(name: str, msg_type: str):
    import rosbag2_py

    return rosbag2_py.TopicMetadata(
        name=name,
        type=msg_type,
        serialization_format="cdr",
        offered_qos_profiles="",
    )


def clone_topic_metadata(topic):
    return make_topic_metadata(topic.name, topic.type)


def infer_sensor_name(db_dir: Path):
    name = db_dir.name
    if not name.endswith("_db"):
        raise ValueError(f"DB directory must end with _db: {db_dir}")

    base = name[: -len("_db")]
    for sensor_name in SUPPORTED_SENSORS:
        if base == sensor_name or base.endswith(f"_{sensor_name}"):
            return sensor_name

    supported = ", ".join(f"*_{sensor}_db" for sensor in SUPPORTED_SENSORS)
    raise ValueError(f"Cannot infer sensor from {db_dir}; expected one of: {supported}")


def load_poses(db_dir: Path):
    poses_path = db_dir / "poses.npy"
    if not poses_path.exists():
        raise FileNotFoundError(f"Missing poses.npy: {poses_path}")

    poses_dict = np.load(poses_path, allow_pickle=True).item()
    if not isinstance(poses_dict, dict) or not poses_dict:
        raise ValueError(f"Expected non-empty timestamp->pose dict in {poses_path}")

    poses = []
    for timestamp_ns in sorted(poses_dict):
        pose = np.asarray(poses_dict[timestamp_ns], dtype=np.float64)
        if pose.shape != (4, 4):
            raise ValueError(f"Pose at timestamp {timestamp_ns} in {poses_path} has shape {pose.shape}, expected (4, 4)")
        poses.append((int(timestamp_ns), pose))
    return poses


def mat3_to_quat_xyzw(R):
    trace = np.trace(R)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    return quat / np.linalg.norm(quat)


def pose_stamped_from_matrix(timestamp_ns: int, pose_matrix: np.ndarray):
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
    msg.header.frame_id = "world"

    msg.pose.position.x = float(pose_matrix[0, 3])
    msg.pose.position.y = float(pose_matrix[1, 3])
    msg.pose.position.z = float(pose_matrix[2, 3])

    qx, qy, qz, qw = mat3_to_quat_xyzw(pose_matrix[:3, :3])
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)
    return msg


def build_pose_events(sensor_specs):
    """sensor_specs: list of (db_dir, expected_sensor, camera_T_tcp_or_None)."""
    from rclpy.serialization import serialize_message

    events = []
    counts = {}

    for db_dir, expected_sensor, camera_T_tcp in sensor_specs:
        if db_dir is None:
            continue

        sensor_name = infer_sensor_name(db_dir)
        if sensor_name != expected_sensor:
            raise ValueError(f"{db_dir} inferred {sensor_name}, but was passed as {expected_sensor}")

        topic = f"/robot/camera/{sensor_name}/left/pose"
        poses = load_poses(db_dir)
        counts[topic] = len(poses)
        for timestamp_ns, pose_matrix in poses:
            if camera_T_tcp is not None:
                pose_matrix = pose_matrix @ camera_T_tcp
            msg = pose_stamped_from_matrix(timestamp_ns, pose_matrix)
            events.append((timestamp_ns, topic, serialize_message(msg)))

    events.sort(key=lambda event: (event[0], event[1]))
    return events, counts


def prepare_output_path(path: Path, overwrite: bool):
    if path.exists():
        if not overwrite:
            raise FileExistsError(f"Output already exists: {path}")
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    path.parent.mkdir(parents=True, exist_ok=True)


def merge_mcap(input_mcap: Path, output_mcap: Path, pose_events, overwrite: bool):
    import rosbag2_py

    if input_mcap.resolve() == output_mcap.resolve():
        raise ValueError("input_mcap and output_mcap must be different paths")

    prepare_output_path(output_mcap, overwrite)

    from tool.umi.flatbuffer_reader import open_sequential_reader

    # include_tactile=True is the one line that carries the 4 tactile channels
    # (as sensor_msgs/PointCloud2, original topic names) into the output.
    reader = open_sequential_reader(
        rosbag2_py.StorageOptions(uri=str(input_mcap), storage_id=storage_id_for(input_mcap)),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
        include_tactile=True,
    )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_mcap), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    created_topics = set()
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(clone_topic_metadata(topic))
        created_topics.add(topic.name)

    pose_topics = sorted({topic for _, topic, _ in pose_events})
    for topic in pose_topics:
        if topic in created_topics:
            raise ValueError(f"Input MCAP already has pose topic: {topic}")
        writer.create_topic(make_topic_metadata(topic, POSE_TOPIC_TYPE))

    pose_index = 0
    copied_count = 0
    written_pose_count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        while pose_index < len(pose_events) and pose_events[pose_index][0] <= timestamp:
            pose_timestamp, pose_topic, pose_data = pose_events[pose_index]
            writer.write(pose_topic, pose_data, pose_timestamp)
            pose_index += 1
            written_pose_count += 1

        writer.write(topic, data, timestamp)
        copied_count += 1

    while pose_index < len(pose_events):
        pose_timestamp, pose_topic, pose_data = pose_events[pose_index]
        writer.write(pose_topic, pose_data, pose_timestamp)
        pose_index += 1
        written_pose_count += 1

    return copied_count, written_pose_count


def main():
    args = parse_args()
    work_dir = default_work_dir(args.input_mcap)
    pose_sensors = parse_pose_sensors(args.pose_sensors)
    output_mcap = args.output_mcap if args.output_mcap is not None else default_output_mcap(args.input_mcap)

    camera_T_tcp = resolve_camera_tcp(args)

    sensor_specs = [
        (resolve_sensor_db(args, work_dir, sensor), sensor, tcp_for_sensor(sensor, camera_T_tcp))
        for sensor in pose_sensors
    ]

    pose_events, pose_counts = build_pose_events(sensor_specs)

    copied_count, written_pose_count = merge_mcap(args.input_mcap, output_mcap, pose_events, args.overwrite)

    print(f"pose_sensors={','.join(pose_sensors) if pose_sensors else '(none)'}")
    print(f"pose_frame={'tcp (world_T_tcp)' if camera_T_tcp is not None else 'camera (world_T_camera)'}")
    print(f"copied_input_messages={copied_count}")
    for topic in sorted(pose_counts):
        print(f"pose_topic={topic} count={pose_counts[topic]}")
    print(f"written_pose_messages={written_pose_count}")
    print(f"output={output_mcap}")


if __name__ == "__main__":
    main()
