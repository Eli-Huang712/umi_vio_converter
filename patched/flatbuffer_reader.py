#!/usr/bin/env python3
"""Read a rollio flatbuffer MCAP as if it were a pre-converted CDR ``/robot`` bag.

``FlatbufferBagReader`` is a duck-typed stand-in for ``rosbag2_py.SequentialReader``
(``open`` / ``get_all_topics_and_types`` / ``has_next`` / ``read_next``). It hides
three differences between a raw rollio recording and the CDR bag the tinynav
pipeline expects:

  1. messages are flatbuffer-encoded  -> decode with ``flatbuffer_codec``;
  2. topics use rollio names          -> remap to ``/robot/...`` canonical names;
  3. camera intrinsics live in MCAP    -> synthesize ``sensor_msgs/CameraInfo``
     ``camera_info`` metadata             messages at ~1 Hz (matching the CDR bag).

Downstream consumers (``build_map_node.BagPlayer``, ``convert_mcap_sqlite``) then
see CDR-serialized ``CompressedVideo`` / ``Imu`` / ``CameraInfo`` messages on the
canonical topics, so their existing decode/route logic is unchanged.

umi_vio_converter surgery: an opt-in ``include_tactile`` flag (default ``False``)
adds ``discover.TactileData`` passthrough as ``sensor_msgs/PointCloud2`` on the
ORIGINAL ``/observation/...`` topic name. It defaults off so build_map / convert
behave bit-for-bit as before; only the merge step opts in.
"""
from __future__ import annotations

import json
import re

from tool.umi.flatbuffer_codec import (
    IMU_TYPE,
    JOINT_STATE_TYPE,
    TACTILE_TYPE,
    VIDEO_TYPE,
    decode_compressed_video,
    decode_imu,
    decode_joint_states,
    decode_tactile,
)

CAMERA_INFO_TYPE = "sensor_msgs/msg/CameraInfo"

# rollio camera device location -> canonical sensor name used by make_sensor_topic_map
_LOCATION_TO_SENSOR = {
    "head": "head",
    "lefthand": "left_wrist",
    "righthand": "right_wrist",
}

_VIDEO_RE = re.compile(r"^/camera/coracam_(?P<loc>\w+)/(?P<side>left|right)_h264/video$")
_IMU_RE = re.compile(r"^/observation/imu_(?P<sensor>\w+)/imu/imu_accel_gyro$")
_JOINT_RE = re.compile(r"^/observation/(?P<arm>\w+)_gripper/gripper/joint_position$")
# --- umi_vio_converter surgical addition ---------------------------------
_TACT_RE = re.compile(r"^/observation/tactile_\w+/tactile/tactile_point_cloud2$")
# -------------------------------------------------------------------------

# Republish camera_info this often, matching the ~1 Hz cadence of the CDR bags.
_CAMERA_INFO_PERIOD_NS = 1_000_000_000


def canonical_topic(recorded_topic: str, include_tactile: bool = False):
    """Map a recorded rollio topic to ``(canonical_topic, ros_type, decoder)``.

    Returns ``None`` for topics the pipeline does not consume. Tactile is only
    mapped when ``include_tactile`` is set, and then it keeps its ORIGINAL topic
    name (no ``/robot`` remap), per team decision.
    """
    m = _VIDEO_RE.match(recorded_topic)
    if m:
        sensor = _LOCATION_TO_SENSOR.get(m.group("loc"))
        if sensor is None:
            return None
        canonical = f"/robot/camera/{sensor}/{m.group('side')}/video_encoded"
        return canonical, VIDEO_TYPE, decode_compressed_video

    m = _IMU_RE.match(recorded_topic)
    if m:
        return f"/robot/imu/{m.group('sensor')}/data", IMU_TYPE, decode_imu

    m = _JOINT_RE.match(recorded_topic)
    if m:
        return f"/robot/{m.group('arm')}_gripper/joint_state", JOINT_STATE_TYPE, decode_joint_states

    # --- umi_vio_converter surgical addition ------------------------------
    if include_tactile:
        m = _TACT_RE.match(recorded_topic)
        if m:
            # keep the original topic name; do not remap to /robot
            return recorded_topic, TACTILE_TYPE, decode_tactile
    # ----------------------------------------------------------------------

    return None


def _open_path(storage_options):
    """Accept a rosbag2 StorageOptions-like object or a bare path string."""
    return getattr(storage_options, "uri", storage_options)


def is_flatbuffer_mcap(path: str) -> bool:
    """True if any channel in the MCAP is flatbuffer-encoded."""
    from mcap.reader import make_reader

    with open(path, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        channels = summary.channels.values() if summary is not None else []
        for channel in channels:
            if channel.message_encoding == "flatbuffer":
                return True
        if summary is not None:
            return False
        # No summary section: fall back to sniffing the first message.
        for _, channel, _ in make_reader(f).iter_messages():
            return channel.message_encoding == "flatbuffer"
    return False


def _build_camera_info(spec: dict):
    """Build a ``sensor_msgs/CameraInfo`` from one camera_info metadata JSON value."""
    from sensor_msgs.msg import CameraInfo, RegionOfInterest

    msg = CameraInfo()
    msg.header.frame_id = spec.get("frame_id", "")
    msg.height = int(spec.get("height", 0))
    msg.width = int(spec.get("width", 0))
    msg.distortion_model = spec.get("distortion_model", "")
    msg.d = [float(v) for v in spec.get("d", [])]
    msg.k = [float(v) for v in spec.get("k", [0.0] * 9)]
    msg.r = [float(v) for v in spec.get("r", [0.0] * 9)]
    msg.p = [float(v) for v in spec.get("p", [0.0] * 12)]
    msg.binning_x = int(spec.get("binning_x", 0))
    msg.binning_y = int(spec.get("binning_y", 0))
    roi = spec.get("roi", {})
    region = RegionOfInterest()
    region.x_offset = int(roi.get("x_offset", 0))
    region.y_offset = int(roi.get("y_offset", 0))
    region.height = int(roi.get("height", 0))
    region.width = int(roi.get("width", 0))
    region.do_rectify = bool(roi.get("do_rectify", False))
    msg.roi = region
    return msg


class FlatbufferBagReader:
    """Duck-typed stand-in for ``rosbag2_py.SequentialReader`` over flatbuffer MCAP."""

    def __init__(self, include_tactile: bool = False):
        self._path = None
        self._include_tactile = include_tactile
        self._topic_map = {}          # recorded_topic -> (canonical, ros_type, decoder)
        self._topic_types = {}        # canonical_topic -> ros_type
        self._camera_infos = []       # list[(canonical_topic, CameraInfo)]
        self._generator = None
        self._peeked = None
        self._exhausted = False

    # -- rosbag2 SequentialReader subset -----------------------------------

    def open(self, storage_options, converter_options=None):
        from mcap.reader import make_reader

        self._path = _open_path(storage_options)
        with open(self._path, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            channels = summary.channels.values() if summary is not None else []
            for channel in channels:
                mapped = canonical_topic(channel.topic, self._include_tactile)
                if mapped is None:
                    continue
                canonical, ros_type, _ = mapped
                self._topic_map[channel.topic] = mapped
                self._topic_types[canonical] = ros_type

            for record in reader.iter_metadata():
                if record.name != "camera_info":
                    continue
                for value in record.metadata.values():
                    spec = json.loads(value)
                    topic = spec.get("topic")
                    if not topic:
                        continue
                    self._camera_infos.append((topic, _build_camera_info(spec)))
                    self._topic_types[topic] = CAMERA_INFO_TYPE

        self._generator = self._generate()

    def get_all_topics_and_types(self):
        import rosbag2_py

        return [
            rosbag2_py.TopicMetadata(
                name=name,
                type=ros_type,
                serialization_format="cdr",
                offered_qos_profiles="",
            )
            for name, ros_type in self._topic_types.items()
        ]

    def has_next(self) -> bool:
        if self._peeked is not None:
            return True
        if self._exhausted:
            return False
        try:
            self._peeked = next(self._generator)
            return True
        except StopIteration:
            self._exhausted = True
            return False

    def read_next(self):
        if not self.has_next():
            raise RuntimeError("read_next called past end of bag")
        item = self._peeked
        self._peeked = None
        return item

    # -- internals ---------------------------------------------------------

    def _camera_info_messages(self, stamp_ns: int):
        from rclpy.serialization import serialize_message

        sec = int(stamp_ns // 1_000_000_000)
        nanosec = int(stamp_ns % 1_000_000_000)
        for topic, msg in self._camera_infos:
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
            yield topic, serialize_message(msg), stamp_ns

    def _generate(self):
        from mcap.reader import make_reader
        from rclpy.serialization import serialize_message

        next_ci_ns = None
        with open(self._path, "rb") as f:
            reader = make_reader(f)
            for _schema, channel, message in reader.iter_messages():
                log_time = int(message.log_time)
                if next_ci_ns is None:
                    next_ci_ns = log_time
                while self._camera_infos and log_time >= next_ci_ns:
                    yield from self._camera_info_messages(next_ci_ns)
                    next_ci_ns += _CAMERA_INFO_PERIOD_NS

                mapped = self._topic_map.get(channel.topic)
                if mapped is None:
                    continue
                canonical, _ros_type, decoder = mapped
                msg = decoder(message.data)
                yield canonical, serialize_message(msg), log_time


def open_sequential_reader(storage_options, converter_options=None, include_tactile: bool = False):
    """Open a bag reader, transparently handling flatbuffer UMI MCAPs.

    Drop-in replacement for ``rosbag2_py.SequentialReader().open(...)``: returns a
    ``FlatbufferBagReader`` when the URI is a flatbuffer-encoded MCAP, otherwise a
    real ``rosbag2_py.SequentialReader``. Both expose the same
    ``get_all_topics_and_types`` / ``has_next`` / ``read_next`` subset the UMI
    tooling relies on.

    ``include_tactile`` (default ``False``) only affects the flatbuffer path and
    is ignored by the real rosbag2 reader (a real CDR bag already contains
    whatever tactile it has). Keeping the default ``False`` means build_map and
    convert see exactly what they saw before.
    """
    import rosbag2_py

    uri = str(_open_path(storage_options))
    if uri.endswith(".mcap") and is_flatbuffer_mcap(uri):
        reader = FlatbufferBagReader(include_tactile=include_tactile)
    else:
        reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader
