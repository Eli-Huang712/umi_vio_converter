#!/usr/bin/env python3
"""Flatbuffer decoding primitives shared by the flatbuffer MCAP tooling.

Hand-rolled flatbuffer table reader plus decoders that turn rollio flatbuffer
messages into ROS 2 message objects. Kept dependency-free (only ``struct``) so it
can be imported from both the offline converter and the live bag reader.

umi_vio_converter surgery: adds ``discover.TactileData`` decoding
(``decode_tactile`` -> ``sensor_msgs/msg/PointCloud2``) and the struct-vector
helper it needs. All original decoders/behaviour are unchanged.
"""
from __future__ import annotations

import struct


VIDEO_TYPE = "foxglove_msgs/msg/CompressedVideo"
JOINT_STATE_TYPE = "sensor_msgs/msg/JointState"
IMU_TYPE = "sensor_msgs/msg/Imu"
# --- umi_vio_converter surgical addition ---------------------------------
TACTILE_TYPE = "sensor_msgs/msg/PointCloud2"
_TACTILE_POINT_STEP = 24  # discover.TactilePoint = 6 x float32 (x,y,z,fx,fy,fz)
# -------------------------------------------------------------------------

TOPIC_TYPE_MAP = {
    "foxglove.CompressedVideo": VIDEO_TYPE,
    "foxglove.JointStates": JOINT_STATE_TYPE,
    "discover.Imu": IMU_TYPE,
    # --- umi_vio_converter surgical addition (offline converter parity) ---
    "discover.TactileData": TACTILE_TYPE,
}


class FlatTable:
    def __init__(self, data: bytes, pos: int | None = None):
        self.data = data
        self.pos = struct.unpack_from("<I", data, 0)[0] if pos is None else pos
        self.vtable = self.pos - struct.unpack_from("<i", data, self.pos)[0]

    def field_offset(self, index: int) -> int:
        offset = 4 + index * 2
        if offset >= struct.unpack_from("<H", self.data, self.vtable)[0]:
            return 0
        return struct.unpack_from("<H", self.data, self.vtable + offset)[0]

    def table(self, index: int):
        offset = self.field_offset(index)
        if offset == 0:
            return None
        field = self.pos + offset
        return FlatTable(self.data, field + struct.unpack_from("<I", self.data, field)[0])

    def string(self, index: int) -> str:
        offset = self.field_offset(index)
        if offset == 0:
            return ""
        field = self.pos + offset
        start = field + struct.unpack_from("<I", self.data, field)[0]
        size = struct.unpack_from("<I", self.data, start)[0]
        return self.data[start + 4:start + 4 + size].decode("utf-8")

    def vector_start(self, index: int) -> int | None:
        offset = self.field_offset(index)
        if offset == 0:
            return None
        field = self.pos + offset
        return field + struct.unpack_from("<I", self.data, field)[0]

    def vector_bytes(self, index: int) -> bytes:
        start = self.vector_start(index)
        if start is None:
            return b""
        size = struct.unpack_from("<I", self.data, start)[0]
        return self.data[start + 4:start + 4 + size]

    # --- umi_vio_converter surgical addition -----------------------------
    def vector_struct_bytes(self, index: int, stride: int) -> bytes:
        """Return the raw bytes of a vector of inline structs.

        ``vector_bytes`` is only correct for ``[ubyte]`` vectors, where the
        vector header (a uint32) equals the byte length. For a vector of inline
        structs (e.g. ``discover.TactilePoint``) that header is the element
        *count*, so the payload spans ``count * stride`` bytes.
        """
        start = self.vector_start(index)
        if start is None:
            return b""
        count = struct.unpack_from("<I", self.data, start)[0]
        return self.data[start + 4:start + 4 + count * stride]
    # ---------------------------------------------------------------------

    def vector_table(self, index: int, item_index: int):
        start = self.vector_start(index)
        if start is None:
            return None
        size = struct.unpack_from("<I", self.data, start)[0]
        if item_index >= size:
            return None
        field = start + 4 + item_index * 4
        return FlatTable(self.data, field + struct.unpack_from("<I", self.data, field)[0])

    def vector_len(self, index: int) -> int:
        start = self.vector_start(index)
        if start is None:
            return 0
        return struct.unpack_from("<I", self.data, start)[0]

    def int32(self, index: int) -> int:
        offset = self.field_offset(index)
        return 0 if offset == 0 else struct.unpack_from("<i", self.data, self.pos + offset)[0]

    def uint32(self, index: int) -> int:
        offset = self.field_offset(index)
        return 0 if offset == 0 else struct.unpack_from("<I", self.data, self.pos + offset)[0]

    def float64(self, index: int) -> float:
        offset = self.field_offset(index)
        return 0.0 if offset == 0 else struct.unpack_from("<d", self.data, self.pos + offset)[0]

    def read_time(self, dst, index: int) -> None:
        """Read a ``foxglove.Time`` *struct* (inline ``sec``/``nsec`` uint32) into dst.

        ``Time`` is a flatbuffer struct, so it is stored inline at the field
        offset (no table indirection). ``sec``/``nsec`` map to ROS ``sec``/``nanosec``.
        """
        offset = self.field_offset(index)
        if offset == 0:
            return
        base = self.pos + offset
        dst.sec = struct.unpack_from("<I", self.data, base)[0]
        dst.nanosec = struct.unpack_from("<I", self.data, base + 4)[0]


def set_vector3(dst, src: FlatTable | None):
    if src is None:
        return
    dst.x = src.float64(0)
    dst.y = src.float64(1)
    dst.z = src.float64(2)


def set_quaternion(dst, src: FlatTable | None):
    if src is None:
        return
    dst.x = src.float64(0)
    dst.y = src.float64(1)
    dst.z = src.float64(2)
    dst.w = src.float64(3)


def decode_compressed_video(data: bytes):
    from foxglove_msgs.msg import CompressedVideo

    table = FlatTable(data)
    msg = CompressedVideo()
    table.read_time(msg.timestamp, 0)
    msg.frame_id = table.string(1)
    msg.data = table.vector_bytes(2)
    msg.format = table.string(3)
    return msg


def decode_joint_states(data: bytes):
    from sensor_msgs.msg import JointState

    table = FlatTable(data)
    msg = JointState()
    table.read_time(msg.header.stamp, 0)
    for i in range(table.vector_len(1)):
        joint = table.vector_table(1, i)
        if joint is None:
            continue
        msg.name.append(joint.string(0))
        msg.position.append(joint.float64(1))
        msg.velocity.append(joint.float64(2))
        msg.effort.append(joint.float64(4))
    return msg


def decode_imu(data: bytes):
    from sensor_msgs.msg import Imu

    table = FlatTable(data)
    msg = Imu()
    table.read_time(msg.header.stamp, 0)
    msg.header.frame_id = table.string(1)
    set_quaternion(msg.orientation, table.table(2))
    set_vector3(msg.angular_velocity, table.table(3))
    set_vector3(msg.linear_acceleration, table.table(4))
    return msg


# --- umi_vio_converter surgical addition ---------------------------------
def decode_tactile(data: bytes):
    """Decode a ``discover.TactileData`` flatbuffer into ``sensor_msgs/PointCloud2``.

    Layout (empirically verified against real bags):
      * field 0 ``timestamp``  -> ``foxglove.Time`` inline struct (sec/nsec u32)
      * field 1 ``frame_id``   -> string (recorded value is the full topic path)
      * field 2 ``points``     -> vector of inline ``discover.TactilePoint``
                                  structs, 24 bytes each = 6 float32
                                  ``[x, y, z, fx, fy, fz]`` (position + force).

    The inline point bytes are already contiguous little-endian float32 in the
    exact ``[x,y,z,fx,fy,fz]`` order, so they are copied verbatim into the cloud
    ``data`` (point_step 24, one FLOAT32 field per component). Lossless.
    """
    from sensor_msgs.msg import PointCloud2, PointField

    table = FlatTable(data)
    msg = PointCloud2()
    table.read_time(msg.header.stamp, 0)
    msg.header.frame_id = table.string(1)

    raw = table.vector_struct_bytes(2, _TACTILE_POINT_STEP)
    count = len(raw) // _TACTILE_POINT_STEP

    field_names = ("x", "y", "z", "fx", "fy", "fz")
    msg.fields = [
        PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(field_names)
    ]
    msg.height = 1
    msg.width = count
    msg.is_bigendian = False
    msg.point_step = _TACTILE_POINT_STEP
    msg.row_step = _TACTILE_POINT_STEP * count
    msg.is_dense = True
    msg.data = raw
    return msg
# -------------------------------------------------------------------------


DECODERS = {
    "foxglove.CompressedVideo": decode_compressed_video,
    "foxglove.JointStates": decode_joint_states,
    "discover.Imu": decode_imu,
    # --- umi_vio_converter surgical addition (offline converter parity) ---
    "discover.TactileData": decode_tactile,
}
