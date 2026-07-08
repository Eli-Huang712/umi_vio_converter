#!/usr/bin/env python3
"""direct_feed_build_map.py — P1: single-process, synchronous, in-process VIO feed.

Replaces the stock two-process build_map path (BagPlayer --ROS topics--> perception
--ROS topics--> BuildMapNode, driven by a paced replay + ApproximateTimeSynchronizer)
with ONE process that decodes each frame to a numpy array and feeds perception +
build_map point-to-point, bypassing the ``sensor_msgs/Image`` CDR (de)serialization
and the rclpy executor spin that the flamegraph pinned at ~63% of build_map CPU.

VIO MATH IS UNTOUCHED. ``perception_node.py`` / ``build_map_node.py`` /
``models_trt.py`` are imported as-is; this driver only changes *how data arrives*:
  * ``ShimBridge`` hands perception/build_map the decoded numpy array directly
    (no byte-loop, no CDR) instead of a serialized Image;
  * output publishers are replaced with in-process capture / no-op sinks so no
    message is ever serialized to DDS;
  * frames are fed in strict timestamp order, so pairing is deterministic and the
    stock ``stereo_queue`` frame-drop nondeterminism is gone (an allowed P1 bonus).

The correctness gate is ``poses.npy`` (perception keyframe poses, collected by
build_map). Tactile/IMU/gripper/camera_info channels are added later by
``merge_sqlite_mcap`` straight from the raw bag, independent of this path.
"""
from __future__ import annotations

import argparse
import bisect
import json
import logging
import time

import numpy as np

logger = logging.getLogger("direct_feed")

# Camera<-IMU rotation, verbatim from convert_mcap_sqlite.IMU_TO_CAMERA_R so the
# IMU frame handed to perception matches the stock (serialized) path exactly.
IMU_TO_CAMERA_R = np.array(
    [[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]], dtype=np.float64
)

# perception throttle (~7.5 Hz) + stereo pairing slop, mirrored from perception_node.py.
_THROTTLE_S = 0.1333
_STEREO_SLOP_S = 0.02


class _NpImage:
    """ROS-free stand-in for sensor_msgs/Image carrying a numpy array by reference.

    The whole point of P1: no CDR (de)serialization, no per-byte copy. Downstream
    code only ever touches ``.header`` and hands the object to ``ShimBridge``.
    """

    __slots__ = ("array", "encoding", "header")

    def __init__(self, array, encoding, header):
        self.array = array
        self.encoding = encoding
        self.header = header

def rotate_imu_inplace(imu_msg):
    """Rotate an ``Imu`` msg into the camera frame (mirrors convert_mcap_sqlite.rotate_imu_msg).

    Done directly on the decoded object — no serialize/deserialize round-trip.
    """
    imu_msg.header.frame_id = "camera"
    for vec in (imu_msg.angular_velocity, imu_msg.linear_acceleration):
        rotated = IMU_TO_CAMERA_R @ np.array([vec.x, vec.y, vec.z], dtype=np.float64)
        vec.x, vec.y, vec.z = float(rotated[0]), float(rotated[1]), float(rotated[2])
    for attr in ("angular_velocity_covariance", "linear_acceleration_covariance"):
        cov = getattr(imu_msg, attr)
        if cov[0] < 0.0:
            continue
        rot = IMU_TO_CAMERA_R @ np.asarray(cov, dtype=np.float64).reshape(3, 3) @ IMU_TO_CAMERA_R.T
        setattr(imu_msg, attr, rot.reshape(-1).astype(float).tolist())
    return imu_msg


def pair_stereo(left_stamps, right_stamps, slop_s=_STEREO_SLOP_S):
    """Pair each left frame with the nearest right frame within ``slop`` seconds.

    ``left_stamps``/``right_stamps`` are integer nanoseconds. UMI left/right are
    hardware-synced to ~33 us, so this is deterministic and reproduces what
    ApproximateTimeSynchronizer would have matched. Unpaired boundary frames
    (|Δcount| ≤ 2) are dropped, exactly as the stock synchronizer would drop them.
    """
    slop_ns = slop_s * 1e9
    pairs = []
    for li, lt in enumerate(left_stamps):
        j = bisect.bisect_left(right_stamps, lt)
        best_ri, best_d = None, None
        for ri in (j - 1, j):
            if 0 <= ri < len(right_stamps):
                d = abs(right_stamps[ri] - lt)
                if best_d is None or d < best_d:
                    best_ri, best_d = ri, d
        if best_ri is not None and best_d <= slop_ns:
            pairs.append((li, best_ri))
    return pairs


def _sensor_recorded_topics(sensor):
    """Recorded (rollio) video/imu topics for a sensor, without importing ROS."""
    loc = {"left_wrist": "lefthand", "right_wrist": "righthand", "head": "head"}[sensor]
    return {
        "left_video": f"/camera/coracam_{loc}/left_h264/video",
        "right_video": f"/camera/coracam_{loc}/right_h264/video",
        "imu": f"/observation/imu_{sensor}/imu/imu_accel_gyro",
    }
def _make_header(stamp_ns, frame_id=""):
    from std_msgs.msg import Header

    header = Header()
    header.stamp.sec = int(stamp_ns // 1_000_000_000)
    header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    header.frame_id = frame_id
    return header


class ShimBridge:
    """Drop-in for cv_bridge.CvBridge that passes numpy arrays with zero CDR cost.

    ``imgmsg_to_cv2`` returns the stored array (converting encoding only if the
    caller asks for a different one); ``cv2_to_imgmsg`` wraps an array in _NpImage.
    Any genuine ROS Image (shouldn't occur on this path) falls back to real CvBridge.
    """

    def __init__(self):
        self._real = None  # lazily created only if a real Image ever shows up

    def imgmsg_to_cv2(self, msg, desired_encoding="passthrough"):
        import cv2

        if isinstance(msg, _NpImage):
            arr = msg.array
            src = msg.encoding
            if desired_encoding in ("passthrough", src, None):
                return arr
            if desired_encoding == "mono8":
                if arr.ndim == 3 and arr.shape[2] == 3:
                    code = cv2.COLOR_RGB2GRAY if src == "rgb8" else cv2.COLOR_BGR2GRAY
                    return cv2.cvtColor(arr, code)
                return arr
            if desired_encoding == "bgr8":
                if arr.ndim == 2:
                    return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
                if src == "rgb8":
                    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                return arr
            if desired_encoding == "32FC1":
                return arr
            return arr
        if self._real is None:
            from cv_bridge import CvBridge

            self._real = CvBridge()
        return self._real.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)

    def cv2_to_imgmsg(self, array, encoding="passthrough"):
        return _NpImage(array, encoding, _make_header(0))


class _NullPub:
    """No-op publisher: the message-building compute still runs (VIO behaviour
    preserved), but nothing is ever serialized to DDS."""

    def publish(self, *_a, **_k):
        pass


def read_episode(bag_path, sensor):
    """Decode one raw episode into ordered numpy events for one sensor.

    Returns (left_frames, right_frames, imu_events, cam_info_right, cam_info_left)
    where left_frames[i] = (stamp_ns, mono8_array, bgr8_array),
    right_frames[i] = (stamp_ns, mono8_array), imu_events[i] = (stamp_ns, Imu).
    Reuses the exact decode primitives of the stock path (flatbuffer_codec + PyAV)
    so pixels/IMU are identical; only the transport (DDS) is removed.
    """
    import av
    from mcap.reader import make_reader

    from tool.umi.flatbuffer_codec import decode_compressed_video, decode_imu
    from tool.umi.flatbuffer_reader import _build_camera_info

    topics = _sensor_recorded_topics(sensor)
    ci_right_topic = f"/robot/camera/{sensor}/right/camera_info"
    ci_left_topic = f"/robot/camera/{sensor}/left/camera_info"
    cam_info_right = cam_info_left = None

    # camera_info lives in MCAP metadata records (same source _build_camera_info uses).
    with open(bag_path, "rb") as f:
        reader = make_reader(f)
        for rec in reader.iter_metadata():
            if rec.name != "camera_info":
                continue
            for value in rec.metadata.values():
                spec = json.loads(value)
                if spec.get("topic") == ci_right_topic:
                    cam_info_right = _build_camera_info(spec)
                elif spec.get("topic") == ci_left_topic:
                    cam_info_left = _build_camera_info(spec)

    if cam_info_right is None:
        raise RuntimeError(f"No camera_info for {ci_right_topic} in {bag_path}")

    left_frames, right_frames, imu_events = [], [], []
    left_dec = av.CodecContext.create("h264", "r")
    right_dec = av.CodecContext.create("h264", "r")

    def _decode(dec, video_msg, want_bgr):
        stamp_ns = int(video_msg.timestamp.sec) * 1_000_000_000 + int(video_msg.timestamp.nanosec)
        out = []
        try:
            for packet in dec.parse(bytes(video_msg.data)):
                for frame in dec.decode(packet):
                    mono = np.ascontiguousarray(frame.to_ndarray(format="gray"))
                    bgr = np.ascontiguousarray(frame.to_ndarray(format="bgr24")) if want_bgr else None
                    out.append((stamp_ns, mono, bgr))
        except av.error.FFmpegError as exc:
            logger.warning("video decode error on %s: %s", sensor, exc)
        return out

    with open(bag_path, "rb") as f:
        for _schema, channel, message in make_reader(f).iter_messages():
            if channel.topic == topics["left_video"]:
                video_msg = decode_compressed_video(message.data)
                for stamp_ns, mono, bgr in _decode(left_dec, video_msg, want_bgr=True):
                    left_frames.append((stamp_ns, mono, bgr))
            elif channel.topic == topics["right_video"]:
                video_msg = decode_compressed_video(message.data)
                for stamp_ns, mono, _ in _decode(right_dec, video_msg, want_bgr=False):
                    right_frames.append((stamp_ns, mono))
            elif channel.topic == topics["imu"]:
                imu = rotate_imu_inplace(decode_imu(message.data))
                stamp_ns = int(imu.header.stamp.sec) * 1_000_000_000 + int(imu.header.stamp.nanosec)
                imu_events.append((stamp_ns, imu))

    left_frames.sort(key=lambda e: e[0])
    right_frames.sort(key=lambda e: e[0])
    imu_events.sort(key=lambda e: e[0])
    return left_frames, right_frames, imu_events, cam_info_right, cam_info_left
def _stub_publishers(perception, build_map, keyframe_holder):
    """Redirect perception's keyframe outputs into an in-process holder, and turn
    every other publisher / TF broadcaster on both nodes into a no-op. The message
    *building* still runs (VIO behaviour unchanged); only DDS serialization is cut.
    """
    def _capture_pose(msg):
        keyframe_holder["odom"] = msg

    def _capture_image(msg):
        keyframe_holder["image"] = msg

    def _capture_depth(msg):
        keyframe_holder["depth"] = msg

    perception.keyframe_pose_pub = _CapturePub(_capture_pose)
    perception.keyframe_image_pub = _CapturePub(_capture_image)
    perception.keyframe_depth_pub = _CapturePub(_capture_depth)
    for attr in ("odom_pub", "slam_camera_info_pub", "depth_pub", "disparity_pub_vis",
                 "stats_pub", "debug_image_pub"):
        setattr(perception, attr, _NullPub())
    perception.tf_broadcaster = _NullBroadcaster()

    for attr in ("marker_pub", "local_map_pub", "pose_graph_trajectory_pub",
                 "project_3d_to_2d_pub", "matches_image_pub", "loop_matches_image_pub",
                 "global_map_marker_pub", "retrieval_result_pub",
                 "mapping_save_finished_pub"):
        setattr(build_map, attr, _NullPub())
    build_map.tf_broadcaster = _NullBroadcaster()


class _CapturePub:
    def __init__(self, fn):
        self._fn = fn

    def publish(self, msg):
        self._fn(msg)


class _NullBroadcaster:
    def sendTransform(self, *_a, **_k):
        pass


def run_conversion(bag_path, sensor, map_save_path, verbose_timer=False):
    """Single-process direct feed. Writes ``poses.npy`` etc. identically to stock."""
    import rclpy

    from tinynav.core.build_map_node import BuildMapNode
    from tinynav.core.perception_node import PerceptionNode

    t0 = time.perf_counter()
    logger.info("decoding episode %s sensor=%s", bag_path, sensor)
    left_frames, right_frames, imu_events, ci_right, ci_left = read_episode(bag_path, sensor)
    left_stamps = [e[0] for e in left_frames]
    right_stamps = [e[0] for e in right_frames]
    pairs = pair_stereo(left_stamps, right_stamps)
    logger.info("decoded L=%d R=%d IMU=%d -> %d stereo pairs in %.1fs",
                len(left_frames), len(right_frames), len(imu_events), len(pairs),
                time.perf_counter() - t0)

    rclpy.init()
    perception = PerceptionNode(verbose_timer=verbose_timer)
    build_map = BuildMapNode(map_save_path, verbose_timer=verbose_timer)
    keyframe_holder = {}
    _stub_publishers(perception, build_map, keyframe_holder)
    perception.bridge = ShimBridge()
    build_map.bridge = ShimBridge()

    # Feed intrinsics once (sets K/baseline/min_disparity). right->infra2 (perception's
    # K source + build_map's K); left->color (build_map rgb intrinsics only).
    perception.info_callback(ci_right)
    build_map.info_callback(ci_right)
    if ci_left is not None:
        build_map.rgb_camera_info_callback(ci_left)

    loop = perception._async_loop

    # IMU is fed with a 1-STEP LOOK-AHEAD before each stereo frame. process() first
    # drains all IMU with ts <= current stereo stamp T, then runs a "specially
    # process the last imu" partial-integration step that needs the FIRST IMU with
    # ts > T to be present in the deque. In the stock (async, 96 Hz IMU, ~127 ms/
    # frame) path that straddling sample has always arrived by the time process()
    # runs; feeding ts <= T only would starve that step and skew the trajectory
    # (motion-dependent; seen as ~13 mm on dynamic wrists). So feed all ts <= T PLUS
    # the first ts > T. imu_idx advances monotonically -> each sample fed exactly
    # once, no gap, no double-count.
    imu_idx = 0
    n_imu = len(imu_events)

    def feed_imu_through(target_ns):
        nonlocal imu_idx
        while imu_idx < n_imu and imu_events[imu_idx][0] <= target_ns:
            perception._process_imu_msg(imu_events[imu_idx][1])
            imu_idx += 1
        if imu_idx < n_imu:  # 1-step look-ahead: the straddling sample (ts > T)
            perception._process_imu_msg(imu_events[imu_idx][1])
            imu_idx += 1

    last_processed = 0.0
    n_kf = 0
    for (li, ri) in pairs:
        ts_ns = left_frames[li][0]
        ts_s = ts_ns * 1e-9
        # feed IMU up to (and one past) this stereo stamp BEFORE processing it
        feed_imu_through(ts_ns)
        if ts_s - last_processed < _THROTTLE_S:
            continue
        last_processed = ts_s
        _, left_mono, left_bgr = left_frames[li]
        _, right_mono = right_frames[ri]
        header = _make_header(ts_ns, "camera")
        left_msg = _NpImage(left_mono, "mono8", header)
        right_msg = _NpImage(right_mono, "mono8", header)

        keyframe_holder.clear()
        loop.run_until_complete(perception.process(left_msg, right_msg))

        # perception emitted a keyframe -> feed build_map synchronously with the
        # left/odom/depth it published + the bgr8 from THIS decoded frame (same
        # stamp the ApproximateTimeSynchronizer would have matched).
        if "odom" in keyframe_holder:
            rgb_msg = _NpImage(left_bgr, "bgr8", _make_header(ts_ns, "camera"))
            build_map.keyframe_callback(
                keyframe_holder["image"],
                keyframe_holder["odom"],
                keyframe_holder["depth"],
                rgb_msg,
            )
            n_kf += 1

    logger.info("processed %d keyframes; saving mapping", n_kf)
    build_map.save_mapping()
    logger.info("direct-feed done: %s (wall %.1fs)", map_save_path, time.perf_counter() - t0)

    try:
        perception.destroy_node()
        build_map.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag_file", required=True)
    parser.add_argument("--sensor", required=True, choices=("left_wrist", "right_wrist"))
    parser.add_argument("--map_save_path", required=True)
    parser.add_argument("--verbose_timer", action="store_true")
    # Accepted for CLI parity with build_map_node.py; direct feed does not pace.
    parser.add_argument("--play_rate", default=None)
    parser.add_argument("--no_verbose_timer", dest="verbose_timer", action="store_false")
    parser.set_defaults(verbose_timer=False)
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(filename)s:%(lineno)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    run_conversion(args.bag_file, args.sensor, args.map_save_path, args.verbose_timer)


if __name__ == "__main__":
    main()


