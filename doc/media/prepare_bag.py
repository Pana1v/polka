#!/usr/bin/env python3
"""Prepare the TIERS Calibration.bag (ROS 1) as a clean ROS 2 mcap for polka.

One pass over the ROS 1 bag:
  /ouster/points             (PointCloud2)                   -> passthrough
  /camera/depth/color/points (PointCloud2)                   -> passthrough
  /avia/livox/lidar          (livox_ros_driver/CustomMsg)    -> /avia/points   (frame avia_frame)
  /mid360/livox/lidar        (livox_ros_driver2/CustomMsg)   -> /mid360/points (frame mid360_frame)

Both Livox streams record frame_id 'livox_frame'; they are reassigned to distinct
frames here so the per-sensor extrinsics from the dataset README can be applied.

Output is written with rosbag2_py (native ROS 2) so `ros2 bag play` works directly.
"""
import argparse
import collections
import pathlib
import shutil

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import rosbag2_py

PASSTHROUGH = {"/ouster/points", "/camera/depth/color/points"}
LIVOX_MAP = {
    "/avia/livox/lidar": ("/avia/points", "avia_frame"),
    "/mid360/livox/lidar": ("/mid360/points", "mid360_frame"),
}


def _ns(stamp):
    nsec = getattr(stamp, "nanosec", None)
    if nsec is None:
        nsec = getattr(stamp, "nsec", 0)
    return int(stamp.sec), int(nsec)


def _header(stamp, frame_id):
    sec, nsec = _ns(stamp)
    h = Header()
    h.stamp = Time(sec=sec, nanosec=nsec)
    h.frame_id = frame_id
    return h


def livox_points(m):
    """Extract a (N,4) float32 array (x,y,z,intensity) from a Livox CustomMsg."""
    return np.array([(p.x, p.y, p.z, float(p.reflectivity)) for p in m.points],
                    dtype=np.float32)


def pc2_from_xyzi(arr, frame_id, stamp):
    """Build a PointCloud2 (x,y,z,intensity, point_step 16) from an (N,4) array."""
    n = arr.shape[0]
    pc = PointCloud2()
    pc.header = _header(stamp, frame_id)
    pc.height = 1
    pc.width = n
    pc.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    pc.is_bigendian = False
    pc.point_step = 16
    pc.row_step = 16 * n
    pc.data = np.ascontiguousarray(arr, dtype=np.float32).tobytes()
    pc.is_dense = True
    return pc


def ros1_pc2_to_ros2(m):
    pc = PointCloud2()
    pc.header = _header(m.header.stamp, m.header.frame_id)
    pc.height = int(m.height)
    pc.width = int(m.width)
    pc.fields = [PointField(name=f.name, offset=int(f.offset),
                            datatype=int(f.datatype), count=int(f.count))
                 for f in m.fields]
    pc.is_bigendian = bool(m.is_bigendian)
    pc.point_step = int(m.point_step)
    pc.row_step = int(m.row_step)
    data = m.data
    pc.data = data.tobytes() if hasattr(data, "tobytes") else bytes(data)
    pc.is_dense = bool(m.is_dense)
    return pc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", default=str(pathlib.Path.home() / "Downloads/Calibration.bag"))
    ap.add_argument("--out", default=str(pathlib.Path.home() / "ros2_ws/bags/calibration_ros2"))
    ap.add_argument("--limit", type=int, default=0, help="stop after N messages (debug)")
    ap.add_argument("--accumulate", type=int, default=40,
                    help="Livox sliding-window size (frames) to densify the "
                         "non-repetitive solid-state scans and remove flicker")
    ap.add_argument("--accum-stride", type=int, default=4,
                    help="emit an accumulated Livox cloud every Nth input message")
    args = ap.parse_args()

    out = pathlib.Path(args.out)
    if out.exists():
        shutil.rmtree(out)
    out.parent.mkdir(parents=True, exist_ok=True)

    ts = get_typestore(Stores.ROS1_NOETIC)

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(out), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))

    out_topics = ["/ouster/points", "/camera/depth/color/points",
                  "/avia/points", "/mid360/points"]
    for t in out_topics:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=t, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))

    counts = {t: 0 for t in out_topics}
    # Sliding window of recent Livox frames per source: each non-repetitive
    # 100 Hz swath is a coherent spatial chunk that relocates every frame, so a
    # single swath flickers against the stable Ouster cloud. Accumulating the
    # last N frames keeps the full FOV covered every output frame.
    win = max(1, args.accumulate)
    stride = max(1, args.accum_stride)
    accum = {ot: collections.deque(maxlen=win) for ot, _ in LIVOX_MAP.values()}
    livox_seen = {ot: 0 for ot, _ in LIVOX_MAP.values()}

    with Reader(pathlib.Path(args.inp)) as r:
        for c in r.connections:
            if "CustomMsg" in c.msgtype:
                text = c.msgdef[1] if isinstance(c.msgdef, tuple) else c.msgdef
                ts.register(get_types_from_msg(text, c.msgtype))

        n_seen = 0
        for conn, t_ns, raw in r.messages():
            topic = conn.topic
            if topic in PASSTHROUGH:
                m = ts.deserialize_ros1(raw, conn.msgtype)
                pc = ros1_pc2_to_ros2(m)
                out_topic = topic
            elif topic in LIVOX_MAP:
                m = ts.deserialize_ros1(raw, conn.msgtype)
                out_topic, frame = LIVOX_MAP[topic]
                accum[out_topic].append(livox_points(m))
                livox_seen[out_topic] += 1
                # emit an accumulated cloud every `stride` input frames
                if livox_seen[out_topic] % stride != 0:
                    continue
                pc = pc2_from_xyzi(np.concatenate(accum[out_topic], axis=0),
                                   frame, m.header.stamp)
            else:
                continue
            writer.write(out_topic, serialize_message(pc), int(t_ns))
            counts[out_topic] += 1
            n_seen += 1
            if args.limit and n_seen >= args.limit:
                break

    del writer
    print("wrote:", str(out))
    for t, n in counts.items():
        print(f"  {t:28} {n}")


if __name__ == "__main__":
    main()
