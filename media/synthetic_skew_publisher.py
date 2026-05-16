#!/usr/bin/env python3
"""Synthetic skew publisher for the deskew demo panel.

Reads PointCloud2 messages from polka_test.mcap (front lidar), applies a
known forward yaw skew at OMEGA rad/s using each point's per-point timestamp,
and republishes on a synthetic topic. Also publishes a matching synthetic IMU
at OMEGA rad/s so polka's deskew has ground-truth motion to invert.

  /synthetic/lidar/points     - sensor_msgs/PointCloud2  (skewed)
  /synthetic/imu              - sensor_msgs/Imu           (omega_z = OMEGA)
  /tf_static (transient_local) - copies the bag's tf_static

When polka is configured with motion_compensation.enabled=true and
motion_compensation.imu_topic=/synthetic/imu, the per-point SE(3) correction
should cancel the forward skew → sharp cloud. With deskew off, the smear is
visible at typical scan-time scales.

Usage:
  python3 synthetic_skew_publisher.py [--omega 1.0] [--rate 10] [--bag PATH]
"""
import argparse
import math
import pathlib
import struct
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage


FRONT_TOPIC = "/lidar/front/rslidar_points"


def parse_field_layout(msg: PointCloud2):
    """Return (offsets_dict, point_step) and a flag telling us if 'timestamp' is
    a float64 per-point field. Bail if not."""
    offsets = {f.name: f for f in msg.fields}
    if "timestamp" not in offsets:
        raise RuntimeError("Cloud missing per-point 'timestamp' field")
    return offsets, msg.point_step


def apply_yaw_skew(msg: PointCloud2, omega: float) -> PointCloud2:
    """Return a new PointCloud2 with every point's XY rotated by
    omega * (t_point - t_scan_start).  ALSO rewrites the per-point timestamp
    field to be a SMALL RELATIVE OFFSET in seconds from t_scan_start, so polka's
    deskew (which treats pt_time > 1e8 as absolute Unix time, otherwise offset)
    consistently interprets the values as offsets.
    """
    fields, step = parse_field_layout(msg)
    n = msg.width * msg.height
    fx, fy, ft = fields["x"], fields["y"], fields["timestamp"]

    dt_fields = [
        ("x",  np.float32, ()),
        ("y",  np.float32, ()),
        ("z",  np.float32, ()),
        ("intensity", np.float32, ()),
        ("_pad0", "V" + str(ft.offset - 16), ()),
        ("timestamp", np.float64, ()),
        ("_pad1", "V" + str(step - (ft.offset + 8)), ()),
    ]
    dtype = np.dtype({
        "names":   [f[0] for f in dt_fields],
        "formats": [f[1] for f in dt_fields],
        "offsets": [0, 4, 8, 12, 16, ft.offset, ft.offset + 8],
        "itemsize": step,
    })
    src = np.frombuffer(msg.data, dtype=dtype, count=n)
    out_arr = src.copy()

    t = out_arr["timestamp"]
    dt_per_point = (t - t[0]).astype(np.float64)  # seconds since scan start

    dyaw = omega * dt_per_point
    cos_a = np.cos(dyaw).astype(np.float32)
    sin_a = np.sin(dyaw).astype(np.float32)

    new_x = cos_a * out_arr["x"] - sin_a * out_arr["y"]
    new_y = sin_a * out_arr["x"] + cos_a * out_arr["y"]
    out_arr["x"] = new_x
    out_arr["y"] = new_y
    # Crucial: rewrite per-point timestamps to relative offsets so polka treats
    # them as scan-start-relative (pt_time < 1e8 branch in source_adapter.cpp).
    out_arr["timestamp"] = dt_per_point

    out = PointCloud2()
    out.header = msg.header
    out.height = msg.height
    out.width = msg.width
    out.fields = msg.fields
    out.is_bigendian = msg.is_bigendian
    out.point_step = msg.point_step
    out.row_step = msg.row_step
    out.is_dense = msg.is_dense
    out.data = out_arr.tobytes()
    return out


def make_imu(stamp, omega: float) -> Imu:
    imu = Imu()
    imu.header.stamp = stamp
    imu.header.frame_id = "imu_link"
    imu.angular_velocity.z = omega
    # Gravity in IMU frame z so polka's deskew sees a "valid" reading.
    imu.linear_acceleration.z = 9.81
    # orientation_covariance[0] >= 0 with valid quat enables gravity subtraction
    imu.orientation.w = 1.0
    imu.orientation_covariance[0] = 0.0
    return imu


class SkewPublisher(Node):
    def __init__(self, bag_path: str, omega: float, rate_hz: float,
                 duration_s: float):
        super().__init__("synthetic_skew_publisher")
        self.omega = omega
        self.rate_hz = rate_hz
        self.duration_s = duration_s

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub_cloud = self.create_publisher(
            PointCloud2, "/synthetic/lidar/points", qos)
        self.pub_imu = self.create_publisher(Imu, "/synthetic/imu", qos)
        self.pub_tf_static = self.create_publisher(
            TFMessage, "/tf_static", latched)

        # Load all front-lidar clouds + first tf_static into memory.
        so = StorageOptions(uri=bag_path, storage_id="mcap")
        co = ConverterOptions("", "")
        r = SequentialReader()
        r.open(so, co)
        tt = {t.name: t.type for t in r.get_all_topics_and_types()}
        cloud_cls = get_message(tt[FRONT_TOPIC])
        tf_cls = get_message(tt["/tf_static"])

        # Cap cloud loading at duration_s × rate_hz frames + some slack so we
        # don't spend startup time loading 1500 frames when we only publish 100.
        max_frames = max(50, int(duration_s * rate_hz) + 20)
        self.clouds = []
        self.tf_static_msg = None
        while r.has_next():
            if len(self.clouds) >= max_frames and self.tf_static_msg is not None:
                break
            topic, data, _ = r.read_next()
            if topic == FRONT_TOPIC and len(self.clouds) < max_frames:
                self.clouds.append(deserialize_message(data, cloud_cls))
            elif topic == "/tf_static" and self.tf_static_msg is None:
                self.tf_static_msg = deserialize_message(data, tf_cls)
        self.get_logger().info(
            f"loaded {len(self.clouds)} cloud frames, tf_static_msg={'yes' if self.tf_static_msg else 'no'}")

        # Latch tf_static once.
        if self.tf_static_msg is not None:
            self.pub_tf_static.publish(self.tf_static_msg)

        # Use wall clock for stamps (matches how a real driver would behave;
        # avoids the use_sim_time landmine — the subscriber must NOT enable
        # use_sim_time for this panel).
        self.clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        self.frame_idx = 0
        self.start_time = time.monotonic()
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self._tick)

    def _tick(self):
        if (time.monotonic() - self.start_time) > self.duration_s:
            self.get_logger().info("duration reached, shutting down")
            rclpy.shutdown()
            return
        if not self.clouds:
            return
        try:
            msg = self.clouds[self.frame_idx % len(self.clouds)]
            self.frame_idx += 1

            now = self.clock.now().to_msg()

            if self.tf_static_msg is not None and (self.frame_idx % 5 == 0):
                self.pub_tf_static.publish(self.tf_static_msg)

            skewed = apply_yaw_skew(msg, self.omega)
            skewed.header.stamp = now
            skewed.header.frame_id = "rslidarfront"
            self.pub_cloud.publish(skewed)

            imu = make_imu(now, self.omega)
            self.pub_imu.publish(imu)

            if self.frame_idx % 10 == 0:
                self.get_logger().info(
                    f"published {self.frame_idx} clouds")
        except Exception as e:
            self.get_logger().error(f"tick error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default="/workspaces/isaac_ros-dev/bags/polka_test/polka_test.mcap")
    ap.add_argument("--omega", type=float, default=1.0,
                    help="synthetic yaw rate in rad/s")
    ap.add_argument("--rate", type=float, default=10.0,
                    help="publish rate Hz")
    ap.add_argument("--duration", type=float, default=15.0,
                    help="run duration in seconds")
    args = ap.parse_args()

    rclpy.init()
    node = SkewPublisher(args.bag, args.omega, args.rate, args.duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
