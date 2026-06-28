#!/usr/bin/env python3
"""Capture one polka run: output bag + performance metrics sidecar.

Launches demo_bringup.launch.py (polka_node + static TFs) with a config, plays
the converted calibration bag, samples CPU/RAM/GPU + end-to-end latency, and
records /polka/merged_cloud and /polka/merged_scan to <out_dir>/run.

The merged cloud/scan are recorded *inside* this node (not a separate
`ros2 bag record` process) so the recorded set matches exactly what the
subscriber receives, with QoS that matches polka's publishers.
"""
import argparse
import json
import os
import pathlib
import signal
import subprocess
import threading
import time

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, LaserScan
import rosbag2_py

try:
    import pynvml
    pynvml.nvmlInit()
    _GPU_HANDLE = pynvml.nvmlDeviceGetHandleByIndex(0)
    _HAS_GPU = True
except Exception:
    _HAS_GPU = False

HERE = pathlib.Path(__file__).parent
LAUNCH_FILE = HERE / "demo_bringup.launch.py"
CLOUD_TOPIC = "/polka/merged_cloud"
SCAN_TOPIC = "/polka/merged_scan"
DEFAULT_BAG = pathlib.Path.home() / "ros2_ws/bags/calibration_ros2"


def _reliable(depth=10):
    q = QoSProfile(depth=depth)
    q.reliability = ReliabilityPolicy.RELIABLE
    q.durability = DurabilityPolicy.VOLATILE
    return q


class CaptureNode(Node):
    def __init__(self, run_dir):
        super().__init__("polka_capture_probe")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])
        self.latencies_ms = []

        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(run_dir), storage_id="mcap"),
                         rosbag2_py.ConverterOptions("", ""))
        self.writer.create_topic(rosbag2_py.TopicMetadata(
            name=CLOUD_TOPIC, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
        self.writer.create_topic(rosbag2_py.TopicMetadata(
            name=SCAN_TOPIC, type="sensor_msgs/msg/LaserScan", serialization_format="cdr"))

        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._cloud_cb, _reliable())
        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_cb, _reliable())

    def _stamp_ns(self, msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _cloud_cb(self, msg):
        now = self.get_clock().now()
        self.latencies_ms.append((now.nanoseconds - self._stamp_ns(msg)) / 1_000_000.0)
        self.writer.write(CLOUD_TOPIC, serialize_message(msg), self._stamp_ns(msg))

    def _scan_cb(self, msg):
        self.writer.write(SCAN_TOPIC, serialize_message(msg), self._stamp_ns(msg))

    def close(self):
        self.writer = None  # drop reference -> flush/close the mcap


def sample_psutil(proc, stop_evt, out_cpu, out_rss, interval=1.0):
    try:
        proc.cpu_percent(interval=None)
    except psutil.NoSuchProcess:
        return
    next_t = time.monotonic() + interval
    while not stop_evt.is_set():
        try:
            out_cpu.append(proc.cpu_percent(interval=None))
            out_rss.append(proc.memory_info().rss / (1024 * 1024))
        except psutil.NoSuchProcess:
            return
        if stop_evt.wait(max(0.0, next_t - time.monotonic())):
            return
        next_t += interval


def sample_gpu(stop_evt, out_gpu, interval=1.0):
    if not _HAS_GPU:
        return
    next_t = time.monotonic() + interval
    while not stop_evt.is_set():
        try:
            out_gpu.append(pynvml.nvmlDeviceGetUtilizationRates(_GPU_HANDLE).gpu)
        except Exception:
            pass
        if stop_evt.wait(max(0.0, next_t - time.monotonic())):
            return
        next_t += interval


def find_polka_pid(parent_pid, timeout=12.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            for child in psutil.Process(parent_pid).children(recursive=True):
                try:
                    if "polka_node" in " ".join(child.cmdline()):
                        return child.pid
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        except psutil.NoSuchProcess:
            return None
        time.sleep(0.3)
    return None


def kill_tree(pid):
    try:
        parent = psutil.Process(pid)
    except psutil.NoSuchProcess:
        return
    procs = parent.children(recursive=True) + [parent]
    for c in procs:
        try:
            c.send_signal(signal.SIGINT)
        except psutil.NoSuchProcess:
            pass
    _, alive = psutil.wait_procs(procs, timeout=3)
    for p in alive:
        try:
            p.kill()
        except psutil.NoSuchProcess:
            pass


def run_once(config_yaml, out_dir, bag_path, duration, warmup=3.0):
    out_dir.mkdir(parents=True, exist_ok=True)
    run_dir = out_dir / "run"
    if run_dir.exists():
        import shutil
        shutil.rmtree(run_dir)

    polka_proc = subprocess.Popen(
        ["ros2", "launch", str(LAUNCH_FILE), f"config_file:={config_yaml}"],
        stdout=open(out_dir / "polka.log", "w"), stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)
    polka_node_pid = find_polka_pid(polka_proc.pid)
    if polka_node_pid is None:
        print(f"  ERROR: polka_node didn't appear under {polka_proc.pid}", flush=True)
        kill_tree(polka_proc.pid)
        return None
    print(f"  polka_node pid={polka_node_pid}", flush=True)

    rclpy.init()
    node = CaptureNode(run_dir)

    time.sleep(1.0)  # let polka settle / TFs latch before playback
    play_proc = subprocess.Popen(
        ["ros2", "bag", "play", str(bag_path), "--clock",
         "--read-ahead-queue-size", "2000"],
        stdout=open(out_dir / "play.log", "w"), stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)

    stop_evt = threading.Event()
    cpu_pct, rss_mb, gpu_pct = [], [], []
    proc = psutil.Process(polka_node_pid)
    th_cpu = threading.Thread(target=sample_psutil, args=(proc, stop_evt, cpu_pct, rss_mb), daemon=True)
    th_gpu = threading.Thread(target=sample_gpu, args=(stop_evt, gpu_pct), daemon=True)
    th_cpu.start(); th_gpu.start()

    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    stop_evt.set()
    kill_tree(play_proc.pid)
    kill_tree(polka_proc.pid)
    th_cpu.join(timeout=2); th_gpu.join(timeout=2)

    latencies = node.latencies_ms[:]
    node.close()
    node.destroy_node()
    rclpy.shutdown()

    latencies = latencies[int(warmup * 10):]
    cpu_pct = cpu_pct[int(warmup):]
    rss_mb = rss_mb[int(warmup):]
    gpu_pct = gpu_pct[int(warmup):]

    metrics = {"config": str(config_yaml), "latency_ms": latencies,
               "cpu_pct": cpu_pct, "rss_mb": rss_mb, "gpu_pct": gpu_pct,
               "n_msgs": len(latencies), "duration_s": duration, "warmup_s": warmup}
    with open(out_dir / "metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)

    print(f"  metrics: n_lat={len(latencies)} n_cpu={len(cpu_pct)} n_gpu={len(gpu_pct)}", flush=True)
    if not latencies:
        print("  WARN: no merged_cloud messages received", flush=True)
        return None
    return metrics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config_yaml", type=pathlib.Path)
    ap.add_argument("out_dir", type=pathlib.Path)
    ap.add_argument("--bag", type=pathlib.Path, default=DEFAULT_BAG)
    ap.add_argument("--duration", type=float, default=11.0)
    ap.add_argument("--warmup", type=float, default=2.0)
    ap.add_argument("--repeats", type=int, default=1)
    args = ap.parse_args()

    if args.repeats == 1:
        run_once(args.config_yaml, args.out_dir, args.bag, args.duration, args.warmup)
    else:
        for i in range(args.repeats):
            sub = args.out_dir / f"rep_{i:02d}"
            print(f"== repeat {i+1}/{args.repeats} -> {sub} ==", flush=True)
            run_once(args.config_yaml, sub, args.bag, args.duration, args.warmup)


if __name__ == "__main__":
    main()
