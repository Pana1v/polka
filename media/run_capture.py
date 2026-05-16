#!/usr/bin/env python3
"""Capture one polka run: bag + performance metrics sidecar.

Launches polka with a config, plays the input bag, samples CPU/RAM/GPU and
end-to-end latency, and writes:
  <out_dir>/run.mcap            # /polka/merged_cloud recorded
  <out_dir>/metrics.json        # latency_ms[], cpu_pct[], rss_mb[], gpu_pct[]

The script must run inside a ROS 2 environment with polka built. The bag
player publishes /tf_static with default QoS; pass qos_override.yaml so polka
can subscribe.
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
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import PointCloud2

try:
    import pynvml
    pynvml.nvmlInit()
    _GPU_HANDLE = pynvml.nvmlDeviceGetHandleByIndex(0)
    _HAS_GPU = True
except Exception:
    _HAS_GPU = False


CLOUD_TOPIC = "/polka/merged_cloud"


class LatencySubscriber(Node):
    def __init__(self, use_sim_time: bool = True):
        super().__init__("polka_latency_probe")
        # Match polka's clock so `self.get_clock().now()` and
        # `msg.header.stamp` are in the same time base. If False, both are
        # wall-clock (used by the synthetic-skew publisher in Panel A).
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, use_sim_time)])
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.latencies_ms = []
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._cb, qos)

    def _cb(self, msg):
        now = self.get_clock().now()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        delta_ns = now.nanoseconds - stamp_ns
        self.latencies_ms.append(delta_ns / 1_000_000.0)


def sample_psutil(proc: psutil.Process, stop_evt: threading.Event, out_cpu, out_rss, interval=1.0):
    # Seed cpu_percent so first call returns meaningful value (psutil quirk).
    try:
        proc.cpu_percent(interval=None)
    except psutil.NoSuchProcess:
        return
    next_t = time.monotonic() + interval
    while not stop_evt.is_set():
        try:
            cpu = proc.cpu_percent(interval=None)
            rss = proc.memory_info().rss / (1024 * 1024)
            out_cpu.append(cpu)
            out_rss.append(rss)
        except psutil.NoSuchProcess:
            return
        sleep = max(0.0, next_t - time.monotonic())
        if stop_evt.wait(sleep):
            return
        next_t += interval


def sample_gpu(stop_evt: threading.Event, out_gpu, interval=1.0):
    if not _HAS_GPU:
        return
    next_t = time.monotonic() + interval
    while not stop_evt.is_set():
        try:
            util = pynvml.nvmlDeviceGetUtilizationRates(_GPU_HANDLE)
            out_gpu.append(util.gpu)
        except Exception:
            pass
        sleep = max(0.0, next_t - time.monotonic())
        if stop_evt.wait(sleep):
            return
        next_t += interval


def find_polka_pid(parent_pid: int, timeout=10.0) -> int | None:
    """Walk descendants of `parent_pid` looking for polka_node."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            parent = psutil.Process(parent_pid)
            for child in parent.children(recursive=True):
                try:
                    if "polka_node" in " ".join(child.cmdline()):
                        return child.pid
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        except psutil.NoSuchProcess:
            return None
        time.sleep(0.3)
    return None


def kill_tree(pid: int):
    try:
        parent = psutil.Process(pid)
    except psutil.NoSuchProcess:
        return
    children = parent.children(recursive=True)
    for c in children + [parent]:
        try:
            c.send_signal(signal.SIGINT)
        except psutil.NoSuchProcess:
            pass
    gone, alive = psutil.wait_procs(children + [parent], timeout=3)
    for p in alive:
        try:
            p.kill()
        except psutil.NoSuchProcess:
            pass


def run_once(config_yaml: pathlib.Path, out_dir: pathlib.Path,
             bag_path: pathlib.Path, qos_path: pathlib.Path,
             duration: float, warmup: float = 3.0,
             start_offset: float = 0.0,
             skip_bag_play: bool = False):
    out_dir.mkdir(parents=True, exist_ok=True)
    rec_bag = out_dir / "run"
    polka_log = out_dir / "polka.log"
    rec_log = out_dir / "record.log"
    play_log = out_dir / "play.log"

    # 1. Launch polka.
    polka_proc = subprocess.Popen(
        ["ros2", "launch", "polka", "polka.launch.py",
         f"config_file:={config_yaml}"],
        stdout=open(polka_log, "w"), stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    polka_node_pid = find_polka_pid(polka_proc.pid, timeout=10.0)
    if polka_node_pid is None:
        print(f"  ERROR: polka_node didn't appear under launch pid {polka_proc.pid}",
              flush=True)
        kill_tree(polka_proc.pid)
        return None
    print(f"  polka_node pid={polka_node_pid}", flush=True)

    # 2. Start recorder.
    rec_proc = subprocess.Popen(
        ["ros2", "bag", "record", "-o", str(rec_bag), "--storage", "mcap",
         CLOUD_TOPIC],
        stdout=open(rec_log, "w"), stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    time.sleep(1.0)

    # 3. Start bag player (skipped when an external publisher feeds polka,
    # e.g. the synthetic-skew publisher used by Panel A).
    play_proc = None
    if not skip_bag_play:
        play_cmd = ["ros2", "bag", "play", str(bag_path), "--clock",
                    "--read-ahead-queue-size", "2000",
                    "--qos-profile-overrides-path", str(qos_path)]
        if start_offset > 0:
            play_cmd += ["--start-offset", str(start_offset)]
        play_proc = subprocess.Popen(
            play_cmd,
            stdout=open(play_log, "w"), stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

    # 4. Start metric samplers + latency subscriber.
    rclpy.init()
    latency_node = LatencySubscriber()
    stop_evt = threading.Event()
    cpu_pct = []
    rss_mb = []
    gpu_pct = []

    proc = psutil.Process(polka_node_pid)
    th_cpu = threading.Thread(target=sample_psutil,
                              args=(proc, stop_evt, cpu_pct, rss_mb),
                              daemon=True)
    th_gpu = threading.Thread(target=sample_gpu, args=(stop_evt, gpu_pct),
                              daemon=True)
    th_cpu.start()
    th_gpu.start()

    # 5. Spin rclpy until duration elapses.
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        rclpy.spin_once(latency_node, timeout_sec=0.1)

    # 6. Teardown.
    stop_evt.set()
    kill_tree(rec_proc.pid)
    if play_proc is not None:
        kill_tree(play_proc.pid)
    kill_tree(polka_proc.pid)
    th_cpu.join(timeout=2)
    th_gpu.join(timeout=2)

    latencies = latency_node.latencies_ms
    latency_node.destroy_node()
    rclpy.shutdown()

    # 7. Drop warmup samples (CUDA init, cuBLAS JIT, subscriber settle).
    warmup_drops = int(warmup * 10)  # latency at ~10Hz output → drop ~30 samples
    latencies = latencies[warmup_drops:]
    cpu_pct = cpu_pct[int(warmup):]
    rss_mb = rss_mb[int(warmup):]
    gpu_pct = gpu_pct[int(warmup):]

    metrics = {
        "config": str(config_yaml),
        "latency_ms": latencies,
        "cpu_pct": cpu_pct,
        "rss_mb": rss_mb,
        "gpu_pct": gpu_pct,
        "n_msgs": len(latencies),
        "duration_s": duration,
        "warmup_s": warmup,
    }
    metrics_path = out_dir / "metrics.json"
    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2)

    n_lat = len(latencies)
    n_cpu = len(cpu_pct)
    n_gpu = len(gpu_pct)
    print(f"  metrics: n_lat={n_lat} n_cpu={n_cpu} n_gpu={n_gpu}", flush=True)
    if n_lat == 0:
        print("  WARN: no merged_cloud messages received", flush=True)
        return None
    return metrics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config_yaml", type=pathlib.Path)
    ap.add_argument("out_dir", type=pathlib.Path)
    ap.add_argument("--bag", type=pathlib.Path,
                    default=pathlib.Path("/workspaces/isaac_ros-dev/bags/polka_test/polka_test.mcap"))
    ap.add_argument("--qos", type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / "qos_override.yaml")
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--warmup", type=float, default=3.0)
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--start-offset", type=float, default=0.0,
                    help="seconds into the bag to start playback")
    ap.add_argument("--skip-bag-play", action="store_true",
                    help="don't start ros2 bag play (e.g. when an external "
                         "publisher feeds polka, like the synthetic-skew demo)")
    args = ap.parse_args()

    if args.repeats == 1:
        run_once(args.config_yaml, args.out_dir, args.bag, args.qos,
                 args.duration, args.warmup, args.start_offset,
                 skip_bag_play=args.skip_bag_play)
    else:
        for i in range(args.repeats):
            sub = args.out_dir / f"rep_{i:02d}"
            print(f"== repeat {i+1}/{args.repeats} → {sub} ==", flush=True)
            run_once(args.config_yaml, sub, args.bag, args.qos,
                     args.duration, args.warmup, args.start_offset,
                     skip_bag_play=args.skip_bag_play)


if __name__ == "__main__":
    main()
