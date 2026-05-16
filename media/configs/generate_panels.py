#!/usr/bin/env python3
"""Emit panel-specific Polka YAMLs by mutating _base.yaml.

The demo has 3 visual panels (B/C/D) and 1 perf panel (E). Panel A (deskew)
was dropped because no available bag has both per-point timestamps and
enough motion to make per-point correction visible.

Panels:
  B fusion     - single front lidar  vs  both lidars merged
  C filters    - range-only / angular-only / box-only / height-only
                 (each applied alone to the same merged input)
  D downsample - voxel (centroid)  vs  random_grid (real points)
  E perf       - full pipeline CUDA  vs  full pipeline CPU
"""
import copy
import pathlib
import sys

try:
    import yaml
except ImportError:
    sys.stderr.write("missing pyyaml; pip install pyyaml\n")
    sys.exit(1)

HERE = pathlib.Path(__file__).resolve().parent
BASE = HERE / "_base.yaml"


def load_base():
    with open(BASE) as f:
        return yaml.safe_load(f)


def write(name, cfg):
    out = HERE / f"{name}.yaml"
    with open(out, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=False)
    print(f"wrote {out.relative_to(HERE.parent.parent)}")


# --- Panel B: fusion -------------------------------------------------------
# B_left: front lidar only, no filters
s = load_base()
s["polka"]["ros__parameters"]["source_names"] = ["front"]
write("B_single", s)

# B_right: both lidars merged, no filters
s = load_base()
write("B_merged", s)

# --- Panel C: filters (each alone on the merged input) ---------------------
def merged_with_filter(label, mutator):
    s = load_base()
    rp = s["polka"]["ros__parameters"]["outputs"]["cloud"]
    mutator(rp)
    write(label, s)


merged_with_filter("C_range", lambda rp: rp["filters"]["range"].update({
    "enabled": True, "min": 0.5, "max": 8.0,
}))

merged_with_filter("C_angular", lambda rp: rp["filters"]["angular"].update({
    "enabled": True,
    "invert": False,
    "ranges": [-45.0, 45.0],  # narrow front hemisphere
}))

merged_with_filter("C_box", lambda rp: rp["filters"]["box"].update({
    "enabled": True,
    "x_min": -5.0, "x_max": 5.0,
    "y_min": -3.0, "y_max": 3.0,
    "z_min": -2.0, "z_max": 5.0,
}))

merged_with_filter("C_height", lambda rp: rp["height_cap"].update({
    "enabled": True, "z_min": 0.0, "z_max": 1.2,
}))

# --- Panel D: downsample (voxel vs random_grid) ----------------------------
def merged_downsample(label, key, leaf, extra=None):
    s = load_base()
    rp = s["polka"]["ros__parameters"]["outputs"]["cloud"]
    rp[key]["enabled"] = True
    rp[key]["leaf_size"] = leaf
    if extra:
        rp[key].update(extra)
    write(label, s)


merged_downsample("D_voxel", "voxel", 0.25)
merged_downsample("D_random_grid", "random_grid", 0.25, extra={"seed": 42})

# --- Panel E: perf (full pipeline CUDA vs CPU) -----------------------------
# Match real-world usage: deskew on (auto-falls-back to whole-scan since no per-
# point timestamps means polka still has work to do), all filters on, voxel on.
# CUDA and CPU configs differ ONLY in enable_gpu so the comparison is fair.
def perf_base(enable_gpu):
    s = load_base()
    p = s["polka"]["ros__parameters"]
    p["enable_gpu"] = enable_gpu
    p["motion_compensation"]["enabled"] = True
    p["motion_compensation"]["imu_topic"] = "/imu/data"
    p["motion_compensation"]["per_point_deskew"] = True
    rp = p["outputs"]["cloud"]
    rp["filters"]["range"].update({"enabled": True, "min": 0.5, "max": 15.0})
    rp["filters"]["box"].update({
        "enabled": True,
        "x_min": -10.0, "x_max": 10.0,
        "y_min": -8.0, "y_max": 8.0,
        "z_min": -2.0, "z_max": 3.0,
    })
    rp["height_cap"].update({"enabled": True, "z_min": -0.5, "z_max": 2.0})
    rp["voxel"]["enabled"] = True
    rp["voxel"]["leaf_size"] = 0.10
    return s


write("E_cuda", perf_base(enable_gpu=True))
write("E_cpu", perf_base(enable_gpu=False))


# --- Panel C2: inverted angular (keep front vs exclude front) --------------
def angular_invert(invert: bool):
    s = load_base()
    rp = s["polka"]["ros__parameters"]["outputs"]["cloud"]
    rp["filters"]["angular"].update({
        "enabled": True,
        "invert": invert,
        "ranges": [-45.0, 45.0],
    })
    return s


write("C2_keep_front",    angular_invert(invert=False))
write("C2_exclude_front", angular_invert(invert=True))


# --- Panel C3: self_filter (off vs chassis box) ----------------------------
def self_filter(enabled: bool):
    s = load_base()
    rp = s["polka"]["ros__parameters"]["outputs"]["cloud"]
    if enabled:
        rp["self_filter"]["enabled"] = True
        rp["self_filter"]["box_names"] = ["chassis"]
        rp["self_filter"]["chassis"] = {
            "x_min": -1.0, "x_max": 1.0,
            "y_min": -0.8, "y_max": 0.8,
            "z_min": -1.0, "z_max": 1.5,
        }
    return s


write("C3_no_self",   self_filter(enabled=False))
write("C3_self",      self_filter(enabled=True))


# --- Panel F: dual output (cloud + scan) -----------------------------------
# F just needs the merged cloud; the LaserScan visualization is computed at
# render time by flattening the cloud at heights z_min/z_max (no separate run).
write("F_dual", load_base())


# --- Panel G: CUDA-live vs CPU-live (same config, just enable_gpu toggle) --
def live_base(enable_gpu):
    s = load_base()
    p = s["polka"]["ros__parameters"]
    p["enable_gpu"] = enable_gpu
    rp = p["outputs"]["cloud"]
    rp["filters"]["range"].update({"enabled": True, "min": 0.5, "max": 12.0})
    rp["height_cap"].update({"enabled": True, "z_min": -0.5, "z_max": 2.0})
    rp["voxel"]["enabled"] = True
    rp["voxel"]["leaf_size"] = 0.10
    return s


write("G_cuda_live", live_base(enable_gpu=True))
write("G_cpu_live",  live_base(enable_gpu=False))


# --- Panel A: deskew (raw vs deskewed) -------------------------------------
# Synthetic-skew demo: a separate publisher streams /synthetic/lidar/points
# (with 1.0 rad/s yaw smear baked in per-point) and /synthetic/imu carrying
# the matching angular velocity. Polka subscribes there instead of the bag.
#
# A_raw         deskew off  → polka outputs the skewed cloud as-is (smear)
# A_deskewed    deskew on   → polka's per-point SE(3) reverses the skew
#
# Both runs disable use_sim_time because the publisher uses wall-clock stamps.
def deskew(enabled: bool):
    s = load_base()
    p = s["polka"]["ros__parameters"]
    p["use_sim_time"] = False
    p["source_names"] = ["front"]
    p["sources"]["front"]["topic"] = "/synthetic/lidar/points"
    # Match the synthetic publisher's RELIABLE QoS so polka's subscriber
    # actually receives messages (best_effort sub of reliable pub *should*
    # work, but discovery is flaky in this container — be explicit).
    p["sources"]["front"]["qos_reliability"] = "reliable"
    # Drop the unused back source to keep the panel single-lidar.
    del p["sources"]["back"]
    p["motion_compensation"]["enabled"] = enabled
    if enabled:
        p["motion_compensation"]["imu_topic"] = "/synthetic/imu"
        p["motion_compensation"]["per_point_deskew"] = True
        p["motion_compensation"]["deskew_timestamp_field"] = "timestamp"
    return s


write("A_raw",      deskew(enabled=False))
write("A_deskewed", deskew(enabled=True))
