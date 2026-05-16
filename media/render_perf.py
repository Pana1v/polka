#!/usr/bin/env python3
"""Render the Panel E perf chart: CUDA vs CPU bar groups.

Reads metrics from work/E_cuda/rep_*/metrics.json and work/E_cpu/rep_*/metrics.json
(one or more repeats each) and emits a single PNG in frames_panels/panel_E/f_0000.png.
"""
import argparse
import json
import pathlib
import statistics

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

BG = "#161616"
W, H = 1920, 1080
DPI = 100


def load_metrics(stage_dir: pathlib.Path):
    """Aggregate metrics across rep_*/metrics.json subdirs (or a single
    metrics.json if no reps)."""
    paths = sorted(stage_dir.glob("rep_*/metrics.json"))
    if not paths:
        flat = stage_dir / "metrics.json"
        if flat.is_file():
            paths = [flat]
    all_lat, all_cpu, all_rss, all_gpu, msg_rates = [], [], [], [], []
    for p in paths:
        m = json.loads(p.read_text())
        if m["latency_ms"]:
            all_lat.extend(m["latency_ms"])
        if m["cpu_pct"]:
            all_cpu.extend(m["cpu_pct"])
        if m["rss_mb"]:
            all_rss.extend(m["rss_mb"])
        if m["gpu_pct"]:
            all_gpu.extend(m["gpu_pct"])
        # Effective output throughput
        if m["n_msgs"] and m["duration_s"]:
            msg_rates.append(m["n_msgs"] / m["duration_s"])
    def agg(xs):
        if not xs:
            return (0.0, 0.0)
        return (float(np.mean(xs)), float(np.std(xs)))
    return {
        "latency_ms": agg(all_lat),
        "cpu_pct":    agg(all_cpu),
        "rss_mb":     agg(all_rss),
        "gpu_pct":    agg(all_gpu),
        "msg_rate":   agg(msg_rates),
        "n_reps":     len(paths),
    }


def render(cuda, cpu, out_path):
    fig, axes = plt.subplots(1, 4, figsize=(W / DPI, H / DPI), dpi=DPI,
                              facecolor=BG, gridspec_kw={"wspace": 0.32, "top": 0.78, "bottom": 0.12})
    fig.suptitle("polka  ·  CUDA vs CPU performance",
                  color="white", fontsize=32, family="monospace",
                  weight="bold", y=0.93)
    fig.text(0.5, 0.86,
             f"merged_cloud throughput  ·  same config except enable_gpu  "
             f"(reps: CUDA={cuda['n_reps']}, CPU={cpu['n_reps']})",
             color="#888", fontsize=14, family="monospace", ha="center")

    metrics = [
        ("msg_rate",   "msg rate (Hz)", "higher = better", 1.0),
        ("cpu_pct",    "CPU %",         "lower = better",  1.0),
        ("rss_mb",     "RAM (MB)",      "lower = better",  1.0),
        ("gpu_pct",    "GPU %",         "info",            1.0),
    ]

    bar_colors = {"CUDA": "#76b900", "CPU": "#ff9248"}

    for ax, (key, label, hint, _) in zip(axes, metrics):
        ax.set_facecolor(BG)
        c_mean, c_std = cuda[key]
        p_mean, p_std = cpu[key]
        positions = [0, 1]
        means = [c_mean, p_mean]
        errs  = [c_std,  p_std]
        bars = ax.bar(positions, means, yerr=errs,
                      color=[bar_colors["CUDA"], bar_colors["CPU"]],
                      edgecolor="white", linewidth=1.5, capsize=8,
                      error_kw={"ecolor": "white", "elinewidth": 1.2})
        for x, mean, std in zip(positions, means, errs):
            if mean == 0 and std == 0:
                ax.text(x, 0.5, "n/a", ha="center", color="#666", fontsize=14)
            else:
                ax.text(x, mean + max(std, 0.02 * max(means + [1])) + max(means + [1]) * 0.04,
                        f"{mean:.1f}", ha="center", color="white",
                        fontsize=16, family="monospace", weight="bold")
        ax.set_xticks(positions)
        ax.set_xticklabels(["CUDA", "CPU"], color="white",
                            fontsize=16, family="monospace", weight="bold")
        ax.set_title(label, color="white", fontsize=20,
                      family="monospace", weight="bold", pad=10)
        ax.text(0.5, -0.08, hint, transform=ax.transAxes, ha="center",
                 color="#888", fontsize=12, family="monospace")
        ax.tick_params(axis="y", colors="white")
        for s in ax.spines.values():
            s.set_color("#555")
        max_v = max(means) + max(errs) if max(means) > 0 else 1
        ax.set_ylim(0, max_v * 1.30 + 0.5)
        ax.grid(axis="y", color="#333", linestyle="--", alpha=0.4)

    fig.text(0.5, 0.02,
             "measured end-to-end on bag replay (use_sim_time);  CUDA=RTX PRO 2000",
             color="#666", fontsize=11, family="monospace", ha="center", style="italic")
    fig.savefig(out_path, dpi=DPI, facecolor=BG, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--work", type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / "work")
    ap.add_argument("--out",  type=pathlib.Path,
                    default=pathlib.Path(__file__).parent / "frames_panels")
    args = ap.parse_args()

    cuda = load_metrics(args.work / "E_cuda")
    cpu  = load_metrics(args.work / "E_cpu")
    print(f"CUDA: {cuda}")
    print(f"CPU:  {cpu}")

    out_dir = args.out / "panel_E"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "f_0000.png"
    render(cuda, cpu, out_path)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
