#!/usr/bin/env python3
"""
Render the polka 0.5.0 before/after performance summary to doc/images/perf_summary.svg.

Three measured 0.5.0 improvements, each its own panel because the units differ
(two are latency, one is data volume). Bars start at zero; the light panel with
dark text keeps the SVG legible on both light and dark README backgrounds.

  python3 doc/media/render_perf_summary.py

Regenerate after the numbers in CHANGELOG.rst change.
"""
import pathlib

import matplotlib
matplotlib.use('Agg')
# embed glyphs as vector paths so the SVG renders without the font installed
matplotlib.rcParams['svg.fonttype'] = 'path'
import matplotlib.pyplot as plt  # noqa: E402

# Light panel + dark text so the SVG survives GitHub light and dark themes.
PANEL = '#eef1f5'
INK = '#1b1e24'
MUTED = '#5b6472'
GRID = '#d3d9e0'
BEFORE = '#94a3b8'   # slate: the old path
AFTER = '#2a9d8f'    # teal (matches the PCL badge): the optimized path

# The two genuine 0.5.0 speedups: the same computation, made faster by a code
# change. Voxel downsampling is deliberately excluded; its point-count ratio is
# a user-set leaf_size tradeoff, not a code speedup, so it does not belong here.
# (title, unit, before, after, before_label, after_label, callout)
METRICS = [
    ('Deskew stage', 'ms / source', 9.8, 1.6, '9.8', '1.6', '6.2x faster'),
    ('CPU angular filter', 'ms / tick @259k pts', 10.47, 3.55, '10.47', '3.55', '3x faster'),
]


def render(out_paths):
    fig, axes = plt.subplots(1, len(METRICS), figsize=(7.6, 3.8), facecolor=PANEL)
    fig.suptitle('Polka 0.5.0 measured improvements (before to after)',
                 color=INK, fontsize=14, family='monospace', weight='bold', y=0.98)

    for ax, (title, unit, before, after, lb, la, callout) in zip(axes, METRICS):
        ax.set_facecolor(PANEL)
        bars = ax.bar([0, 1], [before, after], width=0.62,
                      color=[BEFORE, AFTER], zorder=3)
        ax.set_ylim(0, before * 1.32)  # headroom for the value label + callout
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['before', 'after'], color=INK,
                           fontsize=10, family='monospace')
        ax.set_title(title, color=INK, fontsize=12, family='monospace',
                     weight='bold', pad=8)
        ax.set_ylabel(unit, color=MUTED, fontsize=9, family='monospace')

        for bar, label in zip(bars, [lb, la]):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    label, ha='center', va='bottom', color=INK,
                    fontsize=11, family='monospace', weight='bold')

        ax.text(0.5, 0.93, callout, transform=ax.transAxes, ha='center',
                va='top', color=AFTER, fontsize=12, family='monospace',
                weight='bold')

        ax.tick_params(axis='y', colors=MUTED, labelsize=8)
        ax.tick_params(axis='x', length=0)
        ax.yaxis.grid(True, color=GRID, linewidth=0.8, zorder=0)
        ax.set_axisbelow(True)
        for spine in ('top', 'right'):
            ax.spines[spine].set_visible(False)
        for spine in ('left', 'bottom'):
            ax.spines[spine].set_color(GRID)

    fig.text(0.01, 0.02, 'source: CHANGELOG 0.5.0', color=MUTED,
             fontsize=9, family='monospace')
    fig.text(0.99, 0.02, 'polka', color=MUTED, fontsize=9, family='monospace',
             ha='right', weight='bold')
    fig.subplots_adjust(left=0.1, right=0.97, top=0.82, bottom=0.14, wspace=0.36)

    for p in out_paths:
        fig.savefig(p, facecolor=PANEL)
    plt.close(fig)


def main():
    here = pathlib.Path(__file__).resolve().parent
    svg = here.parent / 'images' / 'perf_summary.svg'
    svg.parent.mkdir(parents=True, exist_ok=True)
    render([svg])
    print(f'wrote {svg}')


if __name__ == '__main__':
    main()
