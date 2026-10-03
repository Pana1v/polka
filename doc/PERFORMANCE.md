# Performance

Measured speedups, all on the CPU path, all from the CHANGELOG.

| Change | Before | After | Factor |
|---|---|---|---|
| Deskew, rotation-only path (real IMU), per point | 41.5 ns | 3.3 ns | ~12x cheaper |
| Deskew, SE(3) path, per point | 41.5 ns | 6.9 ns | ~6x cheaper |
| CPU angular filter, per tick at 259k points | 10.47 ms | 3.55 ms | ~3x cheaper |

Deskew numbers: six 66k-point RoboSense scans, one pinned core. The rotation-only path computes an exact rotation every 16 points and interpolates in between, re-anchoring across larger time jumps. 0.5.0 listed a 6.2x deskew gain from that stride, but the path only ran for IMU acceleration of exactly zero, which real IMUs never report, so it did not apply in practice until this release. The angular filter dropped its per-point `atan2` for a precomputed cross-product half-plane test.

Voxel downsampling is not in that table. It trades resolution for data volume rather than doing the same work faster — see bandwidth below.

## Cheaper deskew is not straighter deskew

Two different claims get mixed up here.

The deskew speedups are cost numbers. They say the correction takes less time to compute, nothing more.

The quality side is separate. A LiDAR collects its points over a few tens of milliseconds, so if it rotates or translates during that sweep a rigid scan smears structure across the frame. Per-point SE(3) correction moves each point by the pose at its own timestamp and takes that smear out. `deskew.gif` in the README shows raw against deskewed under a synthetic 1 rad/s yaw. That clip is the quality claim; the table is the cost claim.

## CUDA is a crossover, not a free win

Built with `-DWITH_CUDA=ON`, polka runs the whole per-point path — transform, filter, voxel, scan flatten — as one fused GPU pass.

When a lot of points go through several filters, that pass wins. On a filterless merge it usually doesn't: there is too little per-point work to hide kernel dispatch and the host-to-device copy, so the CPU keeps up. It falls back to CPU automatically when built without CUDA or when no device is present.

No CUDA timings are published here. Measure it on your own pipeline.

## Bandwidth

Merging saves traffic regardless of how fast the merge itself runs.

- **N streams, one topic.** Downstream nodes subscribe once to the merged output instead of once per sensor. One frame, one QoS, one message to reason about.
- **Voxel downsampling.** Not new in 0.5.0 and not a speedup — a resolution-for-bandwidth trade you set with `leaf_size`. At the leaf size used in the demo clip, 69k points come out as 5k. A bigger leaf thins more, a smaller one thins less.
- **Slimmer messages.** A per-point timestamp field costs 8 bytes on every point. Deskewing needs it on the input; if nothing downstream needs it, publishing without it saves 8 bytes times the point count.

## Regenerate the chart

```bash
python3 doc/media/render_perf_summary.py
```
