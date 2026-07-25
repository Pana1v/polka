# Performance

Two measured improvements shipped in 0.5.0. Only the published 0.5.0 numbers are quoted anywhere in this document, and each is listed with its source and with what it does and does not mean.

## Measured numbers

| Change | Before | After | Factor | Source |
|---|---|---|---|---|
| Deskew stage latency (per source) | 9.8 ms | 1.6 ms | ~6.2x cheaper | CHANGELOG 0.5.0 |
| CPU angular filter (per tick, 259k pts) | 10.47 ms | 3.55 ms | ~3x cheaper | CHANGELOG 0.5.0 |

Both figures are per-stage latency: the same computation, made faster by a code change in 0.5.0. Voxel downsampling is a different kind of thing, a data volume tradeoff covered under bandwidth below, not a code speedup, so it is not listed here.

The angular filter win comes from replacing a per-point `atan2` with a precomputed cross-product half-plane test; the deskew win comes from coarse-stride SE(3) rotation interpolation instead of recomputing the pose at every point (max error about 1.6e-7 cm). Both are on the CPU path.

## Two things "deskew" means

These are separate claims. Keep them apart.

**(a) The deskew computation got 6.2x cheaper.** This is a code optimization. Correcting every point used to recompute a full SE(3) pose per point; 0.5.0 interpolates the rotation on a coarse stride instead, cutting the stage from about 9.8 ms to about 1.6 ms per source with negligible accuracy loss. It makes the correction cheaper to run. It says nothing about how much distortion the correction removes.

**(b) The distortion that deskewing removes is a quality benefit.** A spinning or moving LiDAR samples its points across a few tens of milliseconds. If the sensor rotates or translates during that sweep, a rigid scan smears structure across the frame. Per-point SE(3) correction warps each point by the pose at its own timestamp and removes that intra-scan smear. The `deskew.gif` in the README shows exactly this: raw versus deskewed under a synthetic 1 rad/s yaw. This is what deskewing buys you.

Do not read the 6.2x as the size of the quality gain. 6.2x is how much faster the correction computes, not how much straighter the cloud looks. The gif is the quality claim; the 6.2x is the cost claim. They are unrelated magnitudes.

## CUDA: not universally faster

polka can build a CUDA merge engine with `-DWITH_CUDA=ON` that runs the whole per-point path (transform, filter, voxel, scan flatten) as one fused GPU pass.

On heavy pipelines, where many points flow through several filters, that fused pass wins: the GPU hides per-point work that the CPU would pay for serially. On a filterless merge the CPU stays competitive: with little per-point work to do, kernel dispatch and host to device transfer overhead dominate, and the GPU has nothing to hide them behind.

So CUDA is a crossover, not a free win. It falls back to CPU automatically when built without CUDA or when no device is present. No CUDA timing number is published here; treat the choice as workload dependent and measure on your own pipeline.

## Bandwidth: N streams into one lighter topic

Fusion is a bandwidth win independent of raw compute speed.

- **N streams to 1 topic.** Downstream nodes subscribe once to the merged output instead of to every raw sensor. One frame, one QoS, one message to reason about.
- **Voxel downsampling.** This filter is not new in 0.5.0; it has always been available, and it is a quality and bandwidth tradeoff the user sets through `leaf_size`. At the leaf size chosen for the demo clip the cloud drops from 69k to 5k points (about 14x fewer), but that ratio is specific to that leaf size, not a fixed or guaranteed figure: a larger leaf thins more, a smaller leaf thins less. It is not a code speedup.
- **Slimmer messages.** A per-point timestamp field costs 8 bytes on every point. Deskewing needs it on the input, but when downstream consumers do not, a cloud published without that field is smaller by 8 bytes times the point count.

## Regenerate

Regenerate: python3 doc/media/render_perf_summary.py
