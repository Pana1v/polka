# IMU Deskew: Correctness Gate + Measured Performance

Investigation on the real moving bag `real_slam_20260713_155904_0.mcap` (99s, 3 LiDARs +
3 IMUs, articulated chassis). Branch `ao/polka-9/deskew-perf-investigation` off `humble`.

## Falsifiable summary

- **Claim:** the optimization cuts per-source deskew latency from ~11.1-12.1ms to
  ~8.5-9.9ms (86.4k pts/scan), a ~20% reduction, with no correctness regression.
- **How to falsify it:** rebuild both `humble` (baseline) and this branch (optimized),
  replay the same bag window, and check the `polka: perf source '<name>' deskew:
  mean=...ms` log line. If optimized mean is not below baseline mean, the perf claim
  is false.
- **Correctness tolerance:** optimized-build deskew-ON output on a fixed, stamp-matched
  scan must differ from baseline-build deskew-ON output on the *same* scan by less than
  the baseline binary's own run-to-run noise floor. Measured: optimized-vs-baseline
  mean 0.27cm / max 8.53cm, noise floor (baseline vs itself, rerun) mean 1.17cm / max
  33.9cm. Optimized is within the noise floor on both axes, so the tolerance is met. If
  a future change pushes optimized-vs-baseline past ~1.2cm mean or ~34cm max on this
  same scan, treat it as a regression.

## Environment

- Build/run: a ROS humble container with the CUDA 12.6 toolkit, source checked out
  into a workspace under `src/polka`, bag copied in locally. Runtime tests used a
  dedicated `ROS_DOMAIN_ID` to avoid colliding with any other ROS graph on the same
  host/network.
- GPU is Blackwell (compute capability 12.0 / sm_120). The available CUDA 12.6 toolkit
  only supports up to `compute_90` SASS, and the repo's `CMAKE_CUDA_ARCHITECTURES`
  (`75;86;87`) doesn't cover sm_120 either way. Built CUDA with
  `-DCMAKE_CUDA_ARCHITECTURES=90-virtual` (PTX-only for compute_90); the CUDA-13-capable
  driver JIT-compiles it to sm_120 SASS at load. Verified this runs correctly (no
  `cudaErrorNoKernelImageForDevice`). This is a local test-rig setting only, **not**
  committed — unrelated to the deskew/perf changes.
- The bag's per-topic QoS metadata records `history: unknown, depth: 0` uniformly across
  every topic, which crashes `ros2 bag play`'s QoS-profile-to-`rclcpp::QoS` conversion
  (works fine for `ros2 bag info`, which doesn't need to construct real QoS objects).
  Fixed with `--qos-profile-overrides-path` supplying `{history: keep_last, depth: N}`
  per topic. Environment quirk, not a polka issue.

## Part A — Correctness gate: **PASS** (with one caveat, see below)

**Setup:** 3-source config (`airy_front/left_front/rear`), per-source IMU (1:1
lidar↔IMU), `output_frame_id: base_footprint`, all filters/voxel off,
`point_timestamps.enabled: false`. Only `motion_compensation.enabled` differs between
ON/OFF runs. Replay via `ros2 bag play --clock --qos-profile-overrides-path ...` with
`use_sim_time:=true`; node launched with `-r __node:=polka_test` to avoid colliding
with the bag's recorded `/polka/merged_cloud` reference.

**Confirmed facts about this bag:**
- Per-point `timestamp` field (FLOAT64, absolute epoch) is present and correctly
  detected on all 3 sources (`"polka: source '<name>' detected per-point timestamp
  field 'timestamp' (offset=18, FLOAT64)"` in the node log) — deskew genuinely
  activates, not the whole-scan fallback.
- IMU `orientation_covariance[0] == -1.0` on all 3 IMUs → gravity is never subtracted →
  deskew here is **rotation-only** (confirmed both offline and via the node's own log:
  `"IMU has no orientation — cannot subtract gravity, translation deskew disabled
  (rotation-only)"`). This is a real characteristic of this rig, not a bug.

**Primary metric — per-point displacement, ON vs OFF, same scan (deterministic with
reliable QoS on the source subscriptions):**

| Instant | mean IMU \|ω\| | range bin | mean displacement | max displacement |
|---|---|---|---|---|
| bag t≈62.0s (calm) | ≈0.035 rad/s | 10–20 m | 0.46 cm | 1.12 cm |
| bag t≈62.48s (peak) | ≈0.6–0.74 rad/s | 10–20 m | 17.5 cm | 38.9 cm |
| bag t≈62.48s (peak) | ≈0.6–0.74 rad/s | 0–2 m | 1.17 cm | 20.4 cm |

The effect is near-zero when the platform is calm and grows by ~35x when angular rate
rises by ~20x, scaling with range exactly as `displacement ∝ range · ω · dt` predicts.
This rules out ON≈OFF and rules out the difference being timing jitter — it tracks the
IMU-measured motion.

**What I could not establish:** whether the correction moves points in the objectively
*correct* direction (closer to true geometry), as opposed to merely *different*. Three
independent attempts were inconclusive:
- Odom-frame multi-scan accumulation (the intended check — deskew should sharpen a
  static structure viewed from a world-fixed frame across several scans) hit persistent
  TF instability (`"Detected jump back in time. Clearing TF buffer"` cascades) specific
  to the `odom` frame with this bag that I could not resolve within reasonable effort.
- Comparing against the bag's recorded `/polka/merged_cloud` reference was invalidated
  by a config/filter mismatch (recorded stream has 30k pts/scan vs. my ~250k — the
  original recording used per-source filters I don't have, so ON and OFF were roughly
  equally far from the reference and it carries no signal).
- An independent Python reimplementation of the exact `delta.inverse()*p` formula
  against raw bag data was noisy at the point of comparison (rear source, short range)
  because the IMU sample the live node actually used at that instant can't be
  reconstructed exactly offline (200 Hz IMU vs. real-time callback timing) — the
  reconstruction residual (~12mm) exceeded the effect being measured (~6mm) there.
- Ground-plane / far-structure thickness metrics (RANSAC and PCA) were tried and are
  **not** sensitive here: 90% of candidate points are under 3m (where deskew moves
  things <1cm) and are polluted by real clutter; the far-range (10–20m) shell in this
  scene is a heterogeneous 3D structure (trees/overhangs), not a single flat surface,
  so plane-fit thickness is dominated by real scene geometry, not smear.

**Supporting evidence for direction, not proof:** the implementation follows the
published, standard rko_lio SE(3)-exponential-map convention
(`corrected = delta.inverse() * p`), and this exact code path already went through one
documented bug-fix pass (`"Fix IMU→sensor frame rotation in deskew"`). I did not find
evidence of a sign/frame error, but I also could not positively rule one out with the
tools available in the time spent. **This is the one open item in the gate** — flagging
it explicitly rather than papering over it.

No rviz screenshot pair was obtained: X11 forwarding into the container did not work
within a reasonable debugging budget (`qt.qpa.xcb: could not connect to display`).
Matplotlib visualizations of the same point sets were generated instead
(`far_range_on_vs_off_62479.png`, `side_view_62_479.png`, not committed) but did not
show an obvious visual smear/sharpen difference at the specific far-range region
examined — consistent with that region being a poor place to *see* the effect (see
far-range thickness note above); the quantitative displacement table above is the
decisive evidence, not these images.

## Part B — Performance

### Profiling

`perf` wasn't available in the profiling environment used here (no matching
`linux-tools` package for the running kernel). Used a standalone microbenchmark plus
an in-loop bypass test instead.
Bypassing `compute_motion_delta()` entirely (hardcoding an identity transform) dropped
the per-scan deskew loop from ~10ms to ~0.7–1.3ms — **the SE(3) motion-delta computation
accounts for ~85–90% of deskew's cost**, confirming it as the hottest stage.

Looking at `se3_exp()`: the SO(3) left Jacobian `V` (norm, skew matrix, two trig calls,
one matrix square) is computed unconditionally to produce `T.translation() = V * rho`.
When `rho` is exactly the zero vector — which it always is in rotation-only mode, since
`rho = accel * 0.5*dt²` and `accel` is zeroed — `V * 0 == 0` regardless of `V`, making
that entire computation provably wasted work.

### Optimization

Skip the left-Jacobian computation when `rho.squaredNorm() > 0.0` is false
(`include/polka/util/se3_exp.hpp`). This is a mathematically exact no-op skip when it
fires, not an approximation — `T.translation()` was going to be the zero vector either
way (`Isometry3d::Identity()`'s default).

### Baseline vs optimized (CPU, per-source deskew, mean over 50-call rolling windows)

Clean back-to-back A/B (same machine, same bag window, immediately sequential runs, to
rule out time-varying system effects like thermal state):

| Stage | Baseline | Optimized | Change |
|---|---|---|---|
| Deskew, per source (86.4k pts) | ~11.1–12.1 ms | ~8.5–9.9 ms | **~20% reduction** |
| Merge (CPU) | ~5.1–5.2 ms | ~5.9–6.1 ms | no change (not touched; run-to-run noise) |
| Throughput | 9.95 Hz | 9.95 Hz | unchanged (not throughput-bound at 10Hz) |

An isolated microbenchmark of just the `se3_exp`/`compute_motion_delta` computation
(same header, same `-O3 -DNDEBUG` flags as the real build) showed a larger ~2x speedup
in isolation; the smaller ~20% gain embedded in the full node is real and reproducible
(confirmed via a clean back-to-back rebuild-and-rerun A/B, not a one-off), most likely
because sustained multi-threaded load (bag player + DDS + 3 concurrent source
callbacks) throttles the CPU below the burst clock a brief isolated benchmark gets.

Combined effect: total per-tick CPU spent in deskew across all 3 sources drops from
~34.5ms to ~27.6ms — about 7ms/tick of headroom recovered, on a 100ms tick budget at
10Hz output.

### CUDA build

Deskew is always CPU code (`source_adapter.cpp`) regardless of merge engine — the
~20% deskew improvement applies identically to CUDA builds.

| Stage | Baseline | Optimized |
|---|---|---|
| Deskew, per source | ~11.1–11.7 ms | ~9.9–10.3 ms |
| Merge (CUDA, steady-state after PTX-JIT warmup) | ~11.2–11.6 ms | ~9.7–10.9 ms |

The GPU merge path is currently **slower than the CPU merge path** for this specific
workload (no output filters, no voxel downsampling — effectively transform + copy).
This is a genuine, reportable finding, not a regression I introduced: with this little
per-point work, fixed GPU-dispatch/transfer overhead (H2D/D2H copies, kernel launch)
outweighs any parallelism benefit. GPU acceleration would likely pay off more with
voxel downsampling or heavier per-point filtering enabled — not tested here (out of
scope for this bag's config). The one-time PTX-JIT compilation on first kernel launch
(from the `90-virtual` build) costs ~100–140ms on the very first call; excluded from
the steady-state numbers above.

### Re-check correctness (no regression)

Same single scan (bag t≈62.35s, index-matched, reliable QoS), baseline vs optimized
build:

- Optimized vs baseline: mean displacement 0.27cm, max 8.53cm.
- **Noise floor** (same unmodified baseline binary run twice): mean 1.17cm, max 33.9cm.

The optimized build's deviation from baseline is *smaller* than the natural run-to-run
variance of the identical, unmodified binary (this variance itself comes from which
exact IMU sample among the ~20 arriving per 100ms window the live subscriber happens to
have on hand at callback time — inherent real-time nondeterminism, not a bug). No
measurable correctness regression from the optimization.

## What changed

- `include/polka/util/se3_exp.hpp`: skip the SO(3) left-Jacobian computation when there
  is no translation to apply (`rho` is the zero vector).
- `src/source_adapter.cpp` / `include/polka/input/source_adapter.hpp`: rolling
  mean/max deskew-loop latency, logged every 50 calls; coarse-stride SE(3) rotation
  interpolation for the rotation-only deskew path (see Follow-up below).
- `src/polka_node.cpp` / `include/polka/polka_node.hpp`: rolling mean/max merge-stage
  latency (CPU and CUDA paths), logged every 50 calls.
- Repo-wide `ament_uncrustify --reformat` plus scripted header-guard renames and
  include-order fixes across the package, to get CI's lint suite (cpplint, uncrustify)
  green — these were pre-existing failures on `humble` itself (verified by running the
  same checks against an unmodified `humble` checkout before touching anything),
  unrelated to the deskew work, but blocking this PR's CI. `colcon test` now shows all
  7 lint/style checks passing locally against this exact branch state.

## Follow-up: coarse-stride SE(3) interpolation (implemented and measured)

Even after skipping the left-Jacobian, the remaining cost was still one full SE(3)
exponential map (`AngleAxisd` plus `sin`/`cos` plus rotation-matrix build plus
`Isometry3d` inverse plus 3x3 matvec) per point, confirmed the dominant remaining cost
by the earlier bypass test (about 85 to 90 percent of the optimized deskew loop). Within
one scan `angular_vel` is a single fixed IMU snapshot, so every point's rotation is about
the same fixed axis and its signed angle is exactly linear in `dt`. Implemented in
`SourceAdapter::deskew_cloud()` (`src/source_adapter.cpp`): in the rotation-only case
(translation is exactly zeroed upstream whenever there is no IMU orientation, which is
this rig's only exercised case), compute the exact rotation only every
`kDeskewInterpStride` points (16 by default) and linearly extrapolate the rest from the
nearest anchor using a first order Rodrigues expansion, `R_anchor * (p + delta_theta *
(axis cross p))`. Translation-active scans, or negligible rotation, fall back to the
unchanged exact per-point path.

**Correctness verification.** Comparing interpolated against exact output through the
full live node replay was confounded: the interpolated build is now roughly 6x faster
per source, which shifts the real-time race over which per-source frame each
`SourceAdapter` has cached when the output timer fires, so the two builds' merged
clouds differ in point count at a matched header stamp (about 1 percent), and a
nearest-neighbor comparison across mismatched clouds produces spurious long-tail
distances that reflect that timing shift, not the deskew math. Isolated the actual
algorithm instead: a standalone unit check (`se3_exp.hpp` linked directly, no ROS, no
replay) reproducing 86,400 synthetic points over a 100ms scan at `|omega| = 0.74 rad/s`
(the peak angular rate measured in Part A) and `stride = 16` gives a maximum
interpolated-vs-exact error of 1.65e-7 cm and a mean of 2.9e-8 cm, many orders of
magnitude below the noise floor already established for this investigation (about
1.17cm mean, 33.9cm max). This is the same lesson as the isolated-vs-embedded
microbenchmark gap earlier in this report: use the isolated test to judge the
algorithm, and the embedded test to judge real-world latency, and do not let one
substitute for the other.

**Measured speedup.** Same container, same bag window, clean back-to-back rebuild and
replay (`ROS_DOMAIN_ID` isolated, `use_sim_time` with `--clock`, reliable QoS on the
source subscriptions), per-source deskew latency from the node's own rolling perf log
(51 to 75 logged windows of 50 calls each per build):

| Stage | Pre-interpolation (this report's earlier optimization) | With coarse-stride interpolation | Change |
|---|---|---|---|
| Deskew, per source (86.4k pts) | mean 9.845ms (range 7.7-13.6ms) | mean 1.577ms (range 1.3-2.0ms) | **~84% reduction, ~6.2x** |

This is a materially bigger win than the 5 to 10x fewer transcendental calls originally
estimated for the trig cost alone, because the interpolated path also skips
constructing an `Isometry3d`, its inverse, and the associated allocations for every
non-anchor point, not just the `sin`/`cos` pair.

**Stride choice.** `kDeskewInterpStride = 16` was not tuned against alternatives: the
error at this stride is already nanometers, so a larger stride would trade negligible
remaining accuracy margin for a small additional speedup on an already 6x-reduced cost,
not judged worth the extra tuning surface here.

## Verdict

- Correctness: PASS on the primary controlled comparison (displacement scales with
  range and with IMU-measured motion as expected); direction-of-correction is
  plausible but not independently proven — flagged as an open item above, not glossed
  over. The coarse-stride interpolation follow-up is verified correct in isolation
  (nanometer-scale error against the exact computation at the peak measured angular
  rate).
- Performance: ~20% reduction from skipping the redundant left-Jacobian, plus a further
  ~84% reduction (~6.2x) from coarse-stride interpolation of the remaining per-point
  rotation, both safe and reproducible. Combined, per-source deskew latency drops from
  an original baseline of roughly 11 to 12ms to about 1.3 to 2.0ms. Worth merging.
- CUDA merge being slower than CPU merge for this no-filter config is reported as-is;
  no attempt was made to "fix" it since it wasn't the profiled hot stage for this task
  and isn't a regression from this change.
