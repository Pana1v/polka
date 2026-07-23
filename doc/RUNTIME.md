# Runtime Reconfiguration & Diagnostics

## Runtime Reconfiguration

Every polka parameter can be changed live through the standard ROS 2 parameter
services — no custom service types, so `ros2 param`, rqt, and Foxglove all work:

```bash
# Tighten a source's range filter while the robot runs
ros2 param set /polka sources.front_3d.filters.range.enabled true
ros2 param set /polka sources.front_3d.filters.range.max 4.0

# Add a source at runtime: declare the name first, then set its topic
ros2 param set /polka source_names "['front_3d', 'rear_2d', 'aux']"
ros2 param set /polka sources.aux.topic "/aux_lidar/points"

# Remove it again (its parameters keep their values for a later re-add)
ros2 param set /polka source_names "['front_3d', 'rear_2d']"
```

Invalid values are rejected atomically with a human-readable reason
(`ros2 param set /polka sources.front_3d.filters.range.min -1.0` →
`"sources.front_3d.filters: min_range must be non-negative"`), and nothing is
applied until the whole proposed set validates.

How each parameter group takes effect:

| Parameter group | Applied by |
|---|---|
| `sources.<n>.filters.*` | Filter chain rebuilt in place |
| `sources.<n>.topic` / `.type` / `.qos_*` / `.imu_topic` | That source's subscription recreated |
| `source_names` | Sources added/removed by name (a new name without a topic stays *pending* until the topic is set) |
| `output_rate` | Output timer recreated |
| `outputs.*.enabled` / `.topic` / `.qos.*` | Publisher created/destroyed/recreated |
| `outputs.cloud.filters/voxel/height_cap/self_filter.*`, `outputs.scan.*` | Output pipeline reconfigured in place |
| `motion_compensation.*` | IMU buffer and/or source subscriptions recreated as needed |
| `diagnostics.*` | Drift trackers and diagnostics timer reconfigured |
| `timestamp_strategy`, `source_timeout`, `point_timestamps.*`, ... | Take effect on the next output tick |
| `enable_gpu` | **Read-only** — the merge engine is constructed once at startup |

Thread-safety note: all of polka's callbacks share the node's default mutually
exclusive callback group, so reconfiguration is serialized against data
processing under both single- and multi-threaded executors. Do not move
polka's callbacks into a reentrant group, and do not call `set_parameters()`
on the node object from a thread that is not spinning it.

## Diagnostics & Drift Detection

polka publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics` (default
1 Hz, `diagnostics.enabled: true`), readable by `rqt_robot_monitor`,
`diagnostic_aggregator`, PlotJuggler, or plain `ros2 topic echo`:

- **`polka: node`** — engine (CPU/CUDA), fresh/pending source counts, uptime, reconfigure count
- **`polka: output`** — publish rate, output bandwidth, points in/out per tick, last-publish age
- **`polka: source <name>`** — per-source rate, bandwidth, message age, stamp offset
  vs. peer median, filter drop percentage, status (`OK`, stale, pending, drifting),
  and which capabilities are actually active (range/angular/box filters, deskewing)

Two drift detectors run per source, each raising a `WARN` diagnostic plus a log
line only after `min_ticks` consecutive bad ticks (and clearing with
hysteresis, so a jittery boundary can not flap the flag):

- **Timing drift** — the EWMA of a source's stamp offset from the peer median
  exceeds `diagnostics.timing_drift.threshold_sec`. Catches a sensor whose
  clock or driver latency is walking away from the others.
- **Rate drift** — the observed rate sags more than `diagnostics.rate_drift.sag_pct`
  below the expected rate (`sources.<n>.expected_rate`, or auto-baselined from
  the first `baseline_sec` of traffic). Catches a lagging sensor that is alive
  but degraded — a fully dead source is flagged *stale* instead.

All thresholds are runtime-reconfigurable. See the `diagnostics:` block in
[config/detailed_params.yaml](../config/detailed_params.yaml).

## Terminal Dashboard

`polka_monitor` is an optional terminal UI built on the diagnostics above — no
GUI, no rviz, works over SSH and inside `docker exec`. It subscribes to
`/diagnostics`, `/rosout`, and the merged output topics (which it discovers
automatically from the diagnostics), so it never adds load to the merge path.

```bash
ros2 run polka polka_monitor            # any terminal, while polka runs
ros2 run polka polka_monitor --node polka --rate 4.0   # redraws/sec, default 4.0
```

On terminals at least 130 columns wide, the dashboard lays out in three
columns; narrower terminals drop the capabilities column:

- **Left — capabilities**: engine (CPU/CUDA), output topics, and per-source
  type/rate-mode/active-filters/deskew status, straight from `/diagnostics`.
- **Middle — views**: Braille-rendered top (x-y) and side (x-z) views of the
  merged cloud, with the merged scan overlaid on the top view. Density shades
  the dots; the extent auto-scales (toggle to a fixed extent with `f`).
- **Right**: a per-source table (rate, bandwidth, message age, stamp offset,
  filter drop %) colored by status, plus a live warning feed colored per
  source (when the message names one) that merges polka's `/rosout` warnings
  with stale/drift transitions.
- **Keys**: `q` quit, `f` fixed/auto extent, `v` toggle views, `p` pause feed.

It is opt-in and changes nothing about the node's normal operation. To run it in
the same terminal as a launch, pass `dashboard:=true` (the node's logs are
redirected to the log file so they don't fight the UI):

```bash
ros2 launch polka polka.launch.py dashboard:=true config_file:=<your>.yaml
```

Running `polka_monitor` in a second terminal is the simplest path and always
works; the `dashboard:=true` route additionally re-attaches the UI to the
controlling terminal, which `ros2 launch` otherwise does not provide.
