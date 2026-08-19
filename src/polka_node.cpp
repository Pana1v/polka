// Copyright 2025 Panav Arpit Raaj <praajarpit@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "polka/polka_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <pcl/common/io.h>
#ifdef POLKA_CUDA_ENABLED
#include <cuda_runtime.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <utility>

#include "polka/util/log_format.hpp"
#include "polka/util/qos_builder.hpp"
#include "polka/util/se3_exp.hpp"
#include "polka/merge_engine/cpu_merge_engine.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#ifdef POLKA_CUDA_ENABLED
#include "polka/merge_engine/cuda_merge_engine.hpp"
#endif

namespace polka
{

namespace
{

double steady_seconds(std::chrono::steady_clock::time_point tp)
{
  return std::chrono::duration<double>(tp.time_since_epoch()).count();
}

bool qos_equal(const OutputQosConfig & a, const OutputQosConfig & b)
{
  return a.reliability == b.reliability && a.durability == b.durability &&
         a.history_depth == b.history_depth && a.liveliness == b.liveliness &&
         a.liveliness_lease_duration_ms == b.liveliness_lease_duration_ms &&
         a.deadline_ms == b.deadline_ms && a.lifespan_ms == b.lifespan_ms;
}

bool drift_params_equal(const DiagnosticsConfig & a, const DiagnosticsConfig & b)
{
  return a.timing_threshold_sec == b.timing_threshold_sec &&
         a.timing_ewma_alpha == b.timing_ewma_alpha &&
         a.timing_min_ticks == b.timing_min_ticks &&
         a.rate_sag_pct == b.rate_sag_pct &&
         a.rate_min_ticks == b.rate_min_ticks &&
         a.rate_baseline_sec == b.rate_baseline_sec;
}

bool any_source_has_own_imu(const std::vector<SourceConfig> & sources)
{
  return std::any_of(
    sources.begin(), sources.end(),
    [](const SourceConfig & sc) {return !sc.imu_topic.empty();});
}

}  // namespace

PolkaNode::PolkaNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("polka", options), config_loader_(this)
{
  // Our point type carries a 'time' field that source clouds usually lack (they
  // publish 'timestamp' or nothing), so pcl::fromROSMsg logs a per-message
  // "Failed to find match for field 'time'" warning. populate_point_time fills it
  // explicitly, so silence PCL's redundant warnings (errors still surface).
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  start_steady_ = std::chrono::steady_clock::now();
  config_ = config_loader_.load();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

#ifdef POLKA_CUDA_ENABLED
  if (config_.enable_gpu) {
    int device_count = 0;
    cudaGetDeviceCount(&device_count);
    if (device_count > 0) {
      merge_engine_ = std::make_unique<CudaMergeEngine>(config_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "polka: enable_gpu=true but no CUDA device found, falling back to CPU");
    }
  }
#endif
  if (!merge_engine_) {
    merge_engine_ = std::make_unique<CpuMergeEngine>();
  }

  if (config_.motion_compensation.enabled && !config_.motion_compensation.imu_topic.empty()) {
    global_imu_ = std::make_shared<ImuBuffer>(
      this, config_.motion_compensation.imu_topic,
      config_.motion_compensation.imu_buffer_size);
  } else if (config_.motion_compensation.enabled) {
    if (any_source_has_own_imu(config_.sources)) {
      RCLCPP_INFO(
        get_logger(),
        "polka: no global imu_topic; deskewing only sources with a per-source imu_topic");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "polka: motion compensation enabled but imu_topic is empty, deskewing will not activate");
    }
  }

  for (const auto & src_cfg : config_.sources) {
    SourceSlot slot;
    slot.adapter = make_adapter(src_cfg, config_);
    slot.drift.set_config(drift_config(src_cfg, config_.diagnostics));
    slot.created_at = std::chrono::steady_clock::now();
    sources_.push_back(std::move(slot));
    source_names_.push_back(src_cfg.name);
  }

  output_pipeline_.configure(config_.cloud_output);
  scan_builder_.configure(config_.scan_output, config_.output_rate, config_.output_frame_id);

  if (config_.cloud_output.enabled) {
    cloud_pub_ = create_cloud_publisher(
      this, config_.cloud_output.topic, build_qos(config_.cloud_output.qos));
  }
  if (config_.scan_output.enabled) {
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      config_.scan_output.topic, build_qos(config_.scan_output.qos));
  }

  if (config_.output_rate > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / config_.output_rate);
    output_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PolkaNode::output_callback, this));
  }

  configure_diagnostics(config_.diagnostics);
  log_startup_banner();

  param_cb_ = add_on_set_parameters_callback(
    std::bind(&PolkaNode::validate_parameters, this, std::placeholders::_1));
}

SourceAdapter::ImuGetter PolkaNode::make_imu_getter(const MergeConfig & cfg)
{
  if (!cfg.motion_compensation.enabled || !cfg.motion_compensation.per_point_deskew) {
    return nullptr;
  }
  // Resolve global_imu_ at call time, not capture time: a runtime reconfigure
  // may reset or replace the buffer while adapters keep this getter.
  return [this]() -> std::shared_ptr<const AveragedImu> {
           auto imu = global_imu_;
           return imu ? imu->snapshot() : nullptr;
         };
}

std::unique_ptr<SourceAdapter> PolkaNode::make_adapter(
  const SourceConfig & sc, const MergeConfig & cfg)
{
  const auto & mc = cfg.motion_compensation;
  return std::make_unique<SourceAdapter>(
    this, sc, merge_engine_->is_gpu(), make_imu_getter(cfg),
    mc.enabled && mc.per_point_deskew,
    mc.deskew_timestamp_field, tf_buffer_, mc.imu_buffer_size);
}

DriftTracker::Config PolkaNode::drift_config(
  const SourceConfig & sc, const DiagnosticsConfig & d) const
{
  DriftTracker::Config c;
  c.timing_threshold_sec = d.timing_threshold_sec;
  c.timing_ewma_alpha = d.timing_ewma_alpha;
  c.timing_min_ticks = d.timing_min_ticks;
  c.rate_sag_pct = d.rate_sag_pct;
  c.rate_min_ticks = d.rate_min_ticks;
  c.rate_baseline_sec = d.rate_baseline_sec;
  c.expected_rate = sc.expected_rate;
  return c;
}

rcl_interfaces::msg::SetParametersResult PolkaNode::validate_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  // apply_reconfigure() declares parameters for newly added sources, which
  // re-enters this callback; those declares carry defaults and need no check.
  if (applying_) {return result;}

  std::vector<std::string> names = source_names_;
  for (const auto & p : params) {
    if (p.get_name() == "source_names") {
      names = p.as_string_array();
    }
  }

  try {
    config_loader_.preview(params, names);
  } catch (const std::exception & ex) {
    result.successful = false;
    result.reason = ex.what();
    return result;
  }

  schedule_apply();
  return result;
}

void PolkaNode::schedule_apply()
{
  if (!apply_timer_) {
    apply_timer_ = create_wall_timer(
      std::chrono::nanoseconds(1),
      [this]() {
        apply_timer_->cancel();  // one-shot; re-armed via reset() next time
        apply_reconfigure();
      });
  } else {
    apply_timer_->reset();
  }
}

void PolkaNode::apply_reconfigure()
{
  applying_ = true;
  struct Guard { bool & flag; ~Guard() {flag = false;} } guard{applying_};

  auto new_names = get_parameter("source_names").as_string_array();

  MergeConfig new_config;
  try {
    new_config = config_loader_.reload(new_names);
  } catch (const std::exception & ex) {
    // preview() vetted these values before the commit; reaching here means
    // storage changed in a way preview could not see. Keep the old config.
    RCLCPP_ERROR(get_logger(), "polka: reconfigure failed: %s", ex.what());
    return;
  }

  MergeConfig old_config = config_;
  std::vector<std::string> changes;

  if (new_config.output_rate != old_config.output_rate && new_config.output_rate > 0.0) {
    if (output_timer_) {output_timer_->cancel();}
    auto period = std::chrono::duration<double>(1.0 / new_config.output_rate);
    output_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PolkaNode::output_callback, this));
    char buf[64];
    std::snprintf(buf, sizeof(buf), "output_rate=%.1fHz", new_config.output_rate);
    changes.emplace_back(buf);
  }

  // Publishers: recreate on topic/QoS change, destroy on disable.
  if (!new_config.cloud_output.enabled) {
    if (cloud_pub_) {
      shutdown_cloud_publisher(cloud_pub_);
      changes.emplace_back("cloud_output=off");
    }
  } else {
    const bool recreate = !cloud_pub_ ||
      new_config.cloud_output.topic != old_config.cloud_output.topic ||
      !qos_equal(new_config.cloud_output.qos, old_config.cloud_output.qos);
    if (recreate) {
      changes.emplace_back(
        cloud_pub_ ?
        "cloud_output='" + new_config.cloud_output.topic + "'" : "cloud_output=on");
      cloud_pub_ = create_cloud_publisher(
        this, new_config.cloud_output.topic, build_qos(new_config.cloud_output.qos));
    }
  }
  if (!new_config.scan_output.enabled) {
    if (scan_pub_) {
      scan_pub_.reset();
      changes.emplace_back("scan_output=off");
    }
  } else {
    const bool recreate = !scan_pub_ ||
      new_config.scan_output.topic != old_config.scan_output.topic ||
      !qos_equal(new_config.scan_output.qos, old_config.scan_output.qos);
    if (recreate) {
      changes.emplace_back(
        scan_pub_ ?
        "scan_output='" + new_config.scan_output.topic + "'" : "scan_output=on");
      scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
        new_config.scan_output.topic, build_qos(new_config.scan_output.qos));
    }
  }

#ifdef POLKA_CUDA_ENABLED
  // The CUDA engine sizes its device buffers from the source count at
  // construction; a changed source set needs a rebuilt engine or a runtime
  // add would overflow them.
  if (merge_engine_ && merge_engine_->is_gpu() && new_names != source_names_) {
    merge_engine_ = std::make_unique<CudaMergeEngine>(new_config);
    changes.emplace_back("merge_engine=rebuilt");
  }
#endif

  // Global IMU buffer.
  const auto & new_mc = new_config.motion_compensation;
  const auto & old_mc = old_config.motion_compensation;
  const bool imu_was = (global_imu_ != nullptr);
  const bool imu_now = new_mc.enabled && !new_mc.imu_topic.empty();
  if (imu_now && (!imu_was || new_mc.imu_topic != old_mc.imu_topic ||
    new_mc.imu_buffer_size != old_mc.imu_buffer_size))
  {
    global_imu_ = std::make_shared<ImuBuffer>(this, new_mc.imu_topic, new_mc.imu_buffer_size);
    changes.emplace_back(
      imu_was ?
      "imu_topic='" + new_mc.imu_topic + "'" : "motion_compensation=on");
  } else if (!imu_now && imu_was) {
    // Adapters read global_imu_ through the null-safe getter, so dropping the
    // buffer while they hold the getter is fine.
    global_imu_.reset();
    changes.emplace_back("motion_compensation=off");
  }
  if (new_mc.enabled && new_mc.imu_topic.empty() &&
    !(old_mc.enabled && old_mc.imu_topic.empty()) &&
    !any_source_has_own_imu(new_config.sources))
  {
    RCLCPP_WARN(
      get_logger(),
      "polka: motion compensation enabled but imu_topic is empty, deskewing will not activate");
  }

  rebuild_sources(old_config, new_config, changes);

  if (new_config.output_frame_id != old_config.output_frame_id) {
    // Cached fallback transforms target the old frame; invalidate them.
    for (auto & slot : sources_) {
      slot.last_good_transform = Eigen::Isometry3d::Identity();
    }
    changes.emplace_back("output_frame_id='" + new_config.output_frame_id + "'");
  }

  output_pipeline_.configure(new_config.cloud_output);
  scan_builder_.configure(
    new_config.scan_output, new_config.output_rate, new_config.output_frame_id);
  configure_diagnostics(new_config.diagnostics);

  config_ = std::move(new_config);
  source_names_ = new_names;
  ++reconfig_count_;

  if (changes.empty()) {
    RCLCPP_INFO(get_logger(), "polka: reconfigured (filters only)");
  } else {
    std::string joined;
    for (size_t i = 0; i < changes.size(); ++i) {
      if (i) {joined += ", ";}
      joined += changes[i];
    }
    RCLCPP_INFO(get_logger(), "polka: reconfigured — %s", joined.c_str());
  }
}

void PolkaNode::rebuild_sources(
  const MergeConfig & old_config, const MergeConfig & new_config,
  std::vector<std::string> & changes)
{
  const bool deskew_wiring_changed =
    old_config.motion_compensation.enabled != new_config.motion_compensation.enabled ||
    old_config.motion_compensation.per_point_deskew !=
    new_config.motion_compensation.per_point_deskew ||
    old_config.motion_compensation.deskew_timestamp_field !=
    new_config.motion_compensation.deskew_timestamp_field ||
    old_config.motion_compensation.imu_buffer_size !=
    new_config.motion_compensation.imu_buffer_size;
  const bool drift_cfg_changed =
    !drift_params_equal(old_config.diagnostics, new_config.diagnostics);

  std::vector<SourceSlot> new_slots;
  new_slots.reserve(new_config.sources.size());

  for (const auto & sc : new_config.sources) {
    int old_idx = -1;
    for (size_t j = 0; j < old_config.sources.size(); ++j) {
      if (old_config.sources[j].name == sc.name) {
        old_idx = static_cast<int>(j);
        break;
      }
    }

    SourceSlot slot;
    const bool reusable = old_idx >= 0 && sources_[old_idx].adapter &&
      sc.same_identity(old_config.sources[old_idx]) && !deskew_wiring_changed;

    if (reusable) {
      slot = std::move(sources_[old_idx]);
      slot.adapter->rebuild_filters(sc.filter_params);
      if (drift_cfg_changed ||
        sc.expected_rate != old_config.sources[old_idx].expected_rate)
      {
        slot.drift.set_config(drift_config(sc, new_config.diagnostics));
      }
    } else {
      if (!sc.topic.empty()) {
        slot.adapter = make_adapter(sc, new_config);
        if (old_idx < 0) {
          changes.emplace_back("source '" + sc.name + "' added");
        } else if (sources_[old_idx].adapter) {
          changes.emplace_back("source '" + sc.name + "' recreated");
        } else {
          changes.emplace_back("source '" + sc.name + "' activated");
        }
      } else {
        if (old_idx < 0) {
          RCLCPP_WARN(
            get_logger(),
            "polka: source '%s' added but pending until sources.%s.topic is set",
            sc.name.c_str(), sc.name.c_str());
        }
      }
      slot.drift.set_config(drift_config(sc, new_config.diagnostics));
      slot.created_at = std::chrono::steady_clock::now();
    }
    new_slots.push_back(std::move(slot));
  }

  for (const auto & old_sc : old_config.sources) {
    bool kept = false;
    for (const auto & sc : new_config.sources) {
      if (sc.name == old_sc.name) {kept = true; break;}}
    if (!kept) {
      RCLCPP_INFO(get_logger(), "polka: source '%s' removed", old_sc.name.c_str());
      changes.emplace_back("source '" + old_sc.name + "' removed");
    }
  }

  // Old slots (including removed adapters and their subscriptions) destroyed here.
  sources_ = std::move(new_slots);
}

void PolkaNode::configure_diagnostics(const DiagnosticsConfig & d)
{
  if (!d.enabled) {
    diag_timer_.reset();
    diag_reporter_.reset();
    return;
  }
  if (!diag_reporter_) {
    diag_reporter_ = std::make_unique<DiagnosticsReporter>(this);
  }
  auto period = std::chrono::duration<double>(d.publish_period_sec);
  diag_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PolkaNode::diag_callback, this));
}

void PolkaNode::mark_published()
{
  ever_published_ = true;
  last_publish_steady_ = std::chrono::steady_clock::now();
}

void PolkaNode::diag_callback()
{
  const auto steady_now = std::chrono::steady_clock::now();
  const double now_steady_sec = steady_seconds(steady_now);
  const auto now = this->now();

  // First pass: sample every live source, collect fresh stamps for the peer
  // median (needs >= 2 fresh sources to mean anything).
  struct Snapshot
  {
    StatWindow::Rates rates;
    bool received = false;
    bool stale = false;
    double stamp_sec = 0.0;
  };
  std::vector<Snapshot> snaps(sources_.size());
  std::vector<double> fresh_stamps;
  for (size_t i = 0; i < sources_.size(); ++i) {
    auto & slot = sources_[i];
    if (!slot.adapter) {continue;}
    auto & snap = snaps[i];
    snap.rates = slot.stats_window.update(slot.adapter->stats(), now_steady_sec);
    snap.received = slot.adapter->received();
    snap.stale = slot.adapter->is_stale(config_.source_timeout, now);
    if (snap.received) {
      snap.stamp_sec = slot.adapter->last_stamp().seconds();
      if (!snap.stale) {fresh_stamps.push_back(snap.stamp_sec);}
    }
  }

  bool have_median = fresh_stamps.size() >= 2;
  double median = 0.0;
  if (have_median) {
    auto mid = fresh_stamps.begin() + fresh_stamps.size() / 2;
    std::nth_element(fresh_stamps.begin(), mid, fresh_stamps.end());
    median = *mid;
  }

  const double receive_grace_sec = std::max(5.0 * config_.source_timeout, 10.0);

  std::vector<SourceReport> reports;
  reports.reserve(sources_.size());
  size_t fresh_count = 0;

  for (size_t i = 0; i < sources_.size(); ++i) {
    auto & slot = sources_[i];
    const auto & sc = config_.sources[i];
    const auto & snap = snaps[i];

    SourceReport r;
    r.name = sc.name;
    r.topic = sc.topic;
    r.type_str = (sc.type == SourceType::POINTCLOUD2) ? "pointcloud2" : "laserscan";

    if (!slot.adapter) {
      r.pending = true;
      r.drift = slot.drift.status();
      reports.push_back(std::move(r));
      continue;
    }

    r.frame_id = slot.adapter->frame_id();
    r.fields_invalid = slot.adapter->fields_invalid();
    r.filter_range_enabled = sc.filter_params.range_filter_enabled;
    r.filter_angular_enabled = sc.filter_params.angular_filter_enabled;
    r.filter_box_enabled = sc.filter_params.box_filter_enabled;
    r.deskew_active = slot.adapter->deskew_active();
    r.ever_received = snap.received;
    r.receive_overdue = !snap.received &&
      std::chrono::duration<double>(steady_now - slot.created_at).count() > receive_grace_sec;
    r.stale = snap.received && snap.stale;
    if (snap.received && !snap.stale) {++fresh_count;}

    r.rates = snap.rates;
    if (merge_engine_->is_gpu() && r.rates.valid) {
      // Per-source filters run inside the GPU engine; the kept count is unknown.
      r.rates.points_kept_per_sec = -1.0;
    }
    r.msg_age_sec = snap.received ? (now - slot.adapter->last_stamp()).seconds() : -1.0;

    if (have_median && snap.received && !snap.stale) {
      r.offset_valid = true;
      r.stamp_offset_sec = snap.stamp_sec - median;
    }

    DriftTracker::Input din;
    din.has_offset = r.offset_valid;
    din.stamp_offset_sec = r.stamp_offset_sec;
    // Rate drift means "alive but degraded"; a stale source is already flagged
    // harder, so its collapsing rate must not also trip the drift detector.
    din.has_rate = snap.rates.valid && snap.received && !snap.stale;
    din.rate_hz = snap.rates.msg_hz;
    din.tick_period_sec = config_.diagnostics.publish_period_sec;

    const auto & ds = slot.drift.update(din);
    r.drift = ds;

    if (ds.timing_raised) {
      RCLCPP_WARN(
        get_logger(),
        "polka: source '%s' timing drift: offset %+.3f s from peer median (threshold %.3f s)",
        sc.name.c_str(), ds.offset_ewma_sec, config_.diagnostics.timing_threshold_sec);
    }
    if (ds.timing_cleared) {
      RCLCPP_INFO(
        get_logger(),
        "polka: source '%s' timing drift cleared (offset %+.3f s)",
        sc.name.c_str(), ds.offset_ewma_sec);
    }
    if (ds.rate_raised) {
      RCLCPP_WARN(
        get_logger(),
        "polka: source '%s' rate drift: %.1f Hz < expected %.1f Hz",
        sc.name.c_str(), snap.rates.msg_hz, ds.expected_rate);
    }
    if (ds.rate_cleared) {
      RCLCPP_INFO(
        get_logger(),
        "polka: source '%s' rate drift cleared (%.1f Hz)",
        sc.name.c_str(), snap.rates.msg_hz);
    }

    reports.push_back(std::move(r));
  }

  OutputReport out;
  out.engine = merge_engine_->is_gpu() ? "CUDA" : "CPU";
  out.cloud_topic = cloud_pub_ ? cloud_publisher_topic(cloud_pub_) : "";
  out.scan_topic = scan_pub_ ? scan_pub_->get_topic_name() : "";
  out.cloud_rates = cloud_out_window_.update(cloud_out_counters_.sample(), now_steady_sec);
  out.scan_rates = scan_out_window_.update(scan_out_counters_.sample(), now_steady_sec);
  out.points_in = last_points_in_;
  out.points_out = last_points_out_;
  const double since_pub = std::chrono::duration<double>(
    steady_now - (ever_published_ ? last_publish_steady_ : start_steady_)).count();
  out.last_publish_age_sec = ever_published_ ? since_pub : -1.0;
  out.publish_overdue = fresh_count > 0 && config_.output_rate > 0.0 &&
    since_pub > std::max(3.0 / config_.output_rate, 1.0);

  NodeReport nr;
  nr.engine = out.engine;
  nr.sources_total = sources_.size();
  nr.sources_fresh = fresh_count;
  for (const auto & slot : sources_) {
    if (!slot.adapter) {++nr.sources_pending;}}
  nr.output_rate_hz = config_.output_rate;
  nr.uptime_sec = std::chrono::duration<double>(steady_now - start_steady_).count();
  nr.reconfig_count = reconfig_count_;

  ImuReport imu_rep;
  imu_rep.enabled = global_imu_ != nullptr;
  if (imu_rep.enabled) {
    imu_rep.topic = global_imu_->topic();
    auto imu_rates = imu_stat_window_.update(
      StatSample{global_imu_->msg_count(), 0, 0, 0}, now_steady_sec);
    imu_rep.rate_hz = imu_rates.valid ? imu_rates.msg_hz : -1.0;
    auto snap = global_imu_->snapshot();
    imu_rep.valid = snap && snap->valid;
    imu_rep.msg_age_sec = imu_rep.valid ? (now - global_imu_->last_stamp()).seconds() : -1.0;
  }

  diag_reporter_->publish(nr, out, imu_rep, reports, now);
}

void PolkaNode::diagnose_clock_health(const rclcpp::Time & now)
{
  if (clock_diagnosed_) {return;}

  // Newest sensor stamp across sources that have actually received data. Without any
  // data there is nothing to compare the clock against, so we can't judge yet.
  rclcpp::Time newest;
  bool any = false;
  for (const auto & slot : sources_) {
    if (!slot.adapter || !slot.adapter->received()) {continue;}
    auto stamp = slot.adapter->last_stamp();
    if (!any || stamp > newest) {newest = stamp; any = true;}
  }
  if (!any) {return;}

  const double dt = (now - newest).seconds();
  const bool sim = this->get_parameter("use_sim_time").as_bool();

  // Allow normal jitter (and the correct --clock case): only react when the clock and
  // the data disagree by several staleness windows.
  const double mismatch = std::max(2.0, config_.source_timeout * 4.0);
  if (std::fabs(dt) < mismatch) {return;}

  if (sim && count_publishers("/clock") == 0) {
    // Sim time requested but nothing drives /clock, so the clock is frozen near zero.
    RCLCPP_WARN(
      get_logger(),
      "polka: use_sim_time=true but no publisher on /clock was found — the simulated "
      "clock is not advancing, so timestamp/staleness checks are unreliable. If you are "
      "replaying a rosbag, play it with the --clock flag: ros2 bag play <bag> --clock");
    clock_diagnosed_ = true;
  } else if (!sim && dt > mismatch) {
    // Wall clock vs historical bag stamps: every source will be dropped as stale.
    RCLCPP_WARN(
      get_logger(),
      "polka: sensor timestamps are %.1f s behind the system clock. This looks like "
      "rosbag replay without simulated time; all sources will be dropped as stale and "
      "no merged cloud will be published. Set use_sim_time:=true and replay with the "
      "--clock flag: ros2 bag play <bag> --clock", dt);
    clock_diagnosed_ = true;
  }
  // Otherwise (e.g. sim time enabled and /clock present but not yet synced at startup)
  // leave clock_diagnosed_ unset and re-evaluate on the next tick — no false alarm.
}

void PolkaNode::output_callback()
{
  auto now = this->now();
  diagnose_clock_health(now);

  struct SourceData
  {
    CloudT::ConstPtr cloud;
    Eigen::Isometry3d transform;
    FilterParams filter_params;
    rclcpp::Time stamp;
  };
  std::vector<SourceData> source_data;

  for (auto & slot : sources_) {
    if (!slot.adapter) {continue;}  // pending source
    auto & src = *slot.adapter;
    auto cloud = src.get_latest();
    if (!cloud || cloud->empty()) {continue;}

    // A source past source_timeout is reused (its last-good cloud from
    // get_latest() above, and last-good TF via slot.last_good_transform below)
    // rather than dropped, until it's also past source_stale_reuse_window - a
    // single momentarily-late tick on one source shouldn't visibly shrink the
    // merged cloud. Only a source that's genuinely gone gets excluded.
    if (src.is_stale(config_.source_stale_reuse_window, now)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleNormalMs,
        "polka: source '%s' stale beyond reuse window, dropping", src.name().c_str());
      continue;
    }
    if (src.is_stale(config_.source_timeout, now)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleNormalMs,
        "polka: source '%s' stale, reusing last-good cloud/TF", src.name().c_str());
    }

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    try {
      auto tf_msg = tf_buffer_->lookupTransform(
        config_.output_frame_id, src.frame_id(), tf2::TimePointZero);
      transform = tf2::transformToEigen(tf_msg.transform);
      slot.last_good_transform = transform;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleNormalMs,
        "polka: TF failed for '%s': %s — using last known good transform",
        src.name().c_str(), ex.what());
      transform = slot.last_good_transform;
    }

    source_data.push_back({cloud, transform, src.filter_params(), src.last_stamp()});
  }

  if (source_data.empty()) {
    // No source produced a new frame this tick. When suppressing duplicates, emit
    // nothing rather than re-publishing the last cloud with its original stamp: a
    // duplicate header.stamp hands downstream SLAM (e.g. GLIM) two scans with zero
    // IMU samples between them and stalls odometry. A genuine gap is safe to skip.
    if (config_.suppress_duplicate_timestamps) {return;}
    std::lock_guard<std::mutex> lock(last_data_mutex_);
    if (last_cloud_ && !last_cloud_->empty()) {
      if (cloud_pub_) {
        auto msg = to_cloud_msg(*last_cloud_);
        msg.header.frame_id = config_.output_frame_id;
        msg.header.stamp = last_cloud_stamp_;
        cloud_out_counters_.record(
          msg.data.size() + 72, last_cloud_->size(), last_cloud_->size());
        publish_cloud(cloud_pub_, msg);
        mark_published();
      }
      if (scan_pub_) {
        auto scan = last_scan_ranges_.empty() ?
          scan_builder_.from_cloud(last_cloud_, last_cloud_stamp_) :
          scan_builder_.from_ranges(last_scan_ranges_, last_cloud_stamp_);
        scan_out_counters_.record(
          4 * (scan.ranges.size() + scan.intensities.size()) + 60,
          scan.ranges.size(), scan.ranges.size());
        scan_pub_->publish(scan);
        mark_published();
      }
    }
    return;
  }

  std::vector<rclcpp::Time> stamps;
  stamps.reserve(source_data.size());
  for (const auto & sd : source_data) {
    stamps.push_back(sd.stamp);
  }

  if (stamps.size() > 1) {
    auto [mn, mx] = std::minmax_element(stamps.begin(), stamps.end());
    double spread = (*mx - *mn).seconds();
    if (spread > config_.max_source_spread_warn) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleNormalMs,
        "polka: sync gap %6.3f s > %6.3f s", spread, config_.max_source_spread_warn);
    }
  }

  auto output_stamp = compute_output_stamp(stamps);

  // Drop duplicate-timestamp output: a source can still be within source_timeout
  // ("fresh") yet not have advanced since the last tick, so the chosen stamp
  // repeats. Emitting it would give downstream SLAM two scans with an identical
  // header.stamp and no IMU between them. Skip until the stamp actually moves.
  if (config_.suppress_duplicate_timestamps) {
    std::lock_guard<std::mutex> lock(last_data_mutex_);
    if (last_cloud_ && output_stamp == last_cloud_stamp_) {return;}
  }

  bool do_compensate = false;
  AveragedImu imu_for_alignment;
  if (config_.motion_compensation.enabled && global_imu_) {
    auto imu = global_imu_->snapshot();
    if (imu && imu->valid) {
      imu_for_alignment = *imu;
      do_compensate = true;
    }
  }

  std::vector<MergeInput> inputs;
  inputs.reserve(source_data.size());
  int64_t points_in = 0;
  for (auto & sd : source_data) {
    Eigen::Isometry3d final_transform = sd.transform;
    if (do_compensate) {
      double dt = (sd.stamp - output_stamp).seconds();
      if (std::abs(dt) > 1e-6) {
        Eigen::Isometry3d delta = compute_motion_delta(
          imu_for_alignment.angular_vel, imu_for_alignment.linear_accel, dt);
        final_transform = delta * sd.transform;
      }
    }
    points_in += static_cast<int64_t>(sd.cloud->size());
    inputs.push_back({sd.cloud, final_transform, sd.filter_params});
  }
  last_points_in_ = points_in;

  if (merge_engine_->is_gpu()) {
    auto pcfg = output_pipeline_.to_pipeline_config(
      scan_pub_ != nullptr, config_.scan_output.flatten);
    auto result = merge_engine_->merge_pipeline(inputs, pcfg);
    if (!result.cloud || result.cloud->empty()) {return;}

    last_points_out_ = static_cast<int64_t>(result.cloud->size());
    if (cloud_pub_) {
      if (config_.point_timestamps.enabled &&
        config_.point_timestamps.mode == PerPointTimeMode::OFFSET)
      {
        rebase_point_time(*result.cloud, output_stamp);
      }
      auto msg = to_cloud_msg(*result.cloud);
      msg.header.frame_id = config_.output_frame_id;
      msg.header.stamp = output_stamp;
      cloud_out_counters_.record(
        msg.data.size() + 72, result.cloud->size(), result.cloud->size());
      publish_cloud(cloud_pub_, msg);
      mark_published();
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_cloud_ = result.cloud;
      last_cloud_stamp_ = output_stamp;
    }
    if (scan_pub_) {
      auto scan = result.scan_ranges.empty() ?
        scan_builder_.from_cloud(result.cloud, output_stamp) :
        scan_builder_.from_ranges(result.scan_ranges, output_stamp);
      scan_out_counters_.record(
        4 * (scan.ranges.size() + scan.intensities.size()) + 60,
        scan.ranges.size(), scan.ranges.size());
      scan_pub_->publish(scan);
      mark_published();
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_scan_ranges_ = result.scan_ranges;
    }
  } else {
    auto merged = merge_engine_->merge(inputs);
    if (!merged || merged->empty()) {return;}

    if (config_.point_timestamps.enabled && config_.cloud_output.voxel.enabled) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "polka: CPU voxel downsampling reduces per-point 'time' precision to float; "
        "use enable_gpu for exact per-point time");
    }

    output_pipeline_.process(merged, config_.output_frame_id);
    last_points_out_ = static_cast<int64_t>(merged->size());

    if (cloud_pub_) {
      if (config_.point_timestamps.enabled &&
        config_.point_timestamps.mode == PerPointTimeMode::OFFSET)
      {
        rebase_point_time(*merged, output_stamp);
      }
      auto msg = to_cloud_msg(*merged);
      msg.header.frame_id = config_.output_frame_id;
      msg.header.stamp = output_stamp;
      cloud_out_counters_.record(msg.data.size() + 72, merged->size(), merged->size());
      publish_cloud(cloud_pub_, msg);
      mark_published();
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_cloud_ = merged;
      last_cloud_stamp_ = output_stamp;
    }
    if (scan_pub_) {
      auto scan = scan_builder_.from_cloud(merged, output_stamp);
      scan_out_counters_.record(
        4 * (scan.ranges.size() + scan.intensities.size()) + 60,
        scan.ranges.size(), scan.ranges.size());
      scan_pub_->publish(scan);
      mark_published();
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_scan_ranges_.clear();
    }
  }
}

rclcpp::Time PolkaNode::compute_output_stamp(const std::vector<rclcpp::Time> & stamps)
{
  if (stamps.empty()) {return this->now();}
  switch (config_.timestamp_strategy) {
    case TimestampStrategy::EARLIEST:
      return *std::min_element(stamps.begin(), stamps.end());
    case TimestampStrategy::LATEST:
      return *std::max_element(stamps.begin(), stamps.end());
    case TimestampStrategy::LOCAL:
      return this->now();
    case TimestampStrategy::AVERAGE: {
        double sum = 0.0;
        for (const auto & s : stamps) {
          sum += s.seconds();
        }
        return rclcpp::Time(static_cast<int64_t>((sum / stamps.size()) * 1e9));
      }
    default:
      return this->now();
  }
}

void PolkaNode::rebase_point_time(CloudT & cloud, const rclcpp::Time & stamp)
{
  // Points carry absolute Unix seconds internally; downstream deskewing expects
  // a per-point offset relative to the cloud header. Subtracting in double keeps
  // ~sub-microsecond precision even though the absolute values are ~1.7e9.
  const double base = stamp.seconds();
  for (auto & p : cloud) {
    p.time -= base;
  }
}

sensor_msgs::msg::PointCloud2 PolkaNode::to_cloud_msg(const CloudT & cloud) const
{
  sensor_msgs::msg::PointCloud2 msg;
  if (config_.point_timestamps.enabled) {
    pcl::toROSMsg(cloud, msg);  // includes the float64 'time' field
  } else {
    // Legacy output: drop 'time', emit x/y/z/intensity exactly as before.
    pcl::PointCloud<pcl::PointXYZI> xyzi;
    pcl::copyPointCloud(cloud, xyzi);
    pcl::toROSMsg(xyzi, msg);
  }
  return msg;
}

void PolkaNode::log_startup_banner() const
{
  std::ostringstream os;
  os << '\n';
  os << "polka: ──────────── started ────────────\n";
  os << "polka:   engine        : " << (merge_engine_->is_gpu() ? "CUDA (full pipeline)" : "CPU")
     << '\n';

  char rate_buf[32];
  std::snprintf(rate_buf, sizeof(rate_buf), "%6.1f Hz", config_.output_rate);
  os << "polka:   output rate   : " << rate_buf << '\n';
  os << "polka:   output frame  : '" << config_.output_frame_id << "'\n";

  const auto & mc = config_.motion_compensation;
  if (mc.enabled && !mc.imu_topic.empty()) {
    os << "polka:   imu_topic     : '" << mc.imu_topic
       << "' (buffer=" << mc.imu_buffer_size << ")\n";
  } else if (mc.enabled) {
    os << "polka:   imu_topic     : (motion comp enabled, no global topic)\n";
  } else {
    os << "polka:   imu_topic     : (motion comp disabled)\n";
  }

  if (config_.diagnostics.enabled) {
    char diag_buf[48];
    std::snprintf(
      diag_buf, sizeof(diag_buf), "on (/diagnostics @ %.1f s)",
      config_.diagnostics.publish_period_sec);
    os << "polka:   diagnostics   : " << diag_buf << '\n';
  } else {
    os << "polka:   diagnostics   : off\n";
  }

  os << "polka:   sources (" << config_.sources.size() << "):\n";
  for (const auto & sc : config_.sources) {
    const char * type_str =
      (sc.type == SourceType::POINTCLOUD2) ? "pointcloud2" : "laserscan  ";
    const auto & fp = sc.filter_params;
    int nfilters = (fp.range_filter_enabled ? 1 : 0) +
      (fp.angular_filter_enabled ? 1 : 0) +
      (fp.box_filter_enabled ? 1 : 0);
    os << "polka:     " << sc.name
       << "  " << type_str
       << "  '" << sc.topic << "'"
       << "  filters=" << nfilters
       << "  deskew=" << ((mc.enabled && mc.per_point_deskew) ? "on" : "off")
       << '\n';
  }
  os << "polka: ──────────────────────────────────";

  RCLCPP_INFO(get_logger(), "%s", os.str().c_str());
}

}  // namespace polka

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(polka::PolkaNode)
