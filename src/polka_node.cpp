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
#include "polka/util/log_format.hpp"
#include "polka/util/qos_builder.hpp"
#include "polka/util/se3_exp.hpp"
#include "polka/merge_engine/cpu_merge_engine.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <pcl/common/io.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>

#ifdef POLKA_CUDA_ENABLED
#include "polka/merge_engine/cuda_merge_engine.hpp"
#include <cuda_runtime.h>
#endif

namespace polka {

PolkaNode::PolkaNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("polka", options), config_loader_(this)
{
  // Our point type carries a 'time' field that source clouds usually lack (they
  // publish 'timestamp' or nothing), so pcl::fromROSMsg logs a per-message
  // "Failed to find match for field 'time'" warning. populate_point_time fills it
  // explicitly, so silence PCL's redundant warnings (errors still surface).
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

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
      RCLCPP_WARN(get_logger(),
        "polka: enable_gpu=true but no CUDA device found, falling back to CPU");
    }
  }
#endif
  if (!merge_engine_)
    merge_engine_ = std::make_unique<CpuMergeEngine>();

  if (config_.motion_compensation.enabled && !config_.motion_compensation.imu_topic.empty())
    global_imu_ = std::make_shared<ImuBuffer>(
      this, config_.motion_compensation.imu_topic,
      config_.motion_compensation.imu_buffer_size);
  else if (config_.motion_compensation.enabled)
    RCLCPP_WARN(get_logger(),
      "polka: motion compensation enabled but imu_topic is empty, deskewing will not activate");

  SourceAdapter::ImuGetter imu_getter = nullptr;
  if (config_.motion_compensation.enabled && config_.motion_compensation.per_point_deskew
      && global_imu_) {
    imu_getter = [this]() -> std::shared_ptr<const AveragedImu> {
      return global_imu_->snapshot();
    };
  }

  bool gpu_filters = merge_engine_->is_gpu();
  bool deskew = config_.motion_compensation.enabled && config_.motion_compensation.per_point_deskew;
  for (const auto & src_cfg : config_.sources)
    sources_.push_back(std::make_unique<SourceAdapter>(
      this, src_cfg, gpu_filters, imu_getter, deskew,
      config_.motion_compensation.deskew_timestamp_field,
      tf_buffer_, config_.motion_compensation.imu_buffer_size));

  last_good_transforms_.resize(sources_.size(), Eigen::Isometry3d::Identity());

  output_pipeline_.configure(config_.cloud_output);
  scan_builder_.configure(config_.scan_output, config_.output_rate, config_.output_frame_id);

  if (config_.cloud_output.enabled)
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.cloud_output.topic, build_qos(config_.cloud_output.qos));
  if (config_.scan_output.enabled)
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      config_.scan_output.topic, build_qos(config_.scan_output.qos));

  if (config_.output_rate > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / config_.output_rate);
    output_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PolkaNode::output_callback, this));
  }

  for (const auto & sc : config_.sources)
    source_names_.push_back(sc.name);

  log_startup_banner();

  param_cb_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = reconfigure();
      return result;
    });
}

void PolkaNode::diagnose_clock_health(const rclcpp::Time & now)
{
  if (clock_diagnosed_) return;

  // Newest sensor stamp across sources that have actually received data. Without any
  // data there is nothing to compare the clock against, so we can't judge yet.
  rclcpp::Time newest;
  bool any = false;
  for (const auto & src : sources_) {
    if (!src->received()) continue;
    auto stamp = src->last_stamp();
    if (!any || stamp > newest) { newest = stamp; any = true; }
  }
  if (!any) return;

  const double dt = (now - newest).seconds();
  const bool sim = this->get_parameter("use_sim_time").as_bool();

  // Allow normal jitter (and the correct --clock case): only react when the clock and
  // the data disagree by several staleness windows.
  const double mismatch = std::max(2.0, config_.source_timeout * 4.0);
  if (std::fabs(dt) < mismatch) return;

  if (sim && count_publishers("/clock") == 0) {
    // Sim time requested but nothing drives /clock, so the clock is frozen near zero.
    RCLCPP_WARN(get_logger(),
      "polka: use_sim_time=true but no publisher on /clock was found — the simulated "
      "clock is not advancing, so timestamp/staleness checks are unreliable. If you are "
      "replaying a rosbag, play it with the --clock flag: ros2 bag play <bag> --clock");
    clock_diagnosed_ = true;
  } else if (!sim && dt > mismatch) {
    // Wall clock vs historical bag stamps: every source will be dropped as stale.
    RCLCPP_WARN(get_logger(),
      "polka: sensor timestamps are %.1f s behind the system clock. This looks like "
      "rosbag replay without simulated time; all sources will be dropped as stale and "
      "no merged cloud will be published. Set use_sim_time:=true and replay with the "
      "--clock flag: ros2 bag play <bag> --clock", dt);
    clock_diagnosed_ = true;
  }
  // Otherwise (e.g. sim time enabled and /clock present but not yet synced at startup)
  // leave clock_diagnosed_ unset and re-evaluate on the next tick — no false alarm.
}

namespace {
constexpr uint64_t kMergePerfLogInterval = 50;
}

void PolkaNode::log_merge_perf(double us, const char * engine_label)
{
  merge_total_us_ += us;
  merge_max_us_ = std::max(merge_max_us_, us);
  if (++merge_calls_ % kMergePerfLogInterval == 0) {
    RCLCPP_INFO(get_logger(),
      "polka: perf merge [%s]: mean=%.3fms max=%.3fms (over %llu calls)",
      engine_label, merge_total_us_ / kMergePerfLogInterval / 1000.0,
      merge_max_us_ / 1000.0, static_cast<unsigned long long>(kMergePerfLogInterval));
    merge_total_us_ = 0.0;
    merge_max_us_ = 0.0;
  }
}

void PolkaNode::output_callback()
{
  auto now = this->now();
  diagnose_clock_health(now);

  struct SourceData {
    CloudT::ConstPtr cloud;
    Eigen::Isometry3d transform;
    FilterParams filter_params;
    rclcpp::Time stamp;
  };
  std::vector<SourceData> source_data;

  for (size_t i = 0; i < sources_.size(); ++i) {
    auto & src = sources_[i];
    auto cloud = src->get_latest();
    if (!cloud || cloud->empty()) continue;

    if (src->is_stale(config_.source_timeout, now)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleNormalMs,
        "polka: source '%s' is stale", src->name().c_str());
      continue;
    }

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    try {
      auto tf_msg = tf_buffer_->lookupTransform(
        config_.output_frame_id, src->frame_id(), tf2::TimePointZero);
      transform = tf2::transformToEigen(tf_msg.transform);
      last_good_transforms_[i] = transform;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleNormalMs,
        "polka: TF failed for '%s': %s — using last known good transform",
        src->name().c_str(), ex.what());
      transform = last_good_transforms_[i];
    }

    source_data.push_back({cloud, transform, src->filter_params(), src->last_stamp()});
  }

  if (source_data.empty()) {
    // No source produced a new frame this tick. When suppressing duplicates, emit
    // nothing rather than re-publishing the last cloud with its original stamp: a
    // duplicate header.stamp hands downstream SLAM (e.g. GLIM) two scans with zero
    // IMU samples between them and stalls odometry. A genuine gap is safe to skip.
    if (config_.suppress_duplicate_timestamps) return;
    std::lock_guard<std::mutex> lock(last_data_mutex_);
    if (last_cloud_ && !last_cloud_->empty()) {
      if (cloud_pub_) {
        auto msg = to_cloud_msg(*last_cloud_);
        msg.header.frame_id = config_.output_frame_id;
        msg.header.stamp = last_cloud_stamp_;
        cloud_pub_->publish(msg);
      }
      if (scan_pub_) {
        auto scan = last_scan_ranges_.empty()
          ? scan_builder_.from_cloud(last_cloud_, last_cloud_stamp_)
          : scan_builder_.from_ranges(last_scan_ranges_, last_cloud_stamp_);
        scan_pub_->publish(scan);
      }
    }
    return;
  }

  std::vector<rclcpp::Time> stamps;
  stamps.reserve(source_data.size());
  for (const auto & sd : source_data)
    stamps.push_back(sd.stamp);

  if (stamps.size() > 1) {
    auto [mn, mx] = std::minmax_element(stamps.begin(), stamps.end());
    double spread = (*mx - *mn).seconds();
    if (spread > config_.max_source_spread_warn)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleNormalMs,
        "polka: source spread %6.3f s > %6.3f s", spread, config_.max_source_spread_warn);
  }

  auto output_stamp = compute_output_stamp(stamps);

  // Drop duplicate-timestamp output: a source can still be within source_timeout
  // ("fresh") yet not have advanced since the last tick, so the chosen stamp
  // repeats. Emitting it would give downstream SLAM two scans with an identical
  // header.stamp and no IMU between them. Skip until the stamp actually moves.
  if (config_.suppress_duplicate_timestamps) {
    std::lock_guard<std::mutex> lock(last_data_mutex_);
    if (last_cloud_ && output_stamp == last_cloud_stamp_) return;
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
    inputs.push_back({sd.cloud, final_transform, sd.filter_params});
  }

  if (merge_engine_->is_gpu()) {
    auto pcfg = output_pipeline_.to_pipeline_config(
      scan_pub_ != nullptr, config_.scan_output.flatten);
    auto merge_t0 = std::chrono::steady_clock::now();
    auto result = merge_engine_->merge_pipeline(inputs, pcfg);
    log_merge_perf(std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - merge_t0).count(), "CUDA");
    if (!result.cloud || result.cloud->empty()) return;

    if (cloud_pub_) {
      if (config_.point_timestamps.enabled &&
          config_.point_timestamps.mode == PerPointTimeMode::OFFSET)
        rebase_point_time(*result.cloud, output_stamp);
      auto msg = to_cloud_msg(*result.cloud);
      msg.header.frame_id = config_.output_frame_id;
      msg.header.stamp = output_stamp;
      cloud_pub_->publish(msg);
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_cloud_ = result.cloud;
      last_cloud_stamp_ = output_stamp;
    }
    if (scan_pub_) {
      auto scan = result.scan_ranges.empty()
        ? scan_builder_.from_cloud(result.cloud, output_stamp)
        : scan_builder_.from_ranges(result.scan_ranges, output_stamp);
      scan_pub_->publish(scan);
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_scan_ranges_ = result.scan_ranges;
    }
  } else {
    auto merge_t0 = std::chrono::steady_clock::now();
    auto merged = merge_engine_->merge(inputs);
    log_merge_perf(std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - merge_t0).count(), "CPU");
    if (!merged || merged->empty()) return;

    if (config_.point_timestamps.enabled && config_.cloud_output.voxel.enabled)
      RCLCPP_WARN_ONCE(get_logger(),
        "polka: CPU voxel downsampling reduces per-point 'time' precision to float; "
        "use enable_gpu for exact per-point time");

    output_pipeline_.process(*merged, config_.output_frame_id);

    if (cloud_pub_) {
      if (config_.point_timestamps.enabled &&
          config_.point_timestamps.mode == PerPointTimeMode::OFFSET)
        rebase_point_time(*merged, output_stamp);
      auto msg = to_cloud_msg(*merged);
      msg.header.frame_id = config_.output_frame_id;
      msg.header.stamp = output_stamp;
      cloud_pub_->publish(msg);
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_cloud_ = merged;
      last_cloud_stamp_ = output_stamp;
    }
    if (scan_pub_) {
      auto scan = scan_builder_.from_cloud(merged, output_stamp);
      scan_pub_->publish(scan);
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_scan_ranges_.clear();
    }
  }
}

rclcpp::Time PolkaNode::compute_output_stamp(const std::vector<rclcpp::Time> & stamps)
{
  if (stamps.empty()) return this->now();
  switch (config_.timestamp_strategy) {
    case TimestampStrategy::EARLIEST:
      return *std::min_element(stamps.begin(), stamps.end());
    case TimestampStrategy::LATEST:
      return *std::max_element(stamps.begin(), stamps.end());
    case TimestampStrategy::LOCAL:
      return this->now();
    case TimestampStrategy::AVERAGE: {
      double sum = 0.0;
      for (const auto & s : stamps) sum += s.seconds();
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
  for (auto & p : cloud)
    p.time -= base;
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

bool PolkaNode::reconfigure()
{
  auto prev_rate = config_.output_rate;
  auto prev_cloud_enabled = config_.cloud_output.enabled;
  auto prev_scan_enabled = config_.scan_output.enabled;

  try {
    config_ = config_loader_.reload(source_names_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "polka: reconfigure failed: %s", ex.what());
    return false;
  }

  std::vector<std::string> changes;

  if (config_.output_rate != prev_rate && config_.output_rate > 0.0) {
    output_timer_->cancel();
    auto period = std::chrono::duration<double>(1.0 / config_.output_rate);
    output_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PolkaNode::output_callback, this));
    char buf[64];
    std::snprintf(buf, sizeof(buf), "output_rate=%.1fHz", config_.output_rate);
    changes.emplace_back(buf);
  }

  if (config_.cloud_output.enabled && !prev_cloud_enabled)
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.cloud_output.topic, build_qos(config_.cloud_output.qos));
  else if (!config_.cloud_output.enabled && prev_cloud_enabled)
    cloud_pub_.reset();

  if (config_.scan_output.enabled && !prev_scan_enabled)
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      config_.scan_output.topic, build_qos(config_.scan_output.qos));
  else if (!config_.scan_output.enabled && prev_scan_enabled)
    scan_pub_.reset();

  bool imu_was_enabled = (global_imu_ != nullptr);
  bool imu_now_enabled = config_.motion_compensation.enabled
                         && !config_.motion_compensation.imu_topic.empty();
  if (imu_now_enabled && !imu_was_enabled) {
    global_imu_ = std::make_shared<ImuBuffer>(
      this, config_.motion_compensation.imu_topic,
      config_.motion_compensation.imu_buffer_size);
    changes.emplace_back("motion_compensation=on");
  } else if (!imu_now_enabled && imu_was_enabled) {
    global_imu_.reset();
    changes.emplace_back("motion_compensation=off");
  }

  if (config_.cloud_output.enabled != prev_cloud_enabled)
    changes.emplace_back(
      std::string("cloud_output=") + (config_.cloud_output.enabled ? "on" : "off"));
  if (config_.scan_output.enabled != prev_scan_enabled)
    changes.emplace_back(
      std::string("scan_output=") + (config_.scan_output.enabled ? "on" : "off"));

  output_pipeline_.configure(config_.cloud_output);
  scan_builder_.configure(config_.scan_output, config_.output_rate, config_.output_frame_id);

  for (size_t i = 0; i < sources_.size() && i < config_.sources.size(); ++i)
    sources_[i]->rebuild_filters(config_.sources[i].filter_params);

  if (changes.empty()) {
    RCLCPP_INFO(get_logger(), "polka: reconfigured (filters only)");
  } else {
    std::string joined;
    for (size_t i = 0; i < changes.size(); ++i) {
      if (i) joined += ", ";
      joined += changes[i];
    }
    RCLCPP_INFO(get_logger(), "polka: reconfigured — %s", joined.c_str());
  }
  return true;
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
