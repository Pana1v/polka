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
#include "polka/merge_engine/cpu_merge_engine.hpp"
#include "polka/filters/range_filter.hpp"
#include "polka/filters/angular_filter.hpp"
#include "polka/filters/box_filter.hpp"
#include "polka/se3_exp.hpp"

#ifdef POLKA_CUDA_ENABLED
#include "polka/merge_engine/cuda_merge_engine.hpp"
#include <cuda_runtime.h>
#endif

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

namespace polka {

namespace {

rclcpp::QoS build_qos(const OutputQosConfig & cfg)
{
  rclcpp::QoS qos(cfg.history_depth);

  if (cfg.reliability == "best_effort")
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  else
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

  if (cfg.durability == "transient_local")
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  else
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

  if (cfg.liveliness == "manual_by_topic")
    qos.liveliness(rclcpp::LivelinessPolicy::ManualByTopic);
  else if (cfg.liveliness == "manual_by_node")
    qos.liveliness(rclcpp::LivelinessPolicy::ManualByNode);
  else
    qos.liveliness(rclcpp::LivelinessPolicy::Automatic);

  if (cfg.liveliness_lease_duration_ms > 0.0)
    qos.liveliness_lease_duration(
      std::chrono::milliseconds(static_cast<int64_t>(cfg.liveliness_lease_duration_ms)));

  if (cfg.deadline_ms > 0.0)
    qos.deadline(
      std::chrono::milliseconds(static_cast<int64_t>(cfg.deadline_ms)));

  if (cfg.lifespan_ms > 0.0)
    qos.lifespan(
      std::chrono::milliseconds(static_cast<int64_t>(cfg.lifespan_ms)));

  return qos;
}

}  // namespace

PolkaNode::PolkaNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("polka", options), config_loader_(this)
{
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
      RCLCPP_WARN(get_logger(), "enable_gpu=true but no CUDA device found, falling back to CPU");
    }
  }
#endif
  if (!merge_engine_)
    merge_engine_ = std::make_unique<CpuMergeEngine>();
  RCLCPP_INFO(get_logger(), "using %s merge engine%s",
    merge_engine_->is_gpu() ? "GPU (full pipeline)" : "CPU",
    merge_engine_->is_gpu() ? "" : " (set enable_gpu:=true for GPU acceleration)");

  if (config_.motion_compensation.enabled && !config_.motion_compensation.imu_topic.empty())
    global_imu_ = std::make_shared<ImuBuffer>(
      this, config_.motion_compensation.imu_topic,
      config_.motion_compensation.imu_buffer_size);
  else if (config_.motion_compensation.enabled)
    RCLCPP_WARN(get_logger(),
      "motion compensation enabled but imu_topic is empty, deskewing will not activate");

  // Global IMU getter for source adapters that don't have a per-source IMU
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
  tf_fail_counts_.resize(sources_.size(), 0);

  build_output_filters();

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
    RCLCPP_INFO(get_logger(), "output timer at %.1f Hz", config_.output_rate);
  }

  for (const auto & sc : config_.sources)
    source_names_.push_back(sc.name);

  param_cb_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = reconfigure();
      return result;
    });
}

void PolkaNode::build_output_filters()
{
  const auto & fp = config_.cloud_output.filters;
  if (fp.range_filter_enabled)
    output_filters_.push_back(std::make_unique<RangeFilter>(fp.min_range, fp.max_range));
  if (fp.angular_filter_enabled && !fp.angular_ranges.empty())
    output_filters_.push_back(std::make_unique<AngularFilter>(fp.angular_ranges, fp.angular_invert));
  if (fp.box_filter_enabled)
    output_filters_.push_back(std::make_unique<BoxFilter>(fp.box_min, fp.box_max));

  // Self-filter: inverted box filters to exclude robot body
  const auto & sf = config_.cloud_output.self_filter;
  if (sf.enabled) {
    for (const auto & eb : sf.boxes)
      output_filters_.push_back(std::make_unique<BoxFilter>(eb.min, eb.max, true));
  }
}

void PolkaNode::output_callback()
{
  auto now = this->now();

  // Pass 1: Collect clouds, TF transforms, and stamps
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
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "source '%s' is stale", src->name().c_str());
      continue;
    }

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    try {
      auto tf_msg = tf_buffer_->lookupTransform(
        config_.output_frame_id, src->frame_id(), tf2::TimePointZero);
      transform = tf2::transformToEigen(tf_msg.transform);
      last_good_transforms_[i] = transform;
      tf_fail_counts_[i] = 0;
    } catch (const tf2::TransformException & ex) {
      tf_fail_counts_[i]++;
      if (tf_fail_counts_[i] <= 5)
        RCLCPP_WARN(get_logger(), "TF failed for '%s': %s", src->name().c_str(), ex.what());
      else if (tf_fail_counts_[i] == 6)
        RCLCPP_ERROR(get_logger(), "TF persistently failing for '%s', suppressing", src->name().c_str());
      transform = last_good_transforms_[i];
    }

    source_data.push_back({cloud, transform, src->filter_params(), src->last_stamp()});
  }

  bool has_fresh_data = !source_data.empty();
  if (!has_fresh_data) {
    std::lock_guard<std::mutex> lock(last_data_mutex_);
    if (last_cloud_ && !last_cloud_->empty()) {
      if (cloud_pub_) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*last_cloud_, msg);
        msg.header.frame_id = config_.output_frame_id;
        msg.header.stamp = last_cloud_stamp_;
        cloud_pub_->publish(msg);
      }
      if (scan_pub_) {
        if (!last_scan_ranges_.empty())
          publish_scan_from_ranges(last_scan_ranges_, last_cloud_stamp_);
        else
          publish_scan(last_cloud_, last_cloud_stamp_);
      }
      return;
    } else {
      return;
    }
  }

  std::vector<rclcpp::Time> stamps;
  stamps.reserve(source_data.size());
  for (const auto & sd : source_data)
    stamps.push_back(sd.stamp);

  if (stamps.size() > 1) {
    auto [mn, mx] = std::minmax_element(stamps.begin(), stamps.end());
    double spread = (*mx - *mn).seconds();
    if (spread > config_.max_source_spread_warn)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "source spread %.3f s > %.3f s", spread, config_.max_source_spread_warn);
  }

  auto output_stamp = compute_output_stamp(stamps);

  // Pass 2: Apply IMU-based inter-source compensation and build MergeInputs
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
    auto pcfg = build_pipeline_config();
    auto result = merge_engine_->merge_pipeline(inputs, pcfg);
    if (!result.cloud || result.cloud->empty()) return;

    if (cloud_pub_) {
      publish_cloud(result.cloud, output_stamp);
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_cloud_ = result.cloud;
      last_cloud_stamp_ = output_stamp;
    }
    if (scan_pub_ && !result.scan_ranges.empty()) {
      publish_scan_from_ranges(result.scan_ranges, output_stamp);
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_scan_ranges_ = result.scan_ranges;
    } else if (scan_pub_) {
      publish_scan(result.cloud, output_stamp);
    }
  } else {
    auto merged = merge_engine_->merge(inputs);
    if (!merged || merged->empty()) return;

    for (auto & filter : output_filters_)
      filter->apply(*merged, config_.output_frame_id);

    if (config_.cloud_output.height_cap.enabled)
      height_cap(*merged);

    if (config_.cloud_output.voxel.enabled)
      voxel_downsample(*merged);

    if (cloud_pub_) {
      publish_cloud(merged, output_stamp);
      std::lock_guard<std::mutex> lock(last_data_mutex_);
      last_cloud_ = merged;
      last_cloud_stamp_ = output_stamp;
    }
    if (scan_pub_) {
      publish_scan(merged, output_stamp);
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

void PolkaNode::publish_cloud(CloudT::ConstPtr cloud, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = config_.output_frame_id;
  msg.header.stamp = stamp;
  cloud_pub_->publish(msg);
}

void PolkaNode::publish_scan(CloudT::ConstPtr cloud, const rclcpp::Time & stamp)
{
  const auto & fp = config_.scan_output.flatten;
  const float z_min = fp.z_min, z_max = fp.z_max;
  const float a_min = fp.angle_min, a_max = fp.angle_max;
  const float a_inc = fp.angle_increment;
  const float r_min = fp.range_min, r_max = fp.range_max;
  const int n = fp.n_bins;

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = config_.output_frame_id;
  scan.header.stamp = stamp;
  scan.angle_min = a_min;
  scan.angle_max = a_max;
  scan.angle_increment = a_inc;
  scan.range_min = r_min;
  scan.range_max = r_max;
  scan.time_increment = 0.0f;
  scan.scan_time = (config_.output_rate > 0.0)
    ? 1.0f / static_cast<float>(config_.output_rate) : 0.0f;
  scan.ranges.assign(n, std::numeric_limits<float>::infinity());

  for (const auto & p : *cloud) {
    if (p.z < z_min || p.z > z_max) continue;
    float az = std::atan2(p.y, p.x);
    if (az < a_min || az > a_max) continue;
    int bin = static_cast<int>((az - a_min) / a_inc);
    if (bin < 0 || bin >= n) continue;
    float range = std::sqrt(p.x * p.x + p.y * p.y);
    if (range < r_min || range > r_max) continue;
    scan.ranges[bin] = std::min(scan.ranges[bin], range);
  }

  scan_pub_->publish(scan);
}

PipelineConfig PolkaNode::build_pipeline_config() const
{
  PipelineConfig pcfg;
  pcfg.output_filters = config_.cloud_output.filters;
  pcfg.self_filter_enabled = config_.cloud_output.self_filter.enabled;
  pcfg.self_filter_boxes = config_.cloud_output.self_filter.boxes;
  pcfg.height_cap = config_.cloud_output.height_cap;
  pcfg.voxel = config_.cloud_output.voxel;
  pcfg.scan_enabled = (scan_pub_ != nullptr);
  if (pcfg.scan_enabled)
    pcfg.flatten = config_.scan_output.flatten;
  return pcfg;
}

void PolkaNode::publish_scan_from_ranges(
  const std::vector<float> & ranges, const rclcpp::Time & stamp)
{
  const auto & fp = config_.scan_output.flatten;

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = config_.output_frame_id;
  scan.header.stamp = stamp;
  scan.angle_min = static_cast<float>(fp.angle_min);
  scan.angle_max = static_cast<float>(fp.angle_max);
  scan.angle_increment = static_cast<float>(fp.angle_increment);
  scan.range_min = static_cast<float>(fp.range_min);
  scan.range_max = static_cast<float>(fp.range_max);
  scan.time_increment = 0.0f;
  scan.scan_time = (config_.output_rate > 0.0)
    ? 1.0f / static_cast<float>(config_.output_rate) : 0.0f;
  scan.ranges = ranges;

  scan_pub_->publish(scan);
}

void PolkaNode::voxel_downsample(CloudT & cloud)
{
  const auto & vc = config_.cloud_output.voxel;
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud.makeShared());
  vg.setLeafSize(vc.leaf_x, vc.leaf_y, vc.leaf_z);
  CloudT filtered;
  vg.filter(filtered);
  cloud = std::move(filtered);
}

void PolkaNode::height_cap(CloudT & cloud)
{
  const auto & hc = config_.cloud_output.height_cap;
  const float z_min = static_cast<float>(hc.z_min);
  const float z_max = static_cast<float>(hc.z_max);
  size_t j = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (cloud[i].z >= z_min && cloud[i].z <= z_max)
      cloud[j++] = cloud[i];
  }
  cloud.resize(j);
  cloud.width = static_cast<uint32_t>(j);
  cloud.height = 1;
  cloud.is_dense = true;
}

bool PolkaNode::reconfigure()
{
  auto prev_rate = config_.output_rate;
  auto prev_cloud_enabled = config_.cloud_output.enabled;
  auto prev_scan_enabled = config_.scan_output.enabled;

  try {
    config_ = config_loader_.reload(source_names_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "reconfigure failed: %s", ex.what());
    return false;
  }

  // Rebuild output timer if rate changed
  if (config_.output_rate != prev_rate && config_.output_rate > 0.0) {
    output_timer_->cancel();
    auto period = std::chrono::duration<double>(1.0 / config_.output_rate);
    output_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PolkaNode::output_callback, this));
    RCLCPP_INFO(get_logger(), "output rate changed to %.1f Hz", config_.output_rate);
  }

  // Rebuild cloud publisher if toggled
  if (config_.cloud_output.enabled && !prev_cloud_enabled)
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.cloud_output.topic, build_qos(config_.cloud_output.qos));
  else if (!config_.cloud_output.enabled && prev_cloud_enabled)
    cloud_pub_.reset();

  // Rebuild scan publisher if toggled
  if (config_.scan_output.enabled && !prev_scan_enabled)
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      config_.scan_output.topic, build_qos(config_.scan_output.qos));
  else if (!config_.scan_output.enabled && prev_scan_enabled)
    scan_pub_.reset();

  // Toggle global IMU buffer
  bool imu_was_enabled = (global_imu_ != nullptr);
  bool imu_now_enabled = config_.motion_compensation.enabled
                         && !config_.motion_compensation.imu_topic.empty();
  if (imu_now_enabled && !imu_was_enabled) {
    global_imu_ = std::make_shared<ImuBuffer>(
      this, config_.motion_compensation.imu_topic,
      config_.motion_compensation.imu_buffer_size);
  } else if (!imu_now_enabled && imu_was_enabled) {
    global_imu_.reset();
    RCLCPP_INFO(get_logger(), "motion compensation disabled");
  }

  // Rebuild output filters
  output_filters_.clear();
  build_output_filters();

  // Rebuild per-source filters
  for (size_t i = 0; i < sources_.size() && i < config_.sources.size(); ++i)
    sources_[i]->rebuild_filters(config_.sources[i].filter_params);

  RCLCPP_INFO(get_logger(), "reconfigured");
  return true;
}

}  // namespace polka

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(polka::PolkaNode)
