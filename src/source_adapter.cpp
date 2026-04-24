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

#include "polka/source_adapter.hpp"
#include "polka/se3_exp.hpp"
#include "polka/filters/range_filter.hpp"
#include "polka/filters/angular_filter.hpp"
#include "polka/filters/box_filter.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cstring>

namespace polka {

SourceAdapter::SourceAdapter(rclcpp::Node * node, const SourceConfig & config, bool gpu_filters,
                             ImuGetter imu_getter, bool deskew_enabled,
                             const std::string & timestamp_field_hint,
                             std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                             int imu_buffer_size)
: node_(node), config_(config), logger_(node->get_logger()), gpu_filters_(gpu_filters),
  get_imu_(std::move(imu_getter)), deskew_enabled_(deskew_enabled),
  timestamp_field_hint_(timestamp_field_hint), tf_buffer_(std::move(tf_buffer))
{
  // Per-source IMU: if configured, create a local buffer and override the getter
  if (deskew_enabled_ && !config.imu_topic.empty()) {
    local_imu_ = std::make_shared<ImuBuffer>(node, config.imu_topic, imu_buffer_size);
    get_imu_ = [this]() -> std::shared_ptr<const AveragedImu> {
      return local_imu_->snapshot();
    };
    RCLCPP_INFO(logger_, "polka: source '%s' using per-source IMU on '%s'",
      config.name.c_str(), config.imu_topic.c_str());
  }
  // Build QoS
  rclcpp::QoS qos(config.qos_history_depth);
  if (config.qos_reliability == "best_effort") {
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  } else {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  }

  // Create subscription based on source type
  if (config.type == SourceType::POINTCLOUD2) {
    pc2_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.topic, qos,
      std::bind(&SourceAdapter::pc2_callback, this, std::placeholders::_1));
  } else {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      config.topic, qos,
      std::bind(&SourceAdapter::scan_callback, this, std::placeholders::_1));
  }

  if (!gpu_filters_) {
    const auto & fp = config.filter_params;
    if (fp.range_filter_enabled)
      filters_.push_back(std::make_unique<RangeFilter>(fp.min_range, fp.max_range));
    if (fp.angular_filter_enabled && !fp.angular_ranges.empty())
      filters_.push_back(
        std::make_unique<AngularFilter>(fp.angular_ranges, fp.angular_invert));
    if (fp.box_filter_enabled)
      filters_.push_back(std::make_unique<BoxFilter>(fp.box_min, fp.box_max));
  }

  RCLCPP_INFO(logger_, "polka: source '%s' subscribed to '%s' (%s), %zu filters%s",
    config.name.c_str(), config.topic.c_str(),
    config.type == SourceType::POINTCLOUD2 ? "PointCloud2" : "LaserScan",
    filters_.size(),
    deskew_enabled_ ? ", deskewing enabled" : "");
}

bool SourceAdapter::validate_fields(const sensor_msgs::msg::PointCloud2 & msg)
{
  bool has_x = false, has_y = false, has_z = false, has_intensity = false;
  for (const auto & field : msg.fields) {
    if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) has_x = true;
    if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) has_y = true;
    if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) has_z = true;
    if (field.name == "intensity") has_intensity = true;
  }
  if (!has_x || !has_y || !has_z) {
    RCLCPP_ERROR(logger_,
      "polka: source '%s' missing required FLOAT32 x/y/z fields - dropping all messages",
      config_.name.c_str());
    return false;
  }
  missing_intensity_ = !has_intensity;
  return true;
}

void SourceAdapter::detect_timestamp_field(const sensor_msgs::msg::PointCloud2 & msg)
{
  timestamp_field_detected_ = true;
  has_timestamp_field_ = false;

  // Known per-point timestamp field names (priority order)
  static const std::vector<std::string> known_names = {
    "time", "t", "timestamp", "time_stamp", "offset_time", "timeStamp"
  };

  std::vector<std::string> candidates;
  if (timestamp_field_hint_ != "auto") {
    candidates.push_back(timestamp_field_hint_);
  } else {
    candidates = known_names;
  }

  for (const auto & field : msg.fields) {
    for (const auto & name : candidates) {
      if (field.name == name) {
        if (field.datatype == sensor_msgs::msg::PointField::FLOAT32 ||
            field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
          has_timestamp_field_ = true;
          timestamp_field_offset_ = field.offset;
          timestamp_field_datatype_ = field.datatype;
          RCLCPP_INFO(logger_,
            "polka: source '%s' detected per-point timestamp field '%s' (offset=%u, %s)",
            config_.name.c_str(), field.name.c_str(), field.offset,
            field.datatype == sensor_msgs::msg::PointField::FLOAT64 ? "FLOAT64" : "FLOAT32");
          return;
        }
      }
    }
  }

  RCLCPP_INFO(logger_,
    "polka: source '%s' has no per-point timestamp field, per-point deskewing disabled",
    config_.name.c_str());
}

double SourceAdapter::extract_point_time(const uint8_t * point_data) const
{
  if (timestamp_field_datatype_ == sensor_msgs::msg::PointField::FLOAT64) {
    double val;
    std::memcpy(&val, point_data + timestamp_field_offset_, sizeof(double));
    return val;
  } else {
    float val;
    std::memcpy(&val, point_data + timestamp_field_offset_, sizeof(float));
    return static_cast<double>(val);
  }
}

void SourceAdapter::deskew_cloud(
  CloudT & cloud,
  const sensor_msgs::msg::PointCloud2 & raw_msg,
  const AveragedImu & imu)
{
  size_t n = cloud.size();
  if (n == 0 || n != static_cast<size_t>(raw_msg.width) * raw_msg.height) return;

  // Rotate IMU data from IMU frame into sensor frame (identity if same frame or TF unavailable)
  Eigen::Matrix3d R_imu_to_sensor = Eigen::Matrix3d::Identity();
  if (tf_buffer_ && !imu.frame_id.empty()) {
    std::string sensor_frame;
    { std::lock_guard<std::mutex> lock(meta_mutex_); sensor_frame = frame_id_; }
    if (!sensor_frame.empty() && sensor_frame != imu.frame_id) {
      try {
        auto tf_msg = tf_buffer_->lookupTransform(
          sensor_frame, imu.frame_id, tf2::TimePointZero);
        R_imu_to_sensor = tf2::transformToEigen(tf_msg.transform).rotation();
      } catch (const tf2::TransformException &) {
        // Identity fallback — same behavior as before TF rotation was added
      }
    }
  }

  const Eigen::Vector3d angular_vel = R_imu_to_sensor * imu.angular_vel;
  const Eigen::Vector3d accel = R_imu_to_sensor * imu.linear_accel;
  const double header_sec = rclcpp::Time(raw_msg.header.stamp).seconds();

  const uint8_t * raw_data = raw_msg.data.data();
  const uint32_t point_step = raw_msg.point_step;

  for (size_t i = 0; i < n; ++i) {
    double pt_time = extract_point_time(raw_data + i * point_step);

    // Interpret: if >1e8 it's absolute Unix time, otherwise relative offset from scan start
    double dt = (pt_time > 1e8) ? (pt_time - header_sec) : pt_time;

    if (std::abs(dt) < 1e-9) continue;

    Eigen::Isometry3d delta = compute_motion_delta(angular_vel, accel, dt);
    Eigen::Vector3d p(cloud[i].x, cloud[i].y, cloud[i].z);
    Eigen::Vector3d corrected = delta.inverse() * p;
    cloud[i].x = static_cast<float>(corrected.x());
    cloud[i].y = static_cast<float>(corrected.y());
    cloud[i].z = static_cast<float>(corrected.z());
  }
}

void SourceAdapter::apply_filters(CloudT & cloud)
{
  for (auto & filter : filters_) {
    filter->apply(cloud, frame_id_);
  }
}

void SourceAdapter::store_cloud(CloudT::Ptr cloud, const std_msgs::msg::Header & header)
{
  {
    std::lock_guard<std::mutex> lock(meta_mutex_);
    frame_id_ = header.frame_id;
    last_received_time_ = rclcpp::Time(header.stamp);
  }
  std::atomic_store(&buffer_, std::static_pointer_cast<CloudT>(cloud));
  has_received_.store(true);
  message_counter_.fetch_add(1);
}

void SourceAdapter::pc2_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Validate fields on first message
  if (!fields_validated_) {
    fields_validated_ = true;
    fields_valid_ = validate_fields(*msg);
  }
  if (!fields_valid_) return;

  if (missing_intensity_) {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 10000,
      "polka: source '%s' missing 'intensity' field - publishing with intensity=0",
      config_.name.c_str());
  }

  // Detect per-point timestamp field on first message
  if (!timestamp_field_detected_)
    detect_timestamp_field(*msg);

  auto cloud = std::make_shared<CloudT>();
  pcl::fromROSMsg(*msg, *cloud);

  // Per-point deskewing (before filters, in sensor frame)
  if (deskew_enabled_ && has_timestamp_field_ && get_imu_) {
    auto imu = get_imu_();
    if (imu && imu->valid)
      deskew_cloud(*cloud, *msg, *imu);
  }

  apply_filters(*cloud);
  store_cloud(cloud, msg->header);
}

void SourceAdapter::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  // Convert LaserScan -> PointCloud2 -> CloudT
  sensor_msgs::msg::PointCloud2 pc2_msg;
  projector_.projectLaser(*msg, pc2_msg);

  if (!fields_validated_) {
    fields_validated_ = true;
    fields_valid_ = validate_fields(pc2_msg);
  }
  if (!fields_valid_) return;

  auto cloud = std::make_shared<CloudT>();
  pcl::fromROSMsg(pc2_msg, *cloud);
  apply_filters(*cloud);
  store_cloud(cloud, msg->header);
}

CloudT::ConstPtr SourceAdapter::get_latest() const
{
  return std::atomic_load(&buffer_);
}

std::string SourceAdapter::frame_id() const
{
  std::lock_guard<std::mutex> lock(meta_mutex_);
  return frame_id_;
}

bool SourceAdapter::is_stale(double timeout_sec, const rclcpp::Time & now) const
{
  if (!has_received_.load()) return true;
  std::lock_guard<std::mutex> lock(meta_mutex_);
  return (now - last_received_time_).seconds() > timeout_sec;
}

rclcpp::Time SourceAdapter::last_stamp() const
{
  std::lock_guard<std::mutex> lock(meta_mutex_);
  return last_received_time_;
}

void SourceAdapter::rebuild_filters(const FilterParams & fp)
{
  config_.filter_params = fp;
  filters_.clear();
  if (gpu_filters_) return;
  if (fp.range_filter_enabled)
    filters_.push_back(std::make_unique<RangeFilter>(fp.min_range, fp.max_range));
  if (fp.angular_filter_enabled && !fp.angular_ranges.empty())
    filters_.push_back(std::make_unique<AngularFilter>(fp.angular_ranges, fp.angular_invert));
  if (fp.box_filter_enabled)
    filters_.push_back(std::make_unique<BoxFilter>(fp.box_min, fp.box_max));
}

}  // namespace polka
