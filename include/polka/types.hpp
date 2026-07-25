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

#ifndef POLKA__TYPES_HPP_
#define POLKA__TYPES_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace polka
{

// Point with a per-point acquisition timestamp.
//
// Internally 'time' holds absolute seconds since the Unix epoch: this is the
// common reference that lets points from sources with different scan-start times
// be merged consistently. float64 is required - a float32 cannot represent
// ~1.7e9 s at sub-millisecond resolution. Sources that publish a relative
// per-point offset are rebased to absolute in SourceAdapter; sources without a
// per-point time field fall back to the message header stamp for every point.
// At publish time, with point_timestamps.mode = OFFSET, PolkaNode::rebase_point_time
// converts 'time' to an offset relative to the merged cloud header (the convention
// deskewing consumers expect); with mode = ABSOLUTE it is emitted unchanged.
struct EIGEN_ALIGN16 PointXYZIT
{
  PCL_ADD_POINT4D;       // adds float x, y, z (+ padding) as a SSE-aligned block
  float intensity;
  double time;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

using PointT = PointXYZIT;
using CloudT = pcl::PointCloud<PointT>;

enum class SourceType { POINTCLOUD2, LASERSCAN };
enum class TimestampStrategy { EARLIEST, LATEST, AVERAGE, LOCAL };

// How the per-point 'time' field is emitted on the output cloud (when enabled).
//   OFFSET   - seconds relative to the output cloud header stamp (GLIM/deskew convention)
//   ABSOLUTE - raw Unix seconds, as received from the source LiDAR
enum class PerPointTimeMode { OFFSET, ABSOLUTE };

struct PerPointTimeConfig
{
  bool enabled = false;                          // emit a per-point 'time' field
  PerPointTimeMode mode = PerPointTimeMode::OFFSET;
};

struct FilterParams
{
  double min_range = 0.0;
  double max_range = 100.0;
  double min_range_sq = 0.0;
  double max_range_sq = 10000.0;

  bool range_filter_enabled = false;

  std::vector<std::pair<double, double>> angular_ranges;

  bool angular_filter_enabled = false;
  bool angular_invert = false;
  bool box_filter_enabled = false;

  Eigen::Vector3d box_min = Eigen::Vector3d(-100.0, -100.0, -100.0);
  Eigen::Vector3d box_max = Eigen::Vector3d(100.0, 100.0, 100.0);

  void compute_squared_ranges()
  {
    min_range_sq = min_range * min_range;
    max_range_sq = max_range * max_range;
  }

  void validate() const
  {
    if (min_range < 0) {
      throw std::invalid_argument("min_range must be non-negative");
    }
    if (max_range <= min_range) {
      throw std::invalid_argument("max_range must be greater than min_range");
    }
    for (const auto & r : angular_ranges) {
      if (r.first < 0.0 || r.first > 360.0 ||
        r.second < 0.0 || r.second > 360.0)
      {
        throw std::invalid_argument("angular range values must be in [0, 360] degrees");
      }
    }
    if (box_filter_enabled) {
      if (box_min.x() >= box_max.x() || box_min.y() >= box_max.y() ||
        box_min.z() >= box_max.z())
      {
        throw std::invalid_argument("box_min must be less than box_max in all dimensions");
      }
    }
  }
};

struct FlattenParams
{
  double z_min = -0.15;
  double z_max = 0.15;
  double angle_min = -3.14159265;
  double angle_max = 3.14159265;
  double angle_increment = 0.00436332;
  double range_min = 0.10;
  double range_max = 100.0;
  int n_bins = 0;

  void compute_bins()
  {
    n_bins = static_cast<int>((angle_max - angle_min) / angle_increment);
  }

  void validate() const
  {
    if (z_min >= z_max) {
      throw std::invalid_argument("z_min must be less than z_max");
    }
    if (angle_min >= angle_max) {
      throw std::invalid_argument("angle_min must be less than angle_max");
    }
    if (angle_increment <= 0) {
      throw std::invalid_argument("angle_increment must be positive");
    }
    if (range_min < 0 || range_max <= range_min) {
      throw std::invalid_argument("range_min must be non-negative and less than range_max");
    }
  }
};

struct SourceConfig
{
  std::string name;
  std::string topic;
  std::string imu_topic;  // per-source IMU override (empty = use global)
  SourceType type = SourceType::POINTCLOUD2;
  std::string qos_reliability = "best_effort";
  int qos_history_depth = 1;
  FilterParams filter_params;
  double expected_rate = 0.0;  // Hz; 0 = auto-baseline from observed rates

  // Everything that is baked into a constructed SourceAdapter (subscription
  // topic/type/QoS, per-source IMU). When any of these change at runtime the
  // adapter must be recreated; filters and expected_rate apply in place.
  bool same_identity(const SourceConfig & other) const
  {
    return topic == other.topic && type == other.type &&
           qos_reliability == other.qos_reliability &&
           qos_history_depth == other.qos_history_depth &&
           imu_topic == other.imu_topic;
  }
};

struct HeightCapConfig
{
  bool enabled = false;
  double z_min = -1.0;
  double z_max = 3.0;
};

struct VoxelConfig
{
  bool enabled = false;
  float leaf_x = 0.0f;
  float leaf_y = 0.0f;
  float leaf_z = 0.0f;
};

struct ExclusionBox
{
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  std::string label;
};

struct SelfFilterConfig
{
  bool enabled = false;
  std::vector<ExclusionBox> boxes;
};

struct OutputQosConfig
{
  std::string reliability = "reliable";
  std::string durability = "volatile";
  int history_depth = 10;
  std::string liveliness = "automatic";
  double liveliness_lease_duration_ms = 0.0;
  double deadline_ms = 0.0;
  double lifespan_ms = 0.0;
};

struct CloudOutputConfig
{
  bool enabled = true;
  std::string topic = "~/merged_cloud";
  OutputQosConfig qos;
  FilterParams filters;
  HeightCapConfig height_cap;
  VoxelConfig voxel;
  SelfFilterConfig self_filter;
};

struct ScanOutputConfig
{
  bool enabled = false;
  std::string topic = "~/merged_scan";
  OutputQosConfig qos;
  FlattenParams flatten;
};

struct MotionCompensationConfig
{
  bool enabled = false;
  std::string imu_topic = "";                    // sensor_msgs/Imu topic
  double max_imu_age = 0.2;                      // reject stale IMU data (seconds)
  int imu_buffer_size = 200;                      // ring buffer capacity
  bool per_point_deskew = true;                   // per-point correction if timestamps available
  std::string deskew_timestamp_field = "auto";    // "auto" or specific field name
  std::string imu_frame = "";                     // empty = auto-detect from IMU msg header
};

struct DiagnosticsConfig
{
  bool enabled = true;
  double publish_period_sec = 1.0;
  // Timing drift: EWMA of (source stamp - peer median) trending past a bound.
  double timing_threshold_sec = 0.1;
  double timing_ewma_alpha = 0.2;
  int timing_min_ticks = 5;
  // Rate drift: windowed rate sagging below the expected rate.
  double rate_sag_pct = 20.0;
  int rate_min_ticks = 5;
  double rate_baseline_sec = 10.0;

  void validate() const
  {
    if (publish_period_sec <= 0.0) {
      throw std::invalid_argument("publish_period_sec must be positive");
    }
    if (timing_threshold_sec <= 0.0) {
      throw std::invalid_argument("timing_drift.threshold_sec must be positive");
    }
    if (timing_ewma_alpha <= 0.0 || timing_ewma_alpha > 1.0) {
      throw std::invalid_argument("timing_drift.ewma_alpha must be in (0, 1]");
    }
    if (timing_min_ticks < 1 || rate_min_ticks < 1) {
      throw std::invalid_argument("min_ticks must be >= 1");
    }
    if (rate_sag_pct <= 0.0 || rate_sag_pct >= 100.0) {
      throw std::invalid_argument("rate_drift.sag_pct must be in (0, 100)");
    }
    if (rate_baseline_sec <= 0.0) {
      throw std::invalid_argument("rate_drift.baseline_sec must be positive");
    }
  }
};

struct MergeConfig
{
  std::string output_frame_id = "base_link";
  double output_rate = 20.0;
  double source_timeout = 0.5;
  // A source past source_timeout is reused (last-good cloud + last-good TF)
  // rather than dropped from the merge until it's also past this longer
  // bound - smooths over a single momentarily-late tick (real gaps on one
  // source shouldn't visibly shrink the merged cloud) while still dropping
  // a source that's genuinely gone. Must be >= source_timeout.
  double source_stale_reuse_window = 1.5;
  bool enable_gpu = true;

  TimestampStrategy timestamp_strategy = TimestampStrategy::EARLIEST;
  double max_source_spread_warn = 0.05;

  PerPointTimeConfig point_timestamps;
  // Skip publishing a cloud whose output stamp equals the previous one (and never
  // re-publish the last cloud when no source is fresh). Prevents downstream SLAM
  // (e.g. GLIM) from seeing two scans with an identical header.stamp.
  bool suppress_duplicate_timestamps = true;

  MotionCompensationConfig motion_compensation;
  DiagnosticsConfig diagnostics;

  CloudOutputConfig cloud_output;
  ScanOutputConfig scan_output;
  std::vector<SourceConfig> sources;
};

}  // namespace polka

POINT_CLOUD_REGISTER_POINT_STRUCT(
  polka::PointXYZIT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, time, time))

#endif  // POLKA__TYPES_HPP_
