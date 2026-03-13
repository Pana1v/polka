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

#ifndef POLKA__TYPES_HPP
#define POLKA__TYPES_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

namespace polka {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

enum class SourceType { POINTCLOUD2, LASERSCAN };
enum class TimestampStrategy { EARLIEST, LATEST, AVERAGE, LOCAL };

struct FilterParams {
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

  void compute_squared_ranges() {
    min_range_sq = min_range * min_range;
    max_range_sq = max_range * max_range;
  }

  void validate() const {
    if (min_range < 0)
      throw std::invalid_argument("min_range must be non-negative");
    if (max_range <= min_range)
      throw std::invalid_argument("max_range must be greater than min_range");
    for (const auto & r : angular_ranges) {
      if (r.first < 0.0 || r.first > 360.0 ||
          r.second < 0.0 || r.second > 360.0)
        throw std::invalid_argument("angular range values must be in [0, 360] degrees");
    }
    if (box_filter_enabled) {
      if (box_min.x() >= box_max.x() || box_min.y() >= box_max.y() ||
          box_min.z() >= box_max.z())
        throw std::invalid_argument("box_min must be less than box_max in all dimensions");
    }
  }
};

struct FlattenParams {
  double z_min = -0.15;
  double z_max = 0.15;
  double angle_min = -3.14159265;
  double angle_max = 3.14159265;
  double angle_increment = 0.00436332;
  double range_min = 0.10;
  double range_max = 100.0;
  int n_bins = 0;

  void compute_bins() {
    n_bins = static_cast<int>((angle_max - angle_min) / angle_increment);
  }

  void validate() const {
    if (z_min >= z_max)
      throw std::invalid_argument("z_min must be less than z_max");
    if (angle_min >= angle_max)
      throw std::invalid_argument("angle_min must be less than angle_max");
    if (angle_increment <= 0)
      throw std::invalid_argument("angle_increment must be positive");
    if (range_min < 0 || range_max <= range_min)
      throw std::invalid_argument("range_min must be non-negative and less than range_max");
  }
};

struct SourceConfig {
  std::string name;
  std::string topic;
  SourceType type = SourceType::POINTCLOUD2;
  std::string qos_reliability = "best_effort";
  int qos_history_depth = 1;
  FilterParams filter_params;
};

struct HeightCapConfig {
  bool enabled = false;
  double z_min = -1.0;
  double z_max = 3.0;
};

struct VoxelConfig {
  bool enabled = false;
  float leaf_x = 0.0f;
  float leaf_y = 0.0f;
  float leaf_z = 0.0f;
};

struct ExclusionBox {
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  std::string label;
};

struct SelfFilterConfig {
  bool enabled = false;
  std::vector<ExclusionBox> boxes;
};

struct CloudOutputConfig {
  bool enabled = true;
  std::string topic = "~/merged_cloud";
  FilterParams filters;
  HeightCapConfig height_cap;
  VoxelConfig voxel;
  SelfFilterConfig self_filter;
};

struct ScanOutputConfig {
  bool enabled = false;
  std::string topic = "~/merged_scan";
  FlattenParams flatten;
};

struct MotionCompensationConfig {
  bool enabled = false;
  std::string imu_topic = "";                    // sensor_msgs/Imu topic
  double max_imu_age = 0.2;                      // reject stale IMU data (seconds)
  int imu_buffer_size = 200;                      // ring buffer capacity
  bool per_point_deskew = true;                   // per-point correction if timestamps available
  std::string deskew_timestamp_field = "auto";    // "auto" or specific field name
};

struct MergeConfig {
  std::string output_frame_id = "base_link";
  double output_rate = 20.0;
  double source_timeout = 0.5;
  bool enable_gpu = true;

  TimestampStrategy timestamp_strategy = TimestampStrategy::EARLIEST;
  double max_source_spread_warn = 0.05;

  MotionCompensationConfig motion_compensation;

  CloudOutputConfig cloud_output;
  ScanOutputConfig scan_output;
  std::vector<SourceConfig> sources;
};

}  // namespace polka

#endif  // POLKA__TYPES_HPP
