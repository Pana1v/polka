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

#ifndef POLKA__SOURCE_ADAPTER_HPP
#define POLKA__SOURCE_ADAPTER_HPP

#include "polka/types.hpp"
#include "polka/filters/i_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>

#include <Eigen/Core>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <atomic>

namespace polka {

struct AveragedImu;  // forward declaration

class SourceAdapter {
public:
  using ImuGetter = std::function<std::shared_ptr<const AveragedImu>()>;

  SourceAdapter(rclcpp::Node * node, const SourceConfig & config, bool gpu_filters = false,
                ImuGetter imu_getter = nullptr, bool deskew_enabled = false,
                const std::string & timestamp_field_hint = "auto");

  CloudT::ConstPtr get_latest() const;
  bool is_stale(double timeout_sec, const rclcpp::Time & now) const;
  rclcpp::Time last_stamp() const;
  std::string name() const { return config_.name; }
  std::string frame_id() const { return frame_id_; }
  uint64_t message_count() const { return message_counter_.load(); }
  const FilterParams & filter_params() const { return config_.filter_params; }
  void rebuild_filters(const FilterParams & fp);

private:
  void pc2_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  bool validate_fields(const sensor_msgs::msg::PointCloud2 & msg);
  void apply_filters(CloudT & cloud);
  void store_cloud(CloudT::Ptr cloud, const std_msgs::msg::Header & header);

  // Per-point deskewing
  void detect_timestamp_field(const sensor_msgs::msg::PointCloud2 & msg);
  double extract_point_time(const uint8_t * point_data) const;
  void deskew_cloud(CloudT & cloud, const sensor_msgs::msg::PointCloud2 & raw_msg,
                    const AveragedImu & imu);

  rclcpp::Node * node_;
  SourceConfig config_;
  rclcpp::Logger logger_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::shared_ptr<CloudT> buffer_;
  rclcpp::Time last_received_time_;
  std::string frame_id_;

  std::atomic<bool> has_received_{false};
  std::atomic<uint64_t> message_counter_{0};

  laser_geometry::LaserProjection projector_;
  bool fields_validated_{false};
  bool fields_valid_{false};
  bool missing_intensity_{false};

  std::vector<std::unique_ptr<IFilter>> filters_;
  bool gpu_filters_{false};

  // Deskewing state
  ImuGetter get_imu_;
  bool deskew_enabled_{false};
  std::string timestamp_field_hint_{"auto"};
  bool timestamp_field_detected_{false};
  bool has_timestamp_field_{false};
  uint32_t timestamp_field_offset_{0};
  uint8_t timestamp_field_datatype_{0};  // FLOAT32 or FLOAT64
};

}  // namespace polka

#endif  // POLKA__SOURCE_ADAPTER_HPP
