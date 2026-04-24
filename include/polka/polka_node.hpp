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

#ifndef POLKA__POLKA_NODE_HPP_
#define POLKA__POLKA_NODE_HPP_

#include "polka/types.hpp"
#include "polka/config_loader.hpp"
#include "polka/source_adapter.hpp"
#include "polka/imu_buffer.hpp"
#include "polka/merge_engine/i_merge_engine.hpp"
#include "polka/filters/i_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <string>
#include <mutex>

namespace polka {

class PolkaNode : public rclcpp::Node {
public:
  explicit PolkaNode(const rclcpp::NodeOptions & options);

private:
  void output_callback();
  void publish_cloud(CloudT::ConstPtr cloud, const rclcpp::Time & stamp);
  void publish_scan(CloudT::ConstPtr cloud, const rclcpp::Time & stamp);
  void publish_scan_from_ranges(const std::vector<float> & ranges, const rclcpp::Time & stamp);
  PipelineConfig build_pipeline_config() const;
  rclcpp::Time compute_output_stamp(const std::vector<rclcpp::Time> & stamps);
  void build_output_filters();
  bool reconfigure();
  void voxel_downsample(CloudT & cloud);
  void height_cap(CloudT & cloud);

  // IMU-based motion compensation (global)

  MergeConfig config_;

  std::vector<std::unique_ptr<SourceAdapter>> sources_;
  std::unique_ptr<IMergeEngine> merge_engine_;
  std::vector<std::unique_ptr<IFilter>> output_filters_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr output_timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  // TF failure tracking per source
  std::vector<Eigen::Isometry3d> last_good_transforms_;
  std::vector<int> tf_fail_counts_;

  // Runtime reconfiguration
  ConfigLoader config_loader_;
  std::vector<std::string> source_names_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Global IMU buffer (shared with SourceAdapters that don't have per-source IMU)
  std::shared_ptr<ImuBuffer> global_imu_;

  // Stale data buffering - ensures publishing at specified frequency
  CloudT::Ptr last_cloud_;
  std::vector<float> last_scan_ranges_;
  rclcpp::Time last_cloud_stamp_;
  mutable std::mutex last_data_mutex_;
};

}  // namespace polka

#endif  // POLKA__POLKA_NODE_HPP_
