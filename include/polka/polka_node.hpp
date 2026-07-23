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
#include "polka/config/config_loader.hpp"
#include "polka/input/source_adapter.hpp"
#include "polka/input/imu_buffer.hpp"
#include "polka/merge_engine/i_merge_engine.hpp"
#include "polka/output/output_pipeline.hpp"
#include "polka/output/scan_builder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace polka {

class PolkaNode : public rclcpp::Node {
public:
  explicit PolkaNode(const rclcpp::NodeOptions & options);

private:
  void output_callback();
  rclcpp::Time compute_output_stamp(const std::vector<rclcpp::Time> & stamps);
  // Convert each point's absolute Unix 'time' to a relative offset (seconds) from
  // the output header stamp, the convention deskewing consumers (e.g. GLIM) expect.
  void rebase_point_time(CloudT & cloud, const rclcpp::Time & stamp);
  // Serialize the merged cloud to a message, honouring point_timestamps.enabled
  // (drops the 'time' field when disabled, for a legacy x/y/z/intensity output).
  sensor_msgs::msg::PointCloud2 to_cloud_msg(const CloudT & cloud) const;
  bool reconfigure();
  void log_startup_banner() const;
  // One-shot check that the node clock and the incoming sensor stamps agree. Emits a
  // single actionable warning when they don't (the classic rosbag-without-sim-time or
  // sim-time-without-/clock misconfiguration), then latches via clock_diagnosed_.
  void diagnose_clock_health(const rclcpp::Time & now);
  // Perf instrumentation: accumulates merge-stage latency, logs a mean/max every
  // N calls. `engine_label` is just "CPU" or "CUDA" for the log line.
  void log_merge_perf(double us, const char * engine_label);

  MergeConfig config_;

  // Input
  std::vector<std::unique_ptr<SourceAdapter>> sources_;
  std::shared_ptr<ImuBuffer> global_imu_;

  // Transform
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<Eigen::Isometry3d> last_good_transforms_;

  // Processing
  std::unique_ptr<IMergeEngine> merge_engine_;
  OutputPipeline output_pipeline_;
  ScanBuilder scan_builder_;

  // Output
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr output_timer_;

  // Stale data buffering — ensures publishing at output_rate even without new data
  CloudT::Ptr last_cloud_;
  std::vector<float> last_scan_ranges_;
  rclcpp::Time last_cloud_stamp_;
  mutable std::mutex last_data_mutex_;

  // Set once diagnose_clock_health() has emitted its warning, so it stays quiet after.
  bool clock_diagnosed_{false};

  // Perf instrumentation: rolling merge-stage latency, logged every N calls.
  uint64_t merge_calls_{0};
  double merge_total_us_{0.0};
  double merge_max_us_{0.0};

  // Runtime reconfiguration
  ConfigLoader config_loader_;
  std::vector<std::string> source_names_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace polka

#endif  // POLKA__POLKA_NODE_HPP_
