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
#include "polka/diag/diagnostics_reporter.hpp"
#include "polka/diag/drift_tracker.hpp"
#include "polka/diag/stat_counters.hpp"
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

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace polka {

class PolkaNode : public rclcpp::Node {
public:
  explicit PolkaNode(const rclcpp::NodeOptions & options);

private:
  // Everything owned per source, bundled so runtime add/remove can never skew
  // parallel indices. Order always follows source_names / config_.sources.
  // adapter == nullptr marks a "pending" source: declared in source_names but
  // its topic parameter is still empty; it activates on a later reconfigure.
  struct SourceSlot {
    std::unique_ptr<SourceAdapter> adapter;
    Eigen::Isometry3d last_good_transform = Eigen::Isometry3d::Identity();
    DriftTracker drift;
    StatWindow stats_window;
    std::chrono::steady_clock::time_point created_at{};
  };

  void output_callback();
  rclcpp::Time compute_output_stamp(const std::vector<rclcpp::Time> & stamps);
  // Convert each point's absolute Unix 'time' to a relative offset (seconds) from
  // the output header stamp, the convention deskewing consumers (e.g. GLIM) expect.
  void rebase_point_time(CloudT & cloud, const rclcpp::Time & stamp);
  // Serialize the merged cloud to a message, honouring point_timestamps.enabled
  // (drops the 'time' field when disabled, for a legacy x/y/z/intensity output).
  sensor_msgs::msg::PointCloud2 to_cloud_msg(const CloudT & cloud) const;
  void log_startup_banner() const;
  // One-shot check that the node clock and the incoming sensor stamps agree. Emits a
  // single actionable warning when they don't (the classic rosbag-without-sim-time or
  // sim-time-without-/clock misconfiguration), then latches via clock_diagnosed_.
  void diagnose_clock_health(const rclcpp::Time & now);

  // --- Two-phase runtime reconfiguration ---
  // Humble's on-set callback fires BEFORE the proposed values are committed to
  // parameter storage, so it can only validate (against the proposed list) and
  // must never apply. On success a zero-delay one-shot timer is armed; it runs
  // on the executor thread after the commit, re-reads committed storage and
  // applies the diff. This also serializes apply against output_callback().
  rcl_interfaces::msg::SetParametersResult validate_parameters(
    const std::vector<rclcpp::Parameter> & params);
  void schedule_apply();
  void apply_reconfigure();
  void rebuild_sources(
    const MergeConfig & old_config, const MergeConfig & new_config,
    std::vector<std::string> & changes);
  std::unique_ptr<SourceAdapter> make_adapter(
    const SourceConfig & sc, const MergeConfig & cfg);
  SourceAdapter::ImuGetter make_imu_getter(const MergeConfig & cfg);
  DriftTracker::Config drift_config(
    const SourceConfig & sc, const DiagnosticsConfig & d) const;

  // --- Diagnostics ---
  void configure_diagnostics(const DiagnosticsConfig & d);
  void diag_callback();
  void mark_published();

  MergeConfig config_;

  // Input
  std::vector<SourceSlot> sources_;
  std::shared_ptr<ImuBuffer> global_imu_;

  // Transform
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

  // Runtime reconfiguration.
  //
  // Thread-safety model: validate_parameters (parameter service), the apply
  // timer, output/diagnostics timers, and all subscriptions share the node's
  // default MutuallyExclusive callback group, so every mutation of sources_/
  // config_/publishers is serialized even under a multithreaded executor.
  // Unsupported: moving polka's callbacks into a Reentrant group, or calling
  // set_parameters() from a thread that is not spinning this node.
  ConfigLoader config_loader_;
  std::vector<std::string> source_names_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr apply_timer_;
  bool applying_{false};  // suppresses re-entrant validation while apply declares params
  uint64_t reconfig_count_{0};

  // Diagnostics
  std::unique_ptr<DiagnosticsReporter> diag_reporter_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  StatCounters cloud_out_counters_;
  StatCounters scan_out_counters_;
  StatWindow cloud_out_window_;
  StatWindow scan_out_window_;
  int64_t last_points_in_{-1};
  int64_t last_points_out_{-1};
  std::chrono::steady_clock::time_point start_steady_{};
  std::chrono::steady_clock::time_point last_publish_steady_{};
  bool ever_published_{false};
};

}  // namespace polka

#endif  // POLKA__POLKA_NODE_HPP_
