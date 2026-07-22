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

#ifndef POLKA__DIAG__DIAGNOSTICS_REPORTER_HPP_
#define POLKA__DIAG__DIAGNOSTICS_REPORTER_HPP_

#include "polka/diag/stat_counters.hpp"
#include "polka/diag/drift_tracker.hpp"

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <string>
#include <vector>

namespace polka {

// Plain snapshots computed by PolkaNode's diagnostics tick. The reporter only
// formats and publishes them, so the node code stays free of key/value noise.

struct SourceReport {
  std::string name;
  std::string topic;
  std::string type_str;   // "pointcloud2" | "laserscan"
  std::string frame_id;
  bool pending = false;         // declared in source_names but topic still empty
  bool ever_received = false;
  bool receive_overdue = false;  // never received well past the grace period
  bool fields_invalid = false;   // missing x/y/z - source drops every message
  bool stale = false;
  StatWindow::Rates rates;
  double msg_age_sec = -1.0;     // node now - last stamp; <0 = unknown
  bool offset_valid = false;
  double stamp_offset_sec = 0.0;  // this stamp - peer median
  DriftTracker::Status drift;
};

struct OutputReport {
  std::string engine;  // "CUDA" | "CPU"
  std::string cloud_topic;  // empty when the cloud output is disabled
  std::string scan_topic;   // empty when the scan output is disabled
  StatWindow::Rates cloud_rates;
  StatWindow::Rates scan_rates;
  int64_t points_in = -1;   // merged input points last tick; -1 = unknown
  int64_t points_out = -1;  // published points last tick; -1 = unknown
  double last_publish_age_sec = -1.0;  // <0 = never published
  bool publish_overdue = false;  // sources fresh but output silent
};

struct NodeReport {
  std::string engine;
  size_t sources_total = 0;
  size_t sources_fresh = 0;
  size_t sources_pending = 0;
  double output_rate_hz = 0.0;
  double uptime_sec = 0.0;
  uint64_t reconfig_count = 0;
};

class DiagnosticsReporter {
public:
  explicit DiagnosticsReporter(rclcpp::Node * node);

  void publish(
    const NodeReport & node_report,
    const OutputReport & output_report,
    const std::vector<SourceReport> & source_reports,
    const rclcpp::Time & stamp);

private:
  std::string prefix_;  // "<node name>: " - lets consumers filter by node
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
};

}  // namespace polka

#endif  // POLKA__DIAG__DIAGNOSTICS_REPORTER_HPP_
