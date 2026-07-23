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

#include "polka/diag/diagnostics_reporter.hpp"

#include <cstdio>

namespace polka {

namespace {

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

KeyValue kv(const std::string & key, const std::string & value)
{
  KeyValue out;
  out.key = key;
  out.value = value;
  return out;
}

std::string fmt(const char * format, double value)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), format, value);
  return buf;
}

std::string bool_str(bool v) {return v ? "true" : "false";}

// Rates carry -1 sentinels for "not measurable this tick" so consumers can
// tell "zero traffic" apart from "no window yet".
std::string rate_str(const StatWindow::Rates & r, double StatWindow::Rates::* field)
{
  return r.valid ? fmt("%.2f", r.*field) : "-1";
}

}  // namespace

DiagnosticsReporter::DiagnosticsReporter(rclcpp::Node * node)
: prefix_(std::string(node->get_name()) + ": ")
{
  // Absolute /diagnostics: the ecosystem-wide aggregation topic that
  // rqt_robot_monitor and diagnostic_aggregator subscribe to.
  pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10));
}

void DiagnosticsReporter::publish(
  const NodeReport & node_report,
  const OutputReport & output_report,
  const ImuReport & imu_report,
  const std::vector<SourceReport> & source_reports,
  const rclcpp::Time & stamp)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.reserve(3 + source_reports.size());

  {
    DiagnosticStatus st;
    st.name = prefix_ + "node";
    st.hardware_id = "polka";
    const bool degraded = node_report.sources_fresh < node_report.sources_total;
    st.level = degraded ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
    char msg[96];
    std::snprintf(msg, sizeof(msg), "%s pipeline, %zu/%zu sources fresh",
      node_report.engine.c_str(), node_report.sources_fresh, node_report.sources_total);
    st.message = msg;
    st.values.push_back(kv("engine", node_report.engine));
    st.values.push_back(kv("sources_total", std::to_string(node_report.sources_total)));
    st.values.push_back(kv("sources_fresh", std::to_string(node_report.sources_fresh)));
    st.values.push_back(kv("sources_pending", std::to_string(node_report.sources_pending)));
    st.values.push_back(kv("output_rate_hz", fmt("%.1f", node_report.output_rate_hz)));
    st.values.push_back(kv("uptime_sec", fmt("%.0f", node_report.uptime_sec)));
    st.values.push_back(kv("reconfig_count", std::to_string(node_report.reconfig_count)));
    array.status.push_back(std::move(st));
  }

  {
    DiagnosticStatus st;
    st.name = prefix_ + "output";
    st.hardware_id = "polka";
    st.level = output_report.publish_overdue ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
    if (output_report.publish_overdue) {
      st.message = "sources fresh but nothing published";
    } else if (output_report.cloud_rates.valid || output_report.scan_rates.valid) {
      const auto & r = output_report.cloud_rates.valid
        ? output_report.cloud_rates : output_report.scan_rates;
      st.message = "publishing " + fmt("%.1f", r.msg_hz) + " Hz";
    } else {
      st.message = "no output yet";
    }
    st.values.push_back(kv("cloud_topic", output_report.cloud_topic));
    st.values.push_back(kv("scan_topic", output_report.scan_topic));
    st.values.push_back(kv("cloud_rate_hz",
      rate_str(output_report.cloud_rates, &StatWindow::Rates::msg_hz)));
    st.values.push_back(kv("cloud_bandwidth_Bps",
      rate_str(output_report.cloud_rates, &StatWindow::Rates::bytes_per_sec)));
    st.values.push_back(kv("scan_rate_hz",
      rate_str(output_report.scan_rates, &StatWindow::Rates::msg_hz)));
    st.values.push_back(kv("scan_bandwidth_Bps",
      rate_str(output_report.scan_rates, &StatWindow::Rates::bytes_per_sec)));
    st.values.push_back(kv("points_in", std::to_string(output_report.points_in)));
    st.values.push_back(kv("points_out", std::to_string(output_report.points_out)));
    st.values.push_back(kv("engine", output_report.engine));
    st.values.push_back(kv("last_publish_age_sec",
      fmt("%.2f", output_report.last_publish_age_sec)));
    array.status.push_back(std::move(st));
  }

  if (imu_report.enabled) {
    DiagnosticStatus st;
    st.name = prefix_ + "imu";
    st.hardware_id = "polka";
    if (!imu_report.valid) {
      st.level = DiagnosticStatus::WARN;
      st.message = "no IMU data received on '" + imu_report.topic + "'";
    } else {
      st.level = DiagnosticStatus::OK;
      st.message = "ok";
    }
    st.values.push_back(kv("topic", imu_report.topic));
    st.values.push_back(kv("rate_hz",
      imu_report.rate_hz >= 0.0 ? fmt("%.2f", imu_report.rate_hz) : "-1"));
    st.values.push_back(kv("msg_age_sec",
      imu_report.msg_age_sec >= 0.0 ? fmt("%.3f", imu_report.msg_age_sec) : "-1"));
    st.values.push_back(kv("valid", bool_str(imu_report.valid)));
    array.status.push_back(std::move(st));
  }

  for (const auto & src : source_reports) {
    DiagnosticStatus st;
    st.name = prefix_ + "source " + src.name;
    st.hardware_id = src.name;

    if (src.fields_invalid) {
      st.level = DiagnosticStatus::ERROR;
      st.message = "missing required x/y/z fields - dropping all messages";
    } else if (src.receive_overdue) {
      st.level = DiagnosticStatus::ERROR;
      st.message = "never received on '" + src.topic + "'";
    } else if (src.pending) {
      st.level = DiagnosticStatus::WARN;
      st.message = "pending - set sources." + src.name + ".topic to activate";
    } else if (src.stale) {
      st.level = DiagnosticStatus::WARN;
      st.message = "stale (" + fmt("%.1f", src.msg_age_sec) + " s)";
    } else if (src.drift.rate_drift) {
      st.level = DiagnosticStatus::WARN;
      st.message = "rate " + fmt("%.1f", src.rates.msg_hz) +
        " Hz < expected " + fmt("%.1f", src.drift.expected_rate) + " Hz";
    } else if (src.drift.timing_drift) {
      st.level = DiagnosticStatus::WARN;
      st.message = "timing drift " + fmt("%+.3f", src.drift.offset_ewma_sec) +
        " s from peer median";
    } else if (!src.ever_received) {
      st.level = DiagnosticStatus::OK;
      st.message = "waiting for first message";
    } else {
      st.level = DiagnosticStatus::OK;
      st.message = "ok";
    }

    st.values.push_back(kv("topic", src.topic));
    st.values.push_back(kv("type", src.type_str));
    st.values.push_back(kv("frame_id", src.frame_id));
    st.values.push_back(kv("rate_hz", rate_str(src.rates, &StatWindow::Rates::msg_hz)));
    st.values.push_back(kv("expected_rate_hz", fmt("%.2f", src.drift.expected_rate)));
    st.values.push_back(kv("expected_rate_source", src.drift.expected_rate_source()));
    st.values.push_back(kv("bandwidth_Bps",
      rate_str(src.rates, &StatWindow::Rates::bytes_per_sec)));
    st.values.push_back(kv("msg_age_sec", fmt("%.3f", src.msg_age_sec)));
    st.values.push_back(kv("stamp_offset_sec",
      src.offset_valid ? fmt("%+.4f", src.stamp_offset_sec) : "0"));
    st.values.push_back(kv("stamp_offset_ewma_sec",
      src.drift.ewma_valid ? fmt("%+.4f", src.drift.offset_ewma_sec) : "0"));
    st.values.push_back(kv("points_raw_per_sec",
      rate_str(src.rates, &StatWindow::Rates::points_raw_per_sec)));
    // GPU builds run per-source filters inside the merge engine, so the kept
    // count is unknown there; the node passes points_kept_per_sec < 0.
    const bool kept_known = src.rates.valid && src.rates.points_kept_per_sec >= 0.0;
    st.values.push_back(kv("points_kept_per_sec",
      kept_known ? fmt("%.2f", src.rates.points_kept_per_sec) : "-1"));
    const bool drop_known = kept_known && src.rates.points_raw_per_sec > 0.0;
    st.values.push_back(kv("filter_drop_pct", drop_known
      ? fmt("%.1f", 100.0 * (1.0 - src.rates.points_kept_per_sec / src.rates.points_raw_per_sec))
      : "-1"));
    st.values.push_back(kv("pending", bool_str(src.pending)));
    st.values.push_back(kv("stale", bool_str(src.stale)));
    st.values.push_back(kv("timing_drift", bool_str(src.drift.timing_drift)));
    st.values.push_back(kv("rate_drift", bool_str(src.drift.rate_drift)));
    st.values.push_back(kv("filter_range_enabled", bool_str(src.filter_range_enabled)));
    st.values.push_back(kv("filter_angular_enabled", bool_str(src.filter_angular_enabled)));
    st.values.push_back(kv("filter_box_enabled", bool_str(src.filter_box_enabled)));
    st.values.push_back(kv("deskew_active", bool_str(src.deskew_active)));
    array.status.push_back(std::move(st));
  }

  pub_->publish(array);
}

}  // namespace polka
