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

#include "polka/config/config_loader.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

namespace polka
{

ConfigLoader::ConfigLoader(rclcpp::Node * node)
: node_(node), logger_(node->get_logger())
{
  declare_defaults();
}

rclcpp::Parameter ConfigLoader::param(const std::string & name) const
{
  if (overlay_) {
    for (const auto & p : *overlay_) {
      if (p.get_name() == name) {return p;}
    }
  }
  return node_->get_parameter(name);
}

template<typename T>
T ConfigLoader::param_or(const std::string & name, const T & def) const
{
  if (overlay_) {
    for (const auto & p : *overlay_) {
      if (p.get_name() == name) {return p.get_value<T>();}
    }
  }
  if (node_->has_parameter(name)) {
    return node_->get_parameter(name).get_value<T>();
  }
  return def;
}

void ConfigLoader::declare_defaults()
{
  node_->declare_parameter<std::string>("output_frame_id", "base_link");
  node_->declare_parameter<double>("output_rate", 20.0);
  node_->declare_parameter<double>("source_timeout", 0.5);
  node_->declare_parameter<double>("source_stale_reuse_window", 1.5);
  node_->declare_parameter<std::string>("timestamp_strategy", "earliest");
  node_->declare_parameter<double>("max_source_spread_warn", 0.05);

  // Read-only: the merge engine (and its device buffers) is constructed once;
  // rclcpp itself rejects runtime sets of read-only parameters with a clear
  // reason, so no custom handling is needed.
  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only = true;
  node_->declare_parameter<bool>("enable_gpu", true, read_only);

  // per-point timestamp passthrough
  node_->declare_parameter<bool>("point_timestamps.enabled", false);
  node_->declare_parameter<std::string>("point_timestamps.mode", "offset");
  node_->declare_parameter<bool>("suppress_duplicate_timestamps", true);

  // motion compensation (IMU-based deskewing)
  node_->declare_parameter<bool>("motion_compensation.enabled", false);
  node_->declare_parameter<std::string>("motion_compensation.imu_topic", "");
  node_->declare_parameter<double>("motion_compensation.max_imu_age", 0.2);
  node_->declare_parameter<int>("motion_compensation.imu_buffer_size", 200);
  node_->declare_parameter<bool>("motion_compensation.per_point_deskew", true);
  node_->declare_parameter<std::string>("motion_compensation.deskew_timestamp_field", "auto");
  node_->declare_parameter<std::string>("motion_compensation.imu_frame", "");

  // diagnostics + drift detection
  node_->declare_parameter<bool>("diagnostics.enabled", true);
  node_->declare_parameter<double>("diagnostics.publish_period_sec", 1.0);
  node_->declare_parameter<double>("diagnostics.timing_drift.threshold_sec", 0.1);
  node_->declare_parameter<double>("diagnostics.timing_drift.ewma_alpha", 0.2);
  node_->declare_parameter<int>("diagnostics.timing_drift.min_ticks", 5);
  node_->declare_parameter<double>("diagnostics.rate_drift.sag_pct", 20.0);
  node_->declare_parameter<int>("diagnostics.rate_drift.min_ticks", 5);
  node_->declare_parameter<double>("diagnostics.rate_drift.baseline_sec", 10.0);

  // outputs.cloud
  node_->declare_parameter<bool>("outputs.cloud.enabled", true);
  node_->declare_parameter<std::string>("outputs.cloud.topic", "~/merged_cloud");
  node_->declare_parameter<std::string>("outputs.cloud.qos.reliability", "reliable");
  node_->declare_parameter<std::string>("outputs.cloud.qos.durability", "volatile");
  node_->declare_parameter<int>("outputs.cloud.qos.history_depth", 10);
  node_->declare_parameter<std::string>("outputs.cloud.qos.liveliness", "automatic");
  node_->declare_parameter<double>("outputs.cloud.qos.liveliness_lease_duration_ms", 0.0);
  node_->declare_parameter<double>("outputs.cloud.qos.deadline_ms", 0.0);
  node_->declare_parameter<double>("outputs.cloud.qos.lifespan_ms", 0.0);
  node_->declare_parameter<bool>("outputs.cloud.filters.range.enabled", false);
  node_->declare_parameter<double>("outputs.cloud.filters.range.min", 0.1);
  node_->declare_parameter<double>("outputs.cloud.filters.range.max", 30.0);
  node_->declare_parameter<bool>("outputs.cloud.filters.angular.enabled", false);
  node_->declare_parameter<bool>("outputs.cloud.filters.angular.invert", false);
  node_->declare_parameter<std::vector<double>>(
    "outputs.cloud.filters.angular.ranges",
    {0.0, 360.0});
  node_->declare_parameter<bool>("outputs.cloud.filters.box.enabled", false);
  node_->declare_parameter<double>("outputs.cloud.filters.box.x_min", -20.0);
  node_->declare_parameter<double>("outputs.cloud.filters.box.x_max", 20.0);
  node_->declare_parameter<double>("outputs.cloud.filters.box.y_min", -20.0);
  node_->declare_parameter<double>("outputs.cloud.filters.box.y_max", 20.0);
  node_->declare_parameter<double>("outputs.cloud.filters.box.z_min", -2.0);
  node_->declare_parameter<double>("outputs.cloud.filters.box.z_max", 5.0);

  // outputs.cloud.voxel
  node_->declare_parameter<bool>("outputs.cloud.voxel.enabled", false);
  node_->declare_parameter<double>("outputs.cloud.voxel.leaf_size", 0.0);
  node_->declare_parameter<double>("outputs.cloud.voxel.leaf_x", 0.0);
  node_->declare_parameter<double>("outputs.cloud.voxel.leaf_y", 0.0);
  node_->declare_parameter<double>("outputs.cloud.voxel.leaf_z", 0.0);

  // outputs.cloud.self_filter
  node_->declare_parameter<bool>("outputs.cloud.self_filter.enabled", false);
  node_->declare_parameter<std::vector<std::string>>(
    "outputs.cloud.self_filter.box_names", std::vector<std::string>{});

  // outputs.cloud.height_cap
  node_->declare_parameter<bool>("outputs.cloud.height_cap.enabled", false);
  node_->declare_parameter<double>("outputs.cloud.height_cap.z_min", -1.0);
  node_->declare_parameter<double>("outputs.cloud.height_cap.z_max", 3.0);

  // outputs.scan
  node_->declare_parameter<bool>("outputs.scan.enabled", false);
  node_->declare_parameter<std::string>("outputs.scan.topic", "~/merged_scan");
  node_->declare_parameter<std::string>("outputs.scan.qos.reliability", "reliable");
  node_->declare_parameter<std::string>("outputs.scan.qos.durability", "volatile");
  node_->declare_parameter<int>("outputs.scan.qos.history_depth", 10);
  node_->declare_parameter<std::string>("outputs.scan.qos.liveliness", "automatic");
  node_->declare_parameter<double>("outputs.scan.qos.liveliness_lease_duration_ms", 0.0);
  node_->declare_parameter<double>("outputs.scan.qos.deadline_ms", 0.0);
  node_->declare_parameter<double>("outputs.scan.qos.lifespan_ms", 0.0);
  node_->declare_parameter<double>("outputs.scan.z_min", -0.15);
  node_->declare_parameter<double>("outputs.scan.z_max", 0.15);
  node_->declare_parameter<double>("outputs.scan.angle_min", -3.14159265);
  node_->declare_parameter<double>("outputs.scan.angle_max", 3.14159265);
  node_->declare_parameter<double>("outputs.scan.angle_increment", 0.00436332);
  node_->declare_parameter<double>("outputs.scan.range_min", 0.10);
  node_->declare_parameter<double>("outputs.scan.range_max", 100.0);

  node_->declare_parameter<std::vector<std::string>>("source_names", std::vector<std::string>{});
}

// Defaults here must match the param_or() fallbacks in read_source_config()
// and load_filter_params(): preview() reads a not-yet-declared source through
// those fallbacks, and what it validates must be what a later reload() reads.
void ConfigLoader::declare_source_params(const std::string & name)
{
  std::string p = "sources." + name;
  if (node_->has_parameter(p + ".topic")) {
    return;                                         // re-added source: keep prior values
  }
  node_->declare_parameter<std::string>(p + ".topic", "");
  node_->declare_parameter<std::string>(p + ".imu_topic", "");
  node_->declare_parameter<std::string>(p + ".type", "pointcloud2");
  node_->declare_parameter<std::string>(p + ".qos_reliability", "best_effort");
  node_->declare_parameter<int>(p + ".qos_history_depth", 1);
  node_->declare_parameter<double>(p + ".expected_rate", 0.0);
  node_->declare_parameter<bool>(p + ".filters.range.enabled", false);
  node_->declare_parameter<double>(p + ".filters.range.min", 0.1);
  node_->declare_parameter<double>(p + ".filters.range.max", 100.0);
  node_->declare_parameter<bool>(p + ".filters.angular.enabled", false);
  node_->declare_parameter<bool>(p + ".filters.angular.invert", false);
  node_->declare_parameter<std::vector<double>>(p + ".filters.angular.ranges", {0.0, 360.0});
  node_->declare_parameter<bool>(p + ".filters.box.enabled", false);
  node_->declare_parameter<double>(p + ".filters.box.x_min", -20.0);
  node_->declare_parameter<double>(p + ".filters.box.x_max", 20.0);
  node_->declare_parameter<double>(p + ".filters.box.y_min", -20.0);
  node_->declare_parameter<double>(p + ".filters.box.y_max", 20.0);
  node_->declare_parameter<double>(p + ".filters.box.z_min", -2.0);
  node_->declare_parameter<double>(p + ".filters.box.z_max", 5.0);
}

OutputQosConfig ConfigLoader::load_output_qos(const std::string & prefix)
{
  OutputQosConfig qos;
  qos.reliability = param(prefix + ".reliability").as_string();
  qos.durability = param(prefix + ".durability").as_string();
  qos.history_depth = param(prefix + ".history_depth").as_int();
  qos.liveliness = param(prefix + ".liveliness").as_string();
  qos.liveliness_lease_duration_ms =
    param(prefix + ".liveliness_lease_duration_ms").as_double();
  qos.deadline_ms = param(prefix + ".deadline_ms").as_double();
  qos.lifespan_ms = param(prefix + ".lifespan_ms").as_double();
  return qos;
}

FilterParams ConfigLoader::load_filter_params(const std::string & prefix)
{
  FilterParams fp;
  fp.range_filter_enabled = param_or<bool>(prefix + ".range.enabled", false);
  fp.min_range = param_or<double>(prefix + ".range.min", 0.1);
  fp.max_range = param_or<double>(prefix + ".range.max", 100.0);
  fp.compute_squared_ranges();

  fp.angular_filter_enabled = param_or<bool>(prefix + ".angular.enabled", false);
  fp.angular_invert = param_or<bool>(prefix + ".angular.invert", false);
  auto ang_flat = param_or<std::vector<double>>(prefix + ".angular.ranges", {0.0, 360.0});
  for (size_t i = 0; i + 1 < ang_flat.size(); i += 2) {
    double lo = std::fmod(ang_flat[i], 360.0);
    double hi = std::fmod(ang_flat[i + 1], 360.0);
    if (lo < 0.0) {lo += 360.0;}
    if (hi < 0.0) {hi += 360.0;}
    fp.angular_ranges.emplace_back(lo, hi);
  }

  fp.box_filter_enabled = param_or<bool>(prefix + ".box.enabled", false);
  if (fp.box_filter_enabled) {
    fp.box_min.x() = param_or<double>(prefix + ".box.x_min", -20.0);
    fp.box_max.x() = param_or<double>(prefix + ".box.x_max", 20.0);
    fp.box_min.y() = param_or<double>(prefix + ".box.y_min", -20.0);
    fp.box_max.y() = param_or<double>(prefix + ".box.y_max", 20.0);
    fp.box_min.z() = param_or<double>(prefix + ".box.z_min", -2.0);
    fp.box_max.z() = param_or<double>(prefix + ".box.z_max", 5.0);
  }

  try {
    fp.validate();
  } catch (const std::exception & ex) {
    // Qualify with the parameter prefix so a rejected runtime set tells the
    // caller exactly which filter block was at fault.
    throw std::invalid_argument(prefix + ": " + std::string(ex.what()));
  }
  return fp;
}

MergeConfig ConfigLoader::read_common_params()
{
  MergeConfig cfg;
  cfg.output_frame_id = param("output_frame_id").as_string();
  cfg.output_rate = param("output_rate").as_double();
  cfg.source_timeout = param("source_timeout").as_double();
  cfg.source_stale_reuse_window = param("source_stale_reuse_window").as_double();
  cfg.enable_gpu = param("enable_gpu").as_bool();
  cfg.max_source_spread_warn = param("max_source_spread_warn").as_double();

  auto ts_str = param("timestamp_strategy").as_string();
  if (ts_str == "earliest") {
    cfg.timestamp_strategy = TimestampStrategy::EARLIEST;
  } else if (ts_str == "latest") {
    cfg.timestamp_strategy = TimestampStrategy::LATEST;
  } else if (ts_str == "average") {
    cfg.timestamp_strategy = TimestampStrategy::AVERAGE;
  } else if (ts_str == "local") {cfg.timestamp_strategy = TimestampStrategy::LOCAL;} else {
    throw std::runtime_error("polka: invalid timestamp_strategy '" + ts_str + "'");
  }

  cfg.point_timestamps.enabled = param("point_timestamps.enabled").as_bool();
  auto ppt_mode = param("point_timestamps.mode").as_string();
  if (ppt_mode == "offset") {
    cfg.point_timestamps.mode = PerPointTimeMode::OFFSET;
  } else if (ppt_mode == "absolute") {
    cfg.point_timestamps.mode = PerPointTimeMode::ABSOLUTE;
  } else {throw std::runtime_error("polka: invalid point_timestamps.mode '" + ppt_mode + "'");}
  cfg.suppress_duplicate_timestamps =
    param("suppress_duplicate_timestamps").as_bool();

  cfg.motion_compensation.enabled =
    param("motion_compensation.enabled").as_bool();
  cfg.motion_compensation.imu_topic =
    param("motion_compensation.imu_topic").as_string();
  cfg.motion_compensation.max_imu_age =
    param("motion_compensation.max_imu_age").as_double();
  cfg.motion_compensation.imu_buffer_size =
    param("motion_compensation.imu_buffer_size").as_int();
  cfg.motion_compensation.per_point_deskew =
    param("motion_compensation.per_point_deskew").as_bool();
  cfg.motion_compensation.deskew_timestamp_field =
    param("motion_compensation.deskew_timestamp_field").as_string();
  cfg.motion_compensation.imu_frame =
    param("motion_compensation.imu_frame").as_string();

  cfg.diagnostics.enabled = param("diagnostics.enabled").as_bool();
  cfg.diagnostics.publish_period_sec = param("diagnostics.publish_period_sec").as_double();
  cfg.diagnostics.timing_threshold_sec =
    param("diagnostics.timing_drift.threshold_sec").as_double();
  cfg.diagnostics.timing_ewma_alpha =
    param("diagnostics.timing_drift.ewma_alpha").as_double();
  cfg.diagnostics.timing_min_ticks =
    static_cast<int>(param("diagnostics.timing_drift.min_ticks").as_int());
  cfg.diagnostics.rate_sag_pct = param("diagnostics.rate_drift.sag_pct").as_double();
  cfg.diagnostics.rate_min_ticks =
    static_cast<int>(param("diagnostics.rate_drift.min_ticks").as_int());
  cfg.diagnostics.rate_baseline_sec =
    param("diagnostics.rate_drift.baseline_sec").as_double();

  cfg.cloud_output.enabled = param("outputs.cloud.enabled").as_bool();
  cfg.cloud_output.topic = param("outputs.cloud.topic").as_string();
  cfg.cloud_output.qos = load_output_qos("outputs.cloud.qos");
  cfg.cloud_output.filters = load_filter_params("outputs.cloud.filters");

  {
    double uniform = param("outputs.cloud.voxel.leaf_size").as_double();
    double lx = param("outputs.cloud.voxel.leaf_x").as_double();
    double ly = param("outputs.cloud.voxel.leaf_y").as_double();
    double lz = param("outputs.cloud.voxel.leaf_z").as_double();
    if (uniform > 0.0 && lx == 0.0 && ly == 0.0 && lz == 0.0) {
      lx = ly = lz = uniform;
    }
    cfg.cloud_output.voxel.leaf_x = static_cast<float>(lx);
    cfg.cloud_output.voxel.leaf_y = static_cast<float>(ly);
    cfg.cloud_output.voxel.leaf_z = static_cast<float>(lz);
    cfg.cloud_output.voxel.enabled =
      param("outputs.cloud.voxel.enabled").as_bool() ||
      (lx > 0.0 && ly > 0.0 && lz > 0.0);
  }

  cfg.cloud_output.height_cap.enabled =
    param("outputs.cloud.height_cap.enabled").as_bool();
  cfg.cloud_output.height_cap.z_min =
    param("outputs.cloud.height_cap.z_min").as_double();
  cfg.cloud_output.height_cap.z_max =
    param("outputs.cloud.height_cap.z_max").as_double();

  cfg.scan_output.enabled = param("outputs.scan.enabled").as_bool();
  cfg.scan_output.topic = param("outputs.scan.topic").as_string();
  cfg.scan_output.qos = load_output_qos("outputs.scan.qos");
  cfg.scan_output.flatten.z_min = param("outputs.scan.z_min").as_double();
  cfg.scan_output.flatten.z_max = param("outputs.scan.z_max").as_double();
  cfg.scan_output.flatten.angle_min = param("outputs.scan.angle_min").as_double();
  cfg.scan_output.flatten.angle_max = param("outputs.scan.angle_max").as_double();
  cfg.scan_output.flatten.angle_increment = param("outputs.scan.angle_increment").as_double();
  cfg.scan_output.flatten.range_min = param("outputs.scan.range_min").as_double();
  cfg.scan_output.flatten.range_max = param("outputs.scan.range_max").as_double();
  cfg.scan_output.flatten.compute_bins();
  try {
    cfg.scan_output.flatten.validate();
  } catch (const std::exception & ex) {
    throw std::invalid_argument("outputs.scan: " + std::string(ex.what()));
  }

  return cfg;
}

SourceConfig ConfigLoader::read_source_config(const std::string & name)
{
  std::string p = "sources." + name;
  SourceConfig sc;
  sc.name = name;
  sc.topic = param_or<std::string>(p + ".topic", "");
  sc.imu_topic = param_or<std::string>(p + ".imu_topic", "");
  auto type_str = param_or<std::string>(p + ".type", "pointcloud2");
  sc.type = (type_str == "laserscan") ? SourceType::LASERSCAN : SourceType::POINTCLOUD2;
  sc.qos_reliability = param_or<std::string>(p + ".qos_reliability", "best_effort");
  sc.qos_history_depth = param_or<int>(p + ".qos_history_depth", 1);
  sc.expected_rate = param_or<double>(p + ".expected_rate", 0.0);
  sc.filter_params = load_filter_params(p + ".filters");
  return sc;
}

MergeConfig ConfigLoader::read_config(
  const std::vector<std::string> & source_names, bool allow_pending)
{
  const bool may_declare = (overlay_ == nullptr);  // preview must not mutate node state
  if (may_declare) {
    for (const auto & name : source_names) {
      declare_source_params(name);
    }
  }

  MergeConfig cfg = read_common_params();
  cfg.cloud_output.self_filter =
    load_self_filter_config("outputs.cloud.self_filter", may_declare);

  for (const auto & name : source_names) {
    cfg.sources.push_back(read_source_config(name));
  }

  validate(cfg, allow_pending);
  return cfg;
}

MergeConfig ConfigLoader::load()
{
  auto source_names = node_->get_parameter("source_names").as_string_array();
  return read_config(source_names, /*allow_pending=*/ false);
  // Summary printed via PolkaNode startup banner once construction completes.
}

MergeConfig ConfigLoader::reload(const std::vector<std::string> & source_names)
{
  return read_config(source_names, /*allow_pending=*/ true);
}

MergeConfig ConfigLoader::preview(
  const std::vector<rclcpp::Parameter> & proposed,
  const std::vector<std::string> & source_names)
{
  overlay_ = &proposed;
  try {
    MergeConfig cfg = read_config(source_names, /*allow_pending=*/ true);
    overlay_ = nullptr;
    return cfg;
  } catch (...) {
    overlay_ = nullptr;
    throw;
  }
}

SelfFilterConfig ConfigLoader::load_self_filter_config(
  const std::string & prefix, bool declare_missing)
{
  SelfFilterConfig sf;
  sf.enabled = param(prefix + ".enabled").as_bool();
  if (!sf.enabled) {return sf;}

  auto box_names = param(prefix + ".box_names").as_string_array();
  for (const auto & name : box_names) {
    std::string bp = prefix + "." + name;
    if (declare_missing && !node_->has_parameter(bp + ".x_min")) {
      node_->declare_parameter<double>(bp + ".x_min", 0.0);
      node_->declare_parameter<double>(bp + ".x_max", 0.0);
      node_->declare_parameter<double>(bp + ".y_min", 0.0);
      node_->declare_parameter<double>(bp + ".y_max", 0.0);
      node_->declare_parameter<double>(bp + ".z_min", 0.0);
      node_->declare_parameter<double>(bp + ".z_max", 0.0);
    }

    ExclusionBox box;
    box.label = name;
    box.min.x() = param_or<double>(bp + ".x_min", 0.0);
    box.max.x() = param_or<double>(bp + ".x_max", 0.0);
    box.min.y() = param_or<double>(bp + ".y_min", 0.0);
    box.max.y() = param_or<double>(bp + ".y_max", 0.0);
    box.min.z() = param_or<double>(bp + ".z_min", 0.0);
    box.max.z() = param_or<double>(bp + ".z_max", 0.0);
    sf.boxes.push_back(box);
  }
  return sf;
}

void ConfigLoader::validate(MergeConfig & config, bool allow_pending)
{
  if (config.sources.empty()) {
    throw std::runtime_error("polka: source_names is empty");
  }
  // -1 ("adaptive") was accepted here but never implemented: PolkaNode only creates
  // output_timer_ when output_rate > 0.0, so -1 silently disabled all output with no
  // warning. Fail loudly instead of shipping a node that never publishes.
  if (config.output_rate <= 0.0) {
    throw std::runtime_error(
            "polka: output_rate must be > 0 (adaptive/-1 mode is not implemented)");
  }
  if (config.source_timeout <= 0.0) {
    throw std::runtime_error("polka: source_timeout must be > 0");
  }
  if (config.source_stale_reuse_window < config.source_timeout) {
    RCLCPP_WARN(
      logger_,
      "polka: source_stale_reuse_window (%.3f) < source_timeout (%.3f); "
      "raising it to source_timeout",
      config.source_stale_reuse_window, config.source_timeout);
    config.source_stale_reuse_window = config.source_timeout;
  }
  if (!config.cloud_output.enabled && !config.scan_output.enabled) {
    throw std::runtime_error("polka: at least one output (cloud or scan) must be enabled");
  }
  if (config.cloud_output.voxel.enabled) {
    if (config.cloud_output.voxel.leaf_x <= 0.0f ||
      config.cloud_output.voxel.leaf_y <= 0.0f ||
      config.cloud_output.voxel.leaf_z <= 0.0f)
    {
      throw std::runtime_error(
              "polka: voxel filter enabled but leaf size is <= 0 (would crash pcl::VoxelGrid)");
    }
  }
  try {
    config.diagnostics.validate();
  } catch (const std::exception & ex) {
    throw std::invalid_argument("diagnostics: " + std::string(ex.what()));
  }
  std::set<std::string> seen;
  for (const auto & src : config.sources) {
    if (src.name.empty()) {
      throw std::runtime_error("polka: source names must be non-empty");
    }
    if (!seen.insert(src.name).second) {
      throw std::runtime_error("polka: duplicate source name '" + src.name + "'");
    }
    if (src.expected_rate < 0.0) {
      throw std::runtime_error(
              "polka: sources." + src.name + ".expected_rate must be >= 0");
    }
    // A pending source (empty topic) is legal at runtime - it activates once
    // its topic is set - but a startup config demands complete sources.
    if (!allow_pending && src.topic.empty()) {
      throw std::runtime_error("polka: source '" + src.name + "' has empty topic");
    }
  }
}

}  // namespace polka
