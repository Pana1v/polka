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

#include "polka/config_loader.hpp"
#include <stdexcept>
#include <cmath>

namespace polka {

ConfigLoader::ConfigLoader(rclcpp::Node * node)
: node_(node), logger_(node->get_logger())
{
  declare_defaults();
}

void ConfigLoader::declare_defaults()
{
  node_->declare_parameter<std::string>("output_frame_id", "base_link");
  node_->declare_parameter<double>("output_rate", 20.0);
  node_->declare_parameter<double>("source_timeout", 0.5);
  node_->declare_parameter<bool>("enable_gpu", true);
  node_->declare_parameter<std::string>("timestamp_strategy", "earliest");
  node_->declare_parameter<double>("max_source_spread_warn", 0.05);

  // motion compensation (IMU-based deskewing)
  node_->declare_parameter<bool>("motion_compensation.enabled", false);
  node_->declare_parameter<std::string>("motion_compensation.imu_topic", "");
  node_->declare_parameter<double>("motion_compensation.max_imu_age", 0.2);
  node_->declare_parameter<int>("motion_compensation.imu_buffer_size", 200);
  node_->declare_parameter<bool>("motion_compensation.per_point_deskew", true);
  node_->declare_parameter<std::string>("motion_compensation.deskew_timestamp_field", "auto");
  node_->declare_parameter<std::string>("motion_compensation.imu_frame", "");

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
  node_->declare_parameter<std::vector<double>>("outputs.cloud.filters.angular.ranges", {0.0, 360.0});
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

OutputQosConfig ConfigLoader::load_output_qos(const std::string & prefix)
{
  OutputQosConfig qos;
  qos.reliability = node_->get_parameter(prefix + ".reliability").as_string();
  qos.durability = node_->get_parameter(prefix + ".durability").as_string();
  qos.history_depth = node_->get_parameter(prefix + ".history_depth").as_int();
  qos.liveliness = node_->get_parameter(prefix + ".liveliness").as_string();
  qos.liveliness_lease_duration_ms =
    node_->get_parameter(prefix + ".liveliness_lease_duration_ms").as_double();
  qos.deadline_ms = node_->get_parameter(prefix + ".deadline_ms").as_double();
  qos.lifespan_ms = node_->get_parameter(prefix + ".lifespan_ms").as_double();
  return qos;
}

FilterParams ConfigLoader::load_filter_params(const std::string & prefix)
{
  FilterParams fp;
  fp.range_filter_enabled = node_->get_parameter(prefix + ".range.enabled").as_bool();
  fp.min_range = node_->get_parameter(prefix + ".range.min").as_double();
  fp.max_range = node_->get_parameter(prefix + ".range.max").as_double();
  fp.compute_squared_ranges();

  fp.angular_filter_enabled = node_->get_parameter(prefix + ".angular.enabled").as_bool();
  fp.angular_invert = node_->get_parameter(prefix + ".angular.invert").as_bool();
  auto ang_flat = node_->get_parameter(prefix + ".angular.ranges").as_double_array();
  for (size_t i = 0; i + 1 < ang_flat.size(); i += 2) {
    double lo = std::fmod(ang_flat[i], 360.0);
    double hi = std::fmod(ang_flat[i + 1], 360.0);
    if (lo < 0.0) lo += 360.0;
    if (hi < 0.0) hi += 360.0;
    fp.angular_ranges.emplace_back(lo, hi);
  }

  fp.box_filter_enabled = node_->get_parameter(prefix + ".box.enabled").as_bool();
  if (fp.box_filter_enabled) {
    fp.box_min.x() = node_->get_parameter(prefix + ".box.x_min").as_double();
    fp.box_max.x() = node_->get_parameter(prefix + ".box.x_max").as_double();
    fp.box_min.y() = node_->get_parameter(prefix + ".box.y_min").as_double();
    fp.box_max.y() = node_->get_parameter(prefix + ".box.y_max").as_double();
    fp.box_min.z() = node_->get_parameter(prefix + ".box.z_min").as_double();
    fp.box_max.z() = node_->get_parameter(prefix + ".box.z_max").as_double();
  }

  fp.validate();
  return fp;
}

MergeConfig ConfigLoader::read_common_params()
{
  MergeConfig cfg;
  cfg.output_frame_id = node_->get_parameter("output_frame_id").as_string();
  cfg.output_rate = node_->get_parameter("output_rate").as_double();
  cfg.source_timeout = node_->get_parameter("source_timeout").as_double();
  cfg.enable_gpu = node_->get_parameter("enable_gpu").as_bool();
  cfg.max_source_spread_warn = node_->get_parameter("max_source_spread_warn").as_double();

  auto ts_str = node_->get_parameter("timestamp_strategy").as_string();
  if (ts_str == "earliest") cfg.timestamp_strategy = TimestampStrategy::EARLIEST;
  else if (ts_str == "latest") cfg.timestamp_strategy = TimestampStrategy::LATEST;
  else if (ts_str == "average") cfg.timestamp_strategy = TimestampStrategy::AVERAGE;
  else if (ts_str == "local") cfg.timestamp_strategy = TimestampStrategy::LOCAL;
  else throw std::runtime_error("polka: invalid timestamp_strategy '" + ts_str + "'");

  cfg.motion_compensation.enabled =
    node_->get_parameter("motion_compensation.enabled").as_bool();
  cfg.motion_compensation.imu_topic =
    node_->get_parameter("motion_compensation.imu_topic").as_string();
  cfg.motion_compensation.max_imu_age =
    node_->get_parameter("motion_compensation.max_imu_age").as_double();
  cfg.motion_compensation.imu_buffer_size =
    node_->get_parameter("motion_compensation.imu_buffer_size").as_int();
  cfg.motion_compensation.per_point_deskew =
    node_->get_parameter("motion_compensation.per_point_deskew").as_bool();
  cfg.motion_compensation.deskew_timestamp_field =
    node_->get_parameter("motion_compensation.deskew_timestamp_field").as_string();
  cfg.motion_compensation.imu_frame =
    node_->get_parameter("motion_compensation.imu_frame").as_string();

  cfg.cloud_output.enabled = node_->get_parameter("outputs.cloud.enabled").as_bool();
  cfg.cloud_output.topic = node_->get_parameter("outputs.cloud.topic").as_string();
  cfg.cloud_output.qos = load_output_qos("outputs.cloud.qos");
  cfg.cloud_output.filters = load_filter_params("outputs.cloud.filters");

  {
    double uniform = node_->get_parameter("outputs.cloud.voxel.leaf_size").as_double();
    double lx = node_->get_parameter("outputs.cloud.voxel.leaf_x").as_double();
    double ly = node_->get_parameter("outputs.cloud.voxel.leaf_y").as_double();
    double lz = node_->get_parameter("outputs.cloud.voxel.leaf_z").as_double();
    if (uniform > 0.0 && lx == 0.0 && ly == 0.0 && lz == 0.0)
      lx = ly = lz = uniform;
    cfg.cloud_output.voxel.leaf_x = static_cast<float>(lx);
    cfg.cloud_output.voxel.leaf_y = static_cast<float>(ly);
    cfg.cloud_output.voxel.leaf_z = static_cast<float>(lz);
    cfg.cloud_output.voxel.enabled =
      node_->get_parameter("outputs.cloud.voxel.enabled").as_bool() ||
      (lx > 0.0 && ly > 0.0 && lz > 0.0);
  }

  cfg.cloud_output.height_cap.enabled =
    node_->get_parameter("outputs.cloud.height_cap.enabled").as_bool();
  cfg.cloud_output.height_cap.z_min =
    node_->get_parameter("outputs.cloud.height_cap.z_min").as_double();
  cfg.cloud_output.height_cap.z_max =
    node_->get_parameter("outputs.cloud.height_cap.z_max").as_double();

  cfg.scan_output.enabled = node_->get_parameter("outputs.scan.enabled").as_bool();
  cfg.scan_output.topic = node_->get_parameter("outputs.scan.topic").as_string();
  cfg.scan_output.qos = load_output_qos("outputs.scan.qos");
  cfg.scan_output.flatten.z_min = node_->get_parameter("outputs.scan.z_min").as_double();
  cfg.scan_output.flatten.z_max = node_->get_parameter("outputs.scan.z_max").as_double();
  cfg.scan_output.flatten.angle_min = node_->get_parameter("outputs.scan.angle_min").as_double();
  cfg.scan_output.flatten.angle_max = node_->get_parameter("outputs.scan.angle_max").as_double();
  cfg.scan_output.flatten.angle_increment = node_->get_parameter("outputs.scan.angle_increment").as_double();
  cfg.scan_output.flatten.range_min = node_->get_parameter("outputs.scan.range_min").as_double();
  cfg.scan_output.flatten.range_max = node_->get_parameter("outputs.scan.range_max").as_double();
  cfg.scan_output.flatten.compute_bins();
  cfg.scan_output.flatten.validate();

  return cfg;
}

MergeConfig ConfigLoader::load()
{
  MergeConfig cfg = read_common_params();

  cfg.cloud_output.self_filter = load_self_filter_config("outputs.cloud.self_filter");

  auto source_names = node_->get_parameter("source_names").as_string_array();
  for (const auto & name : source_names) {
    std::string p = "sources." + name;

    node_->declare_parameter<std::string>(p + ".topic", "");
    node_->declare_parameter<std::string>(p + ".imu_topic", "");
    node_->declare_parameter<std::string>(p + ".type", "pointcloud2");
    node_->declare_parameter<std::string>(p + ".qos_reliability", "best_effort");
    node_->declare_parameter<int>(p + ".qos_history_depth", 1);
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

    SourceConfig sc;
    sc.name = name;
    sc.topic = node_->get_parameter(p + ".topic").as_string();
    sc.imu_topic = node_->get_parameter(p + ".imu_topic").as_string();
    auto type_str = node_->get_parameter(p + ".type").as_string();
    sc.type = (type_str == "laserscan") ? SourceType::LASERSCAN : SourceType::POINTCLOUD2;
    sc.qos_reliability = node_->get_parameter(p + ".qos_reliability").as_string();
    sc.qos_history_depth = node_->get_parameter(p + ".qos_history_depth").as_int();
    sc.filter_params = load_filter_params(p + ".filters");
    cfg.sources.push_back(std::move(sc));
  }

  validate(cfg);
  RCLCPP_INFO(logger_, "loaded %zu sources, frame='%s', rate=%.1f Hz",
    cfg.sources.size(), cfg.output_frame_id.c_str(), cfg.output_rate);
  return cfg;
}

MergeConfig ConfigLoader::reload(const std::vector<std::string> & source_names)
{
  MergeConfig cfg = read_common_params();

  cfg.cloud_output.self_filter.enabled =
    node_->get_parameter("outputs.cloud.self_filter.enabled").as_bool();
  if (cfg.cloud_output.self_filter.enabled) {
    auto box_names = node_->get_parameter("outputs.cloud.self_filter.box_names").as_string_array();
    for (const auto & name : box_names) {
      std::string bp = "outputs.cloud.self_filter." + name;
      ExclusionBox box;
      box.label = name;
      box.min.x() = node_->get_parameter(bp + ".x_min").as_double();
      box.max.x() = node_->get_parameter(bp + ".x_max").as_double();
      box.min.y() = node_->get_parameter(bp + ".y_min").as_double();
      box.max.y() = node_->get_parameter(bp + ".y_max").as_double();
      box.min.z() = node_->get_parameter(bp + ".z_min").as_double();
      box.max.z() = node_->get_parameter(bp + ".z_max").as_double();
      cfg.cloud_output.self_filter.boxes.push_back(box);
    }
  }

  for (const auto & name : source_names) {
    std::string p = "sources." + name;
    SourceConfig sc;
    sc.name = name;
    sc.topic = node_->get_parameter(p + ".topic").as_string();
    sc.imu_topic = node_->get_parameter(p + ".imu_topic").as_string();
    auto type_str = node_->get_parameter(p + ".type").as_string();
    sc.type = (type_str == "laserscan") ? SourceType::LASERSCAN : SourceType::POINTCLOUD2;
    sc.qos_reliability = node_->get_parameter(p + ".qos_reliability").as_string();
    sc.qos_history_depth = node_->get_parameter(p + ".qos_history_depth").as_int();
    sc.filter_params = load_filter_params(p + ".filters");
    cfg.sources.push_back(std::move(sc));
  }

  validate(cfg);
  return cfg;
}

SelfFilterConfig ConfigLoader::load_self_filter_config(const std::string & prefix)
{
  SelfFilterConfig sf;
  sf.enabled = node_->get_parameter(prefix + ".enabled").as_bool();
  if (!sf.enabled) return sf;

  auto box_names = node_->get_parameter(prefix + ".box_names").as_string_array();
  for (const auto & name : box_names) {
    std::string bp = prefix + "." + name;
    node_->declare_parameter<double>(bp + ".x_min", 0.0);
    node_->declare_parameter<double>(bp + ".x_max", 0.0);
    node_->declare_parameter<double>(bp + ".y_min", 0.0);
    node_->declare_parameter<double>(bp + ".y_max", 0.0);
    node_->declare_parameter<double>(bp + ".z_min", 0.0);
    node_->declare_parameter<double>(bp + ".z_max", 0.0);

    ExclusionBox box;
    box.label = name;
    box.min.x() = node_->get_parameter(bp + ".x_min").as_double();
    box.max.x() = node_->get_parameter(bp + ".x_max").as_double();
    box.min.y() = node_->get_parameter(bp + ".y_min").as_double();
    box.max.y() = node_->get_parameter(bp + ".y_max").as_double();
    box.min.z() = node_->get_parameter(bp + ".z_min").as_double();
    box.max.z() = node_->get_parameter(bp + ".z_max").as_double();
    sf.boxes.push_back(box);
  }
  return sf;
}

void ConfigLoader::validate(const MergeConfig & config)
{
  if (config.sources.empty())
    throw std::runtime_error("polka: source_names is empty");
  if (config.output_rate <= 0.0 && config.output_rate != -1.0)
    throw std::runtime_error("polka: output_rate must be > 0 or -1 (adaptive)");
  if (config.source_timeout <= 0.0)
    throw std::runtime_error("polka: source_timeout must be > 0");
  if (!config.cloud_output.enabled && !config.scan_output.enabled)
    throw std::runtime_error("polka: at least one output (cloud or scan) must be enabled");
  if (config.cloud_output.voxel.enabled) {
    if (config.cloud_output.voxel.leaf_x <= 0.0f ||
        config.cloud_output.voxel.leaf_y <= 0.0f ||
        config.cloud_output.voxel.leaf_z <= 0.0f)
      throw std::runtime_error(
        "polka: voxel filter enabled but leaf size is <= 0 (would crash pcl::VoxelGrid)");
  }
  for (const auto & src : config.sources) {
    if (src.topic.empty())
      throw std::runtime_error("polka: source '" + src.name + "' has empty topic");
  }
}

}  // namespace polka
