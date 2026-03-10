#include "polka/source_adapter.hpp"
#include "polka/filters/range_filter.hpp"
#include "polka/filters/angular_filter.hpp"
#include "polka/filters/box_filter.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_field.hpp>

namespace polka {

SourceAdapter::SourceAdapter(rclcpp::Node * node, const SourceConfig & config)
: node_(node), config_(config), logger_(node->get_logger())
{
  // Build QoS
  rclcpp::QoS qos(config.qos_history_depth);
  if (config.qos_reliability == "best_effort") {
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  } else {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  }

  // Create subscription based on source type
  if (config.type == SourceType::POINTCLOUD2) {
    pc2_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.topic, qos,
      std::bind(&SourceAdapter::pc2_callback, this, std::placeholders::_1));
  } else {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      config.topic, qos,
      std::bind(&SourceAdapter::scan_callback, this, std::placeholders::_1));
  }

  // Build per-source filter pipeline
  const auto & fp = config.filter_params;
  if (fp.range_filter_enabled) {
    filters_.push_back(std::make_unique<RangeFilter>(fp.min_range, fp.max_range));
  }
  if (fp.angular_filter_enabled && !fp.angular_ranges.empty()) {
    filters_.push_back(
      std::make_unique<AngularFilter>(fp.angular_ranges, fp.angular_invert));
  }
  if (fp.box_filter_enabled) {
    filters_.push_back(std::make_unique<BoxFilter>(fp.box_min, fp.box_max));
  }

  RCLCPP_INFO(logger_, "polka: source '%s' subscribed to '%s' (%s), %zu filters",
    config.name.c_str(), config.topic.c_str(),
    config.type == SourceType::POINTCLOUD2 ? "PointCloud2" : "LaserScan",
    filters_.size());
}

bool SourceAdapter::validate_fields(const sensor_msgs::msg::PointCloud2 & msg)
{
  bool has_x = false, has_y = false, has_z = false;
  for (const auto & field : msg.fields) {
    if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) has_x = true;
    if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) has_y = true;
    if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) has_z = true;
  }
  if (!has_x || !has_y || !has_z) {
    RCLCPP_ERROR(logger_,
      "polka: source '%s' missing required FLOAT32 x/y/z fields — dropping all messages",
      config_.name.c_str());
    return false;
  }
  return true;
}

void SourceAdapter::apply_filters(CloudT & cloud)
{
  for (auto & filter : filters_) {
    filter->apply(cloud, frame_id_);
  }
}

void SourceAdapter::store_cloud(CloudT::Ptr cloud, const std_msgs::msg::Header & header)
{
  frame_id_ = header.frame_id;
  std::atomic_store(&buffer_, std::static_pointer_cast<CloudT>(cloud));
  last_received_time_ = rclcpp::Time(header.stamp);
  has_received_.store(true);
  message_counter_.fetch_add(1);
}

void SourceAdapter::pc2_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Validate fields on first message
  if (!fields_validated_) {
    fields_validated_ = true;
    fields_valid_ = validate_fields(*msg);
  }
  if (!fields_valid_) return;

  auto cloud = std::make_shared<CloudT>();
  pcl::fromROSMsg(*msg, *cloud);
  apply_filters(*cloud);
  store_cloud(cloud, msg->header);
}

void SourceAdapter::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  // Convert LaserScan -> PointCloud2 -> CloudT
  sensor_msgs::msg::PointCloud2 pc2_msg;
  projector_.projectLaser(*msg, pc2_msg);

  if (!fields_validated_) {
    fields_validated_ = true;
    fields_valid_ = validate_fields(pc2_msg);
  }
  if (!fields_valid_) return;

  auto cloud = std::make_shared<CloudT>();
  pcl::fromROSMsg(pc2_msg, *cloud);
  apply_filters(*cloud);
  store_cloud(cloud, msg->header);
}

CloudT::ConstPtr SourceAdapter::get_latest() const
{
  return std::atomic_load(&buffer_);
}

bool SourceAdapter::is_stale(double timeout_sec, const rclcpp::Time & now) const
{
  if (!has_received_.load()) return true;
  return (now - last_received_time_).seconds() > timeout_sec;
}

rclcpp::Time SourceAdapter::last_stamp() const
{
  return last_received_time_;
}

void SourceAdapter::rebuild_filters(const FilterParams & fp)
{
  config_.filter_params = fp;
  filters_.clear();
  if (fp.range_filter_enabled)
    filters_.push_back(std::make_unique<RangeFilter>(fp.min_range, fp.max_range));
  if (fp.angular_filter_enabled && !fp.angular_ranges.empty())
    filters_.push_back(std::make_unique<AngularFilter>(fp.angular_ranges, fp.angular_invert));
  if (fp.box_filter_enabled)
    filters_.push_back(std::make_unique<BoxFilter>(fp.box_min, fp.box_max));
}

}  // namespace polka