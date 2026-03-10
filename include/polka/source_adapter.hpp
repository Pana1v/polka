#ifndef POLKA__SOURCE_ADAPTER_HPP
#define POLKA__SOURCE_ADAPTER_HPP

#include "polka/types.hpp"
#include "polka/filters/i_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>

#include <memory>
#include <string>
#include <vector>
#include <atomic>

namespace polka {

class SourceAdapter {
public:
  SourceAdapter(rclcpp::Node * node, const SourceConfig & config);

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

  std::vector<std::unique_ptr<IFilter>> filters_;
};

}  // namespace polka

#endif  // POLKA__SOURCE_ADAPTER_HPP
