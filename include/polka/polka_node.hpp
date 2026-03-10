#ifndef POLKA__POLKA_NODE_HPP_
#define POLKA__POLKA_NODE_HPP_

#include "polka/types.hpp"
#include "polka/config_loader.hpp"
#include "polka/source_adapter.hpp"
#include "polka/merge_engine/i_merge_engine.hpp"
#include "polka/filters/i_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
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
  rclcpp::Time compute_output_stamp(const std::vector<rclcpp::Time> & stamps);
  void build_output_filters();
  void reconfigure();
  void voxel_downsample(CloudT & cloud);
  void height_cap(CloudT & cloud);

  // Motion compensation
  void setup_velocity_subscriber();
  void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void twist_callback(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  Eigen::Isometry3d compute_velocity_delta(double dt) const;

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

  // Velocity-based motion compensation
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

  struct CachedVelocity {
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double wx = 0.0, wy = 0.0, wz = 0.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid = false;
  };
  CachedVelocity cached_velocity_;
  mutable std::mutex velocity_mutex_;
};

}  // namespace polka

#endif  // POLKA__POLKA_NODE_HPP_
