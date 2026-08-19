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

#ifndef POLKA__UTIL__CLOUD_TRANSPORT_HPP_
#define POLKA__UTIL__CLOUD_TRANSPORT_HPP_

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef POLKA_POINT_CLOUD_TRANSPORT_ENABLED
#include <point_cloud_transport/point_cloud_transport.hpp>
#endif

namespace polka
{

// Uses point_cloud_transport where available (not released for Iron, see
// MAINTAINING.md); "raw" transport is wire-compatible with plain PointCloud2.
#ifdef POLKA_POINT_CLOUD_TRANSPORT_ENABLED
using CloudPublisher = point_cloud_transport::Publisher;
using CloudSubscriber = point_cloud_transport::Subscriber;
#else
using CloudPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
using CloudSubscriber = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr;
#endif

inline CloudPublisher create_cloud_publisher(
  rclcpp::Node * node, const std::string & topic, const rclcpp::QoS & qos)
{
#ifdef POLKA_POINT_CLOUD_TRANSPORT_ENABLED
  // Non-owning alias: called before shared_from_this() is valid in the ctor.
  auto node_alias = std::shared_ptr<rclcpp::Node>(node, [](rclcpp::Node *) {});
  return point_cloud_transport::create_publisher(node_alias, topic, qos.get_rmw_qos_profile());
#else
  return node->create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);
#endif
}

inline CloudSubscriber create_cloud_subscription(
  rclcpp::Node * node, const std::string & topic, const rclcpp::QoS & qos,
  std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)> callback)
{
#ifdef POLKA_POINT_CLOUD_TRANSPORT_ENABLED
  auto node_alias = std::shared_ptr<rclcpp::Node>(node, [](rclcpp::Node *) {});
  return point_cloud_transport::create_subscription(
    node_alias, topic, callback, "raw", qos.get_rmw_qos_profile());
#else
  return node->create_subscription<sensor_msgs::msg::PointCloud2>(topic, qos, callback);
#endif
}

inline void shutdown_cloud_publisher(CloudPublisher & pub)
{
#ifdef POLKA_POINT_CLOUD_TRANSPORT_ENABLED
  pub.shutdown();
  pub = CloudPublisher();
#else
  pub.reset();
#endif
}

inline std::string cloud_publisher_topic(const CloudPublisher & pub)
{
#ifdef POLKA_POINT_CLOUD_TRANSPORT_ENABLED
  return pub.getTopic();
#else
  return pub->get_topic_name();
#endif
}

inline void publish_cloud(const CloudPublisher & pub, const sensor_msgs::msg::PointCloud2 & msg)
{
#ifdef POLKA_POINT_CLOUD_TRANSPORT_ENABLED
  pub.publish(msg);
#else
  pub->publish(msg);
#endif
}

}  // namespace polka

#endif  // POLKA__UTIL__CLOUD_TRANSPORT_HPP_
