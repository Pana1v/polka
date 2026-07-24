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

#ifndef POLKA__UTIL__QOS_BUILDER_HPP_
#define POLKA__UTIL__QOS_BUILDER_HPP_

#include <chrono>

#include "polka/types.hpp"
#include <rclcpp/rclcpp.hpp>

namespace polka
{

inline rclcpp::QoS build_qos(const OutputQosConfig & cfg)
{
  rclcpp::QoS qos(cfg.history_depth);

  if (cfg.reliability == "best_effort") {
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  } else {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  }

  if (cfg.durability == "transient_local") {
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  } else {
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
  }

  if (cfg.liveliness == "manual_by_topic") {
    qos.liveliness(rclcpp::LivelinessPolicy::ManualByTopic);
  } else {
    qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
  }

  if (cfg.liveliness_lease_duration_ms > 0.0) {
    qos.liveliness_lease_duration(
      std::chrono::milliseconds(static_cast<int64_t>(cfg.liveliness_lease_duration_ms)));
  }

  if (cfg.deadline_ms > 0.0) {
    qos.deadline(
      std::chrono::milliseconds(static_cast<int64_t>(cfg.deadline_ms)));
  }

  if (cfg.lifespan_ms > 0.0) {
    qos.lifespan(
      std::chrono::milliseconds(static_cast<int64_t>(cfg.lifespan_ms)));
  }

  return qos;
}

}  // namespace polka

#endif  // POLKA__UTIL__QOS_BUILDER_HPP_
