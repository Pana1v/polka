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

#include "polka/input/gyro_bias.hpp"

namespace polka
{

namespace
{

// Length of the stillness test. Long enough to average gyro noise well below the
// bias, short enough to catch a brief stop.
constexpr double kWindowSec = 1.0;
// A window must span this share of kWindowSec, so start-up or a dropout
// cannot pass a handful of samples as a still second.
constexpr double kMinCoverage = 0.9;
constexpr size_t kMinSamples = 20;
// Largest per-axis standard deviation of a still gyro. Parked Airy IMUs spread
// 0.7 mrad/s; a moving vehicle, far more.
constexpr double kMaxStillSpreadRadS = 2e-3;
// Largest believable bias. Anything steadier and faster is a turn.
constexpr double kMaxBiasRadS = 35e-3;

}  // namespace

void GyroBias::add(const rclcpp::Time & stamp, const Eigen::Vector3d & gyro)
{
  window_.push_back({stamp, gyro});
  const rclcpp::Time oldest_kept = stamp - rclcpp::Duration::from_seconds(kWindowSec);
  while (!window_.empty() && window_.front().stamp < oldest_kept) {
    window_.pop_front();
  }

  // Only judge full, dense windows.
  const double span = (window_.back().stamp - window_.front().stamp).seconds();
  if (span < kMinCoverage * kWindowSec || window_.size() < kMinSamples) {return;}

  // Still when no axis spreads and the mean is a plausible bias.
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & s : window_) {
    mean += s.gyro;
  }
  mean /= static_cast<double>(window_.size());

  Eigen::Vector3d var = Eigen::Vector3d::Zero();
  for (const auto & s : window_) {
    var += (s.gyro - mean).cwiseAbs2();
  }
  var /= static_cast<double>(window_.size());

  const bool spread_ok = (var.array() < kMaxStillSpreadRadS * kMaxStillSpreadRadS).all();
  if (!spread_ok || mean.norm() >= kMaxBiasRadS) {return;}

  bias_ = mean;
  learned_ = true;
}

}  // namespace polka
