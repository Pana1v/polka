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

#include <gtest/gtest.h>
#include <Eigen/Core>

#include <random>

#include "polka/input/gyro_bias.hpp"

namespace polka
{

namespace
{

constexpr double kRate = 200.0;               // Hz, a RoboSense Airy's built-in IMU
const Eigen::Vector3d kBias(0.0136, 0.0151, 0.0143);   // rad/s, measured on one
const rclcpp::Time kStart(1700000000, 0, RCL_ROS_TIME);

// Feeds 'seconds' of gyro at kRate: 'rate' plus white noise of 'noise' rad/s per axis.
void feed(GyroBias & gb, double seconds, const Eigen::Vector3d & rate, double noise)
{
  std::mt19937 rng(7);
  std::normal_distribution<double> n(0.0, noise);
  const int samples = static_cast<int>(seconds * kRate);
  for (int k = 0; k < samples; ++k) {
    const Eigen::Vector3d w = rate + Eigen::Vector3d(n(rng), n(rng), n(rng));
    gb.add(kStart + rclcpp::Duration::from_seconds(k / kRate), w);
  }
}

}  // namespace

TEST(GyroBias, LearnsParkedBiasThroughNoise)
{
  // Parked Airy IMUs spread 0.7 mrad/s per axis.
  GyroBias gb;
  feed(gb, 2.0, kBias, 0.7e-3);
  ASSERT_TRUE(gb.learned());
  EXPECT_LT((gb.bias() - kBias).norm(), 0.2e-3);
}

TEST(GyroBias, IgnoresAVibratingVehicle)
{
  // Driving shakes the gyro far past a parked one, even at a bias-sized mean.
  GyroBias gb;
  feed(gb, 2.0, kBias, 5e-3);
  EXPECT_FALSE(gb.learned());
}

TEST(GyroBias, KeepsASteadyTurnAsMotion)
{
  GyroBias gb;
  feed(gb, 2.0, Eigen::Vector3d(0.0, 0.0, 0.05), 0.7e-3);
  EXPECT_FALSE(gb.learned());
}

TEST(GyroBias, NeedsAFullStillSecond)
{
  GyroBias gb;
  feed(gb, 0.5, kBias, 0.7e-3);
  EXPECT_FALSE(gb.learned());
}

}  // namespace polka
