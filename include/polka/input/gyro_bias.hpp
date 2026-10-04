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

#ifndef POLKA__INPUT__GYRO_BIAS_HPP_
#define POLKA__INPUT__GYRO_BIAS_HPP_

#include <Eigen/Core>

#include <deque>

#include <rclcpp/time.hpp>

namespace polka
{

// Gyro bias, learned whenever the sensor stands still.
//
//   gyro --> [last 1 s] --> still? --> bias = mean of that second
//                           every axis spreads < 2 mrad/s, and |mean| < 35 mrad/s
//
// A MEMS gyro reads a few to tens of mrad/s while parked (24.6 on a RoboSense Airy),
// which deskew would apply as a turn: 5 cm at 20 m per 0.1 s scan. Moving vehicles
// vibrate well past that spread, and any real turn faster than the cap is kept as
// motion. A perfectly smooth turn slower than the cap would be learned as bias.
class GyroBias
{
public:
  // Feeds one raw sample. Not thread-safe: call from one IMU callback.
  void add(const rclcpp::Time & stamp, const Eigen::Vector3d & gyro);

  // Newest learned bias (rad/s), zero until the first still second.
  const Eigen::Vector3d & bias() const {return bias_;}
  bool learned() const {return learned_;}

private:
  struct Sample
  {
    rclcpp::Time stamp;
    Eigen::Vector3d gyro;
  };

  std::deque<Sample> window_;
  Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();
  bool learned_ = false;
};

}  // namespace polka

#endif  // POLKA__INPUT__GYRO_BIAS_HPP_
