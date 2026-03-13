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

// SE(3) exponential map for per-point deskewing.
// Motion model inspired by rko_lio:
//   M.V.R. Malladi, T. Guadagnino, L. Lobefaro, C. Stachniss,
//   "A Robust Approach for LiDAR-Inertial Odometry Without Sensor-Specific Modeling,"
//   arXiv:2509.06593, 2025.

#ifndef POLKA__SE3_EXP_HPP_
#define POLKA__SE3_EXP_HPP_

#include <Eigen/Geometry>
#include <cmath>

namespace polka {

/// Skew-symmetric matrix [v]_x such that [v]_x * w = v x w.
inline Eigen::Matrix3d hat(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m <<     0, -v.z(),  v.y(),
       v.z(),      0, -v.x(),
      -v.y(),  v.x(),      0;
  return m;
}

/// Left Jacobian V of SO(3), used in the SE(3) exponential map.
///   V = I + (1 - cos θ)/θ² [φ]_x + (θ - sin θ)/θ³ [φ]_x²
/// For small θ, uses Taylor expansion for numerical stability.
inline Eigen::Matrix3d so3_left_jacobian(const Eigen::Vector3d & phi)
{
  double theta = phi.norm();
  Eigen::Matrix3d Phi = hat(phi);

  if (theta < 1e-10)
    return Eigen::Matrix3d::Identity() + 0.5 * Phi;

  double theta2 = theta * theta;
  double a = (1.0 - std::cos(theta)) / theta2;
  double b = (theta - std::sin(theta)) / (theta2 * theta);

  return Eigen::Matrix3d::Identity() + a * Phi + b * Phi * Phi;
}

/// SE(3) exponential map: twist (ρ, φ) ∈ se(3) → rigid transform ∈ SE(3).
///   ρ: translational component
///   φ: rotational component (axis-angle, ‖φ‖ = rotation angle)
/// Returns T such that T.translation() = V * ρ and T.rotation() = exp(φ).
inline Eigen::Isometry3d se3_exp(const Eigen::Vector3d & rho,
                                 const Eigen::Vector3d & phi)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  double theta = phi.norm();

  if (theta < 1e-10) {
    T.translation() = rho;
    return T;
  }

  T.linear() = Eigen::AngleAxisd(theta, phi / theta).toRotationMatrix();
  T.translation() = so3_left_jacobian(phi) * rho;
  return T;
}

/// Constant-acceleration + constant-angular-velocity motion model.
///   angular_vel: body-frame angular velocity ω (rad/s)
///   accel:       body-frame linear acceleration a (m/s²)
///   dt:          time offset from reference (seconds, can be negative)
/// Returns the SE(3) pose delta representing sensor motion over dt.
///   Translation: ρ = ½ a dt²   (no velocity term — IMU provides accel only)
///   Rotation:    φ = ω dt
inline Eigen::Isometry3d compute_motion_delta(
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & accel,
  double dt)
{
  Eigen::Vector3d rho = accel * (0.5 * dt * dt);
  Eigen::Vector3d phi = angular_vel * dt;
  return se3_exp(rho, phi);
}

}  // namespace polka

#endif  // POLKA__SE3_EXP_HPP_
