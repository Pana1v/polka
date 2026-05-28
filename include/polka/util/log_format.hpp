// Copyright 2025 Panav Arpit Raaj <praajarpit@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

namespace polka {

// Standard throttle windows (milliseconds) for RCLCPP_*_THROTTLE.
//   fast   — per-message validation (e.g. non-finite IMU values)
//   normal — steady-state degradation (e.g. stale source, TF failing)
//   slow   — persistent suppressed state
constexpr int kLogThrottleFastMs   = 1000;
constexpr int kLogThrottleNormalMs = 5000;
constexpr int kLogThrottleSlowMs   = 30000;

}  // namespace polka
