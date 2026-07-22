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

#ifndef POLKA__DIAG__DRIFT_TRACKER_HPP_
#define POLKA__DIAG__DRIFT_TRACKER_HPP_

#include <vector>

namespace polka {

// Per-source drift detector, fed once per diagnostics tick. ROS-free so the
// raise/clear behavior is unit-testable in isolation.
//
// Both detectors use the same streak-with-hysteresis shape: a flag raises only
// after 'min_ticks' consecutive bad ticks (one bad tick is jitter, a streak is
// a condition) and clears only after 'min_ticks' consecutive ticks below a
// clear threshold set at 80% of the raise threshold, so a value hovering at
// the boundary cannot flap the flag.
class DriftTracker {
public:
  struct Config {
    // Timing drift: EWMA of (source stamp - peer median stamp).
    double timing_threshold_sec = 0.1;
    double timing_ewma_alpha = 0.2;
    int timing_min_ticks = 5;
    // Rate drift: windowed rate sagging below the expected rate.
    double rate_sag_pct = 20.0;
    int rate_min_ticks = 5;
    double rate_baseline_sec = 10.0;  // auto-baseline observation window
    double expected_rate = 0.0;       // Hz; 0 = auto-baseline from observed rates
  };

  struct Input {
    bool has_offset = false;        // false when fewer than 2 fresh peers exist
    double stamp_offset_sec = 0.0;  // this source's stamp minus the peer median
    bool has_rate = false;          // false when the rate window is not yet valid
    double rate_hz = 0.0;
    double tick_period_sec = 1.0;   // advances the auto-baseline window
  };

  struct Status {
    bool timing_drift = false;
    bool rate_drift = false;
    bool ewma_valid = false;
    double offset_ewma_sec = 0.0;
    double expected_rate = 0.0;  // 0 until known
    // Transition markers, true only on the tick the flag flipped (for logging).
    bool timing_raised = false;
    bool timing_cleared = false;
    bool rate_raised = false;
    bool rate_cleared = false;

    const char * expected_rate_source() const
    {
      if (expected_rate <= 0.0) return "none";
      return expected_rate_from_param ? "param" : "auto";
    }
    bool expected_rate_from_param = false;
  };

  DriftTracker();
  explicit DriftTracker(const Config & cfg);

  // Replacing the config resets all tracking state: thresholds and the
  // expected-rate semantics changed under us, so streaks and baselines
  // measured against the old config are meaningless.
  void set_config(const Config & cfg);

  const Status & update(const Input & in);
  const Status & status() const { return status_; }

private:
  void update_timing(const Input & in);
  void update_rate(const Input & in);
  void reset_state();

  Config cfg_;
  Status status_;

  int timing_bad_streak_ = 0;
  int timing_good_streak_ = 0;
  int rate_bad_streak_ = 0;
  int rate_good_streak_ = 0;

  // Auto-baseline accumulation (only used while expected_rate is unknown).
  std::vector<double> baseline_samples_;
  double baseline_elapsed_sec_ = 0.0;
};

}  // namespace polka

#endif  // POLKA__DIAG__DRIFT_TRACKER_HPP_
