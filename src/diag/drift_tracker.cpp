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

#include "polka/diag/drift_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace polka {

namespace {
constexpr double kClearFraction = 0.8;  // clear threshold as fraction of raise threshold
}

DriftTracker::DriftTracker()
: DriftTracker(Config())
{
}

DriftTracker::DriftTracker(const Config & cfg)
{
  set_config(cfg);
}

void DriftTracker::set_config(const Config & cfg)
{
  cfg_ = cfg;
  reset_state();
}

void DriftTracker::reset_state()
{
  status_ = Status();
  status_.expected_rate_from_param = cfg_.expected_rate > 0.0;
  if (status_.expected_rate_from_param)
    status_.expected_rate = cfg_.expected_rate;
  timing_bad_streak_ = 0;
  timing_good_streak_ = 0;
  rate_bad_streak_ = 0;
  rate_good_streak_ = 0;
  baseline_samples_.clear();
  baseline_elapsed_sec_ = 0.0;
}

const DriftTracker::Status & DriftTracker::update(const Input & in)
{
  status_.timing_raised = false;
  status_.timing_cleared = false;
  status_.rate_raised = false;
  status_.rate_cleared = false;

  update_timing(in);
  update_rate(in);
  return status_;
}

void DriftTracker::update_timing(const Input & in)
{
  // A tick without an offset (peers stale, single source) carries no evidence
  // either way: freeze the streaks rather than counting it as good or bad.
  if (!in.has_offset) return;

  if (!status_.ewma_valid) {
    status_.offset_ewma_sec = in.stamp_offset_sec;
    status_.ewma_valid = true;
  } else {
    status_.offset_ewma_sec = cfg_.timing_ewma_alpha * in.stamp_offset_sec +
      (1.0 - cfg_.timing_ewma_alpha) * status_.offset_ewma_sec;
  }

  const double magnitude = std::fabs(status_.offset_ewma_sec);
  if (!status_.timing_drift) {
    if (magnitude > cfg_.timing_threshold_sec) {
      if (++timing_bad_streak_ >= cfg_.timing_min_ticks) {
        status_.timing_drift = true;
        status_.timing_raised = true;
        timing_good_streak_ = 0;
      }
    } else {
      timing_bad_streak_ = 0;
    }
  } else {
    if (magnitude < kClearFraction * cfg_.timing_threshold_sec) {
      if (++timing_good_streak_ >= cfg_.timing_min_ticks) {
        status_.timing_drift = false;
        status_.timing_cleared = true;
        timing_bad_streak_ = 0;
      }
    } else {
      timing_good_streak_ = 0;
    }
  }
}

void DriftTracker::update_rate(const Input & in)
{
  if (!in.has_rate) return;

  // Auto-baseline: observe until the window fills, then lock the median as
  // the expected rate. Median (not mean) so a slow startup tick can't drag
  // the baseline down.
  if (status_.expected_rate <= 0.0) {
    baseline_samples_.push_back(in.rate_hz);
    baseline_elapsed_sec_ += in.tick_period_sec;
    if (baseline_elapsed_sec_ + 1e-9 >= cfg_.rate_baseline_sec) {
      auto mid = baseline_samples_.begin() + baseline_samples_.size() / 2;
      std::nth_element(baseline_samples_.begin(), mid, baseline_samples_.end());
      status_.expected_rate = *mid;
      baseline_samples_.clear();
    }
    return;
  }

  const double raise_floor = status_.expected_rate * (1.0 - cfg_.rate_sag_pct / 100.0);
  const double clear_floor =
    status_.expected_rate * (1.0 - kClearFraction * cfg_.rate_sag_pct / 100.0);

  if (!status_.rate_drift) {
    if (in.rate_hz < raise_floor) {
      if (++rate_bad_streak_ >= cfg_.rate_min_ticks) {
        status_.rate_drift = true;
        status_.rate_raised = true;
        rate_good_streak_ = 0;
      }
    } else {
      rate_bad_streak_ = 0;
    }
  } else {
    if (in.rate_hz >= clear_floor) {
      if (++rate_good_streak_ >= cfg_.rate_min_ticks) {
        status_.rate_drift = false;
        status_.rate_cleared = true;
        rate_bad_streak_ = 0;
      }
    } else {
      rate_good_streak_ = 0;
    }
  }
}

}  // namespace polka
