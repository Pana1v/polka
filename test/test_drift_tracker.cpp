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
#include "polka/diag/drift_tracker.hpp"

namespace polka {

namespace {

DriftTracker::Config test_config()
{
  DriftTracker::Config cfg;
  cfg.timing_threshold_sec = 0.1;
  cfg.timing_ewma_alpha = 1.0;  // no smoothing: raw offset drives the flag directly
  cfg.timing_min_ticks = 3;
  cfg.rate_sag_pct = 20.0;
  cfg.rate_min_ticks = 3;
  cfg.rate_baseline_sec = 3.0;
  cfg.expected_rate = 10.0;
  return cfg;
}

DriftTracker::Input offset_tick(double offset)
{
  DriftTracker::Input in;
  in.has_offset = true;
  in.stamp_offset_sec = offset;
  return in;
}

DriftTracker::Input rate_tick(double rate)
{
  DriftTracker::Input in;
  in.has_rate = true;
  in.rate_hz = rate;
  in.tick_period_sec = 1.0;
  return in;
}

}  // namespace

TEST(DriftTrackerTiming, RaisesOnlyAfterConsecutiveTicksOverThreshold)
{
  DriftTracker t(test_config());
  EXPECT_FALSE(t.update(offset_tick(0.2)).timing_drift);
  EXPECT_FALSE(t.update(offset_tick(0.2)).timing_drift);
  auto s = t.update(offset_tick(0.2));
  EXPECT_TRUE(s.timing_drift);
  EXPECT_TRUE(s.timing_raised);  // transition flagged exactly on the raising tick
  EXPECT_FALSE(t.update(offset_tick(0.2)).timing_raised);
}

TEST(DriftTrackerTiming, SingleBadTickDoesNotRaise)
{
  DriftTracker t(test_config());
  t.update(offset_tick(0.2));
  t.update(offset_tick(0.0));  // streak broken
  t.update(offset_tick(0.2));
  EXPECT_FALSE(t.update(offset_tick(0.2)).timing_drift);  // only 2 consecutive
}

TEST(DriftTrackerTiming, NegativeOffsetAlsoRaises)
{
  DriftTracker t(test_config());
  t.update(offset_tick(-0.2));
  t.update(offset_tick(-0.2));
  EXPECT_TRUE(t.update(offset_tick(-0.2)).timing_drift);
}

TEST(DriftTrackerTiming, ClearsWithHysteresis)
{
  DriftTracker t(test_config());
  for (int i = 0; i < 3; ++i) t.update(offset_tick(0.2));
  ASSERT_TRUE(t.status().timing_drift);

  // 0.09 is below the 0.1 raise threshold but above the 0.8*0.1 clear
  // threshold: the flag must hold (this is the hysteresis band).
  for (int i = 0; i < 10; ++i)
    EXPECT_TRUE(t.update(offset_tick(0.09)).timing_drift);

  // Below the clear threshold: needs min_ticks consecutive ticks to clear.
  EXPECT_TRUE(t.update(offset_tick(0.01)).timing_drift);
  EXPECT_TRUE(t.update(offset_tick(0.01)).timing_drift);
  auto s = t.update(offset_tick(0.01));
  EXPECT_FALSE(s.timing_drift);
  EXPECT_TRUE(s.timing_cleared);
}

TEST(DriftTrackerTiming, EwmaSmoothsSpikes)
{
  auto cfg = test_config();
  cfg.timing_ewma_alpha = 0.2;
  DriftTracker t(cfg);
  // A single huge spike into a settled-at-zero EWMA must not push the
  // smoothed offset over threshold (0.2 * 0.3 = 0.06 < 0.1).
  for (int i = 0; i < 10; ++i) t.update(offset_tick(0.0));
  for (int i = 0; i < 3; ++i) t.update(offset_tick(0.3));
  // 3 ticks of 0.3 through alpha=0.2 EWMA: 0.06, 0.108, 0.146 - only 2 ticks
  // exceed 0.1, so the flag is still down.
  EXPECT_FALSE(t.status().timing_drift);
}

TEST(DriftTrackerTiming, NoOffsetTicksDoNotAdvanceStreaks)
{
  DriftTracker t(test_config());
  t.update(offset_tick(0.2));
  t.update(offset_tick(0.2));
  DriftTracker::Input gap;  // has_offset = false (e.g. peers went stale)
  t.update(gap);
  // The no-data tick neither raised nor reset the streak; next bad tick completes it.
  EXPECT_TRUE(t.update(offset_tick(0.2)).timing_drift);
}

TEST(DriftTrackerRate, ExplicitExpectedRateRaisesAndClears)
{
  DriftTracker t(test_config());  // expected 10 Hz, sag 20% -> raise below 8 Hz
  EXPECT_FALSE(t.update(rate_tick(9.0)).rate_drift);   // above sag floor
  for (int i = 0; i < 2; ++i) t.update(rate_tick(6.0));
  auto s = t.update(rate_tick(6.0));
  EXPECT_TRUE(s.rate_drift);
  EXPECT_TRUE(s.rate_raised);
  EXPECT_DOUBLE_EQ(s.expected_rate, 10.0);
  EXPECT_STREQ(s.expected_rate_source(), "param");

  // Recovery needs min_ticks consecutive healthy ticks.
  t.update(rate_tick(10.0));
  t.update(rate_tick(10.0));
  auto s2 = t.update(rate_tick(10.0));
  EXPECT_FALSE(s2.rate_drift);
  EXPECT_TRUE(s2.rate_cleared);
}

TEST(DriftTrackerRate, AutoBaselineLocksMedianThenDetectsSag)
{
  auto cfg = test_config();
  cfg.expected_rate = 0.0;  // auto
  DriftTracker t(cfg);

  // During baseline accumulation (3 s at 1 s ticks) nothing can be flagged.
  EXPECT_STREQ(t.status().expected_rate_source(), "none");
  t.update(rate_tick(10.0));
  t.update(rate_tick(10.2));
  auto s = t.update(rate_tick(9.8));
  EXPECT_FALSE(s.rate_drift);
  EXPECT_STREQ(s.expected_rate_source(), "auto");
  EXPECT_NEAR(s.expected_rate, 10.0, 0.01);  // median of 10.0, 10.2, 9.8

  for (int i = 0; i < 3; ++i) t.update(rate_tick(5.0));
  EXPECT_TRUE(t.status().rate_drift);
}

TEST(DriftTrackerRate, InvalidRateTicksDoNotAdvanceStreaks)
{
  DriftTracker t(test_config());
  t.update(rate_tick(5.0));
  t.update(rate_tick(5.0));
  DriftTracker::Input gap;  // has_rate = false
  t.update(gap);
  EXPECT_TRUE(t.update(rate_tick(5.0)).rate_drift);
}

TEST(DriftTrackerRate, SetConfigResetsTracking)
{
  DriftTracker t(test_config());
  for (int i = 0; i < 3; ++i) t.update(rate_tick(5.0));
  ASSERT_TRUE(t.status().rate_drift);

  auto cfg = test_config();
  cfg.expected_rate = 5.0;  // operator declares 5 Hz is nominal
  t.set_config(cfg);
  EXPECT_FALSE(t.status().rate_drift);
  for (int i = 0; i < 5; ++i)
    EXPECT_FALSE(t.update(rate_tick(5.0)).rate_drift);
}

}  // namespace polka
