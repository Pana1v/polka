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
#include "polka/diag/stat_counters.hpp"

namespace polka
{

TEST(StatCounters, RecordAccumulates)
{
  StatCounters c;
  c.record(1000, 500, 400);
  c.record(1000, 500, 300);
  auto s = c.sample();
  EXPECT_EQ(s.msgs, 2u);
  EXPECT_EQ(s.bytes, 2000u);
  EXPECT_EQ(s.points_raw, 1000u);
  EXPECT_EQ(s.points_kept, 700u);
}

TEST(StatWindow, FirstSampleIsInvalid)
{
  StatWindow w;
  StatSample s{10, 1000, 0, 0};
  EXPECT_FALSE(w.update(s, 100.0).valid);
}

TEST(StatWindow, ComputesRatesFromDeltas)
{
  StatWindow w;
  w.update(StatSample{10, 1000, 100, 90}, 100.0);
  auto r = w.update(StatSample{30, 5000, 300, 210}, 102.0);
  ASSERT_TRUE(r.valid);
  EXPECT_DOUBLE_EQ(r.msg_hz, 10.0);            // 20 msgs / 2 s
  EXPECT_DOUBLE_EQ(r.bytes_per_sec, 2000.0);   // 4000 B / 2 s
  EXPECT_DOUBLE_EQ(r.points_raw_per_sec, 100.0);
  EXPECT_DOUBLE_EQ(r.points_kept_per_sec, 60.0);
}

TEST(StatWindow, ZeroOrNegativePeriodIsInvalid)
{
  StatWindow w;
  w.update(StatSample{10, 1000, 0, 0}, 100.0);
  EXPECT_FALSE(w.update(StatSample{20, 2000, 0, 0}, 100.0).valid);
  EXPECT_FALSE(w.update(StatSample{20, 2000, 0, 0}, 99.0).valid);
  // The window re-anchors on the bad sample; a later good one works again.
  auto r = w.update(StatSample{40, 4000, 0, 0}, 101.0);
  ASSERT_TRUE(r.valid);
  EXPECT_DOUBLE_EQ(r.msg_hz, 10.0);  // (40-20) msgs over (101-99)... re-anchored at 99
}

TEST(StatWindow, CounterResetReanchorsInsteadOfGoingNegative)
{
  // Counters restart from zero when a SourceAdapter is recreated at runtime;
  // the window must treat that as a fresh start, not emit a negative rate.
  StatWindow w;
  w.update(StatSample{100, 10000, 0, 0}, 100.0);
  auto r = w.update(StatSample{5, 500, 0, 0}, 101.0);
  EXPECT_FALSE(r.valid);
  auto r2 = w.update(StatSample{15, 1500, 0, 0}, 102.0);
  ASSERT_TRUE(r2.valid);
  EXPECT_DOUBLE_EQ(r2.msg_hz, 10.0);
}

}  // namespace polka
