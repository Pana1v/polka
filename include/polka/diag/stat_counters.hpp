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

#ifndef POLKA__DIAG__STAT_COUNTERS_HPP_
#define POLKA__DIAG__STAT_COUNTERS_HPP_

#include <atomic>
#include <cstdint>

namespace polka
{

// Cumulative totals sampled from StatCounters. Plain values so the diagnostics
// tick can copy them out of the hot path and diff them at leisure.
struct StatSample
{
  uint64_t msgs = 0;
  uint64_t bytes = 0;
  uint64_t points_raw = 0;    // points as received, before per-source filters
  uint64_t points_kept = 0;   // points surviving per-source filters
};

// Monotonic counters incremented from message callbacks. Atomics because the
// producing subscription callback and the consuming diagnostics timer may run
// on different threads when the node is composed into a multithreaded
// container; relaxed ordering is enough for statistics.
class StatCounters
{
public:
  void record(uint64_t byte_count, uint64_t raw_points, uint64_t kept_points)
  {
    msgs_.fetch_add(1, std::memory_order_relaxed);
    bytes_.fetch_add(byte_count, std::memory_order_relaxed);
    points_raw_.fetch_add(raw_points, std::memory_order_relaxed);
    points_kept_.fetch_add(kept_points, std::memory_order_relaxed);
  }

  StatSample sample() const
  {
    return {
      msgs_.load(std::memory_order_relaxed),
      bytes_.load(std::memory_order_relaxed),
      points_raw_.load(std::memory_order_relaxed),
      points_kept_.load(std::memory_order_relaxed)};
  }

private:
  std::atomic<uint64_t> msgs_{0};
  std::atomic<uint64_t> bytes_{0};
  std::atomic<uint64_t> points_raw_{0};
  std::atomic<uint64_t> points_kept_{0};
};

// Turns cumulative totals into per-second rates between successive samples.
// One instance per observed stream, owned by the diagnostics tick (single
// caller, so no synchronization needed here).
class StatWindow
{
public:
  struct Rates
  {
    bool valid = false;
    double msg_hz = 0.0;
    double bytes_per_sec = 0.0;
    double points_raw_per_sec = 0.0;
    double points_kept_per_sec = 0.0;
  };

  // 'now_sec' must come from a monotonic clock: sim-time jumps or a looping
  // rosbag would otherwise produce negative windows.
  Rates update(const StatSample & total, double now_sec)
  {
    Rates r;
    const double period = now_sec - prev_time_;
    // Invalid window (first call, non-advancing clock) or counters that went
    // backwards (the underlying SourceAdapter was recreated at runtime):
    // re-anchor on this sample and report nothing rather than a bogus rate.
    if (!has_prev_ || period <= 0.0 || total.msgs < prev_.msgs) {
      prev_ = total;
      prev_time_ = now_sec;
      has_prev_ = true;
      return r;
    }
    r.valid = true;
    r.msg_hz = static_cast<double>(total.msgs - prev_.msgs) / period;
    r.bytes_per_sec = static_cast<double>(total.bytes - prev_.bytes) / period;
    r.points_raw_per_sec = static_cast<double>(total.points_raw - prev_.points_raw) / period;
    r.points_kept_per_sec = static_cast<double>(total.points_kept - prev_.points_kept) / period;
    prev_ = total;
    prev_time_ = now_sec;
    return r;
  }

private:
  StatSample prev_{};
  double prev_time_ = 0.0;
  bool has_prev_ = false;
};

}  // namespace polka

#endif  // POLKA__DIAG__STAT_COUNTERS_HPP_
