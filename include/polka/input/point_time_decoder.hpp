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

// Decoding of the per-point time field carried by a PointCloud2, kept separate
// from SourceAdapter so the vendor-facing guesswork is unit-testable on its own.

#ifndef POLKA__INPUT__POINT_TIME_DECODER_HPP_
#define POLKA__INPUT__POINT_TIME_DECODER_HPP_

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace polka
{

/// Field names LiDAR vendors use for the per-point acquisition time.
inline const std::vector<std::string> & known_point_time_names()
{
  static const std::vector<std::string> names = {
    "time", "t", "timestamp", "time_stamp", "offset_time", "timeStamp"
  };
  return names;
}

/// A FLOAT64 per-point time at or above this is absolute Unix seconds rather
/// than an offset from the scan header stamp. Unix time passed 1e8 s in 1973,
/// and no scan is 3 years long, so the two cases cannot overlap in practice.
constexpr double kAbsoluteUnixTimeThresholdSec = 1e8;

/// Human-readable PointField datatype, for log lines.
inline const char * point_time_datatype_name(uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::FLOAT64: return "FLOAT64 (seconds)";
    case sensor_msgs::msg::PointField::FLOAT32: return "FLOAT32 (seconds)";
    case sensor_msgs::msg::PointField::UINT32: return "UINT32 (nanoseconds)";
    default: return "UNKNOWN";
  }
}

/// Resolved layout of a source's per-point time field. Decodes raw point bytes
/// to seconds relative to the message header stamp.
struct PointTimeDecoder
{
  uint32_t offset = 0;
  uint8_t datatype = 0;

  /// Seconds from header.stamp to this point's acquisition. May be negative.
  ///
  /// Units and epoch come from the declared datatype, not from the value's
  /// magnitude. An integer field is always a nanosecond offset - no vendor
  /// packs absolute time into one - so it needs no guesswork at all. Only
  /// FLOAT64 is genuinely ambiguous, and only there does the magnitude test
  /// survive.
  double dt(const uint8_t * point_data, double header_sec) const
  {
    switch (datatype) {
      case sensor_msgs::msg::PointField::UINT32: {
          uint32_t val;
          std::memcpy(&val, point_data + offset, sizeof(val));
          return static_cast<double>(val) * 1e-9;
        }

      case sensor_msgs::msg::PointField::FLOAT64: {
          double val;
          std::memcpy(&val, point_data + offset, sizeof(val));
          return (val > kAbsoluteUnixTimeThresholdSec) ? (val - header_sec) : val;
        }

      default: {
          float val;
          std::memcpy(&val, point_data + offset, sizeof(val));
          return static_cast<double>(val);
        }
    }
  }
};

/// Find a usable per-point time field. 'hint' names one field, or "auto" to try
/// the known names. Note the outer loop is over the message's fields, so on a
/// cloud carrying several candidates the message's field order decides, not the
/// order of known_point_time_names().
inline std::optional<PointTimeDecoder> detect_point_time_field(
  const std::vector<sensor_msgs::msg::PointField> & fields,
  const std::string & hint,
  std::string * matched_name = nullptr)
{
  const std::vector<std::string> candidates =
    (hint == "auto") ? known_point_time_names() : std::vector<std::string>{hint};

  for (const auto & field : fields) {
    for (const auto & name : candidates) {
      if (field.name != name) {continue;}

      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 &&
        field.datatype != sensor_msgs::msg::PointField::FLOAT64 &&
        field.datatype != sensor_msgs::msg::PointField::UINT32)
      {
        continue;
      }

      if (matched_name) {*matched_name = field.name;}
      return PointTimeDecoder{field.offset, field.datatype};
    }
  }

  return std::nullopt;
}

/// Largest |dt| a decode may yield before we treat it as a misread rather than
/// a scan. Deliberately not a scan-duration bound: a unit misinference is off by
/// >=1e3 (us read as s) or 1e9 (ns read as s), so a 100 ms scan misread lands at
/// 1e5..1e8 s. Ten seconds sits ~1e7 below that while still clearing a 0.2 Hz
/// aggregated frame. A UINT32 ns field is capped by its own type at 4.295 s and
/// so can never trip this; the bound exists for the unbounded float paths.
constexpr double kMaxPlausibleAbsDtSec = 10.0;

/// max|dt| over a strided sample of the cloud. Feeds the plausibility guard.
/// Deliberately max|dt| and not the dt span: a span is offset-invariant, so it
/// cannot see a source whose clock is skewed wholesale from the header stamp.
inline double point_time_max_abs_dt(
  const PointTimeDecoder & decoder, const uint8_t * data,
  uint32_t point_step, size_t n, double header_sec)
{
  constexpr size_t kSampleCount = 64;
  if (n == 0 || data == nullptr) {return 0.0;}

  const size_t stride = std::max<size_t>(1, (n + kSampleCount - 1) / kSampleCount);

  double max_abs = 0.0;
  for (size_t i = 0; i < n; i += stride) {
    max_abs = std::max(max_abs, std::abs(decoder.dt(data + i * point_step, header_sec)));
  }

  // The extreme sits at the end of a scan, which a stride can step over.
  max_abs = std::max(max_abs, std::abs(decoder.dt(data + (n - 1) * point_step, header_sec)));

  return max_abs;
}

}  // namespace polka

#endif  // POLKA__INPUT__POINT_TIME_DECODER_HPP_
