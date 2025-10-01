#include "polka/filters/angular_filter.hpp"
#include <cmath>

namespace polka {

AngularFilter::AngularFilter(
  const std::vector<std::pair<double, double>> & ranges_deg, bool invert)
: ranges_deg_(ranges_deg), invert_(invert)
{
}

bool AngularFilter::in_ranges(double angle_deg) const
{
  for (const auto & r : ranges_deg_) {
    if (r.first <= r.second) {
      if (angle_deg >= r.first && angle_deg <= r.second) return true;
    } else {
      if (angle_deg >= r.first || angle_deg <= r.second) return true;
    }
  }
  return false;
}

void AngularFilter::apply(CloudT & cloud, const std::string & /*frame_id*/)
{
  size_t j = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto & p = cloud[i];
    double angle_rad = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
    double angle_deg = angle_rad * 180.0 / M_PI;
    if (angle_deg < 0.0) angle_deg += 360.0;

    bool match = in_ranges(angle_deg);
    bool keep = invert_ ? !match : match;
    if (keep) {
      cloud[j++] = p;
    }
  }
  cloud.resize(j);
  cloud.width = static_cast<uint32_t>(j);
  cloud.height = 1;
  cloud.is_dense = true;
}

}  // namespace polka
