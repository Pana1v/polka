#include "polka/filters/box_filter.hpp"

namespace polka {

BoxFilter::BoxFilter(const Eigen::Vector3d & box_min, const Eigen::Vector3d & box_max,
                     bool invert)
: bx_min_(static_cast<float>(box_min.x())), bx_max_(static_cast<float>(box_max.x())),
  by_min_(static_cast<float>(box_min.y())), by_max_(static_cast<float>(box_max.y())),
  bz_min_(static_cast<float>(box_min.z())), bz_max_(static_cast<float>(box_max.z())),
  invert_(invert)
{
}

void BoxFilter::apply(CloudT & cloud, const std::string & /*frame_id*/)
{
  size_t j = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto & p = cloud[i];
    bool inside = p.x >= bx_min_ && p.x <= bx_max_ &&
                  p.y >= by_min_ && p.y <= by_max_ &&
                  p.z >= bz_min_ && p.z <= bz_max_;
    if (inside != invert_) {
      cloud[j++] = p;
    }
  }
  cloud.resize(j);
  cloud.width = static_cast<uint32_t>(j);
  cloud.height = 1;
  cloud.is_dense = true;
}

}  // namespace polka
