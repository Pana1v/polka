#include "polka/merge_engine/cpu_merge_engine.hpp"

namespace polka {

CloudT::Ptr CpuMergeEngine::merge(const std::vector<MergeInput> & sources)
{
  auto output = std::make_shared<CloudT>();
  size_t total = 0;
  for (const auto & src : sources)
    total += src.cloud->size();
  output->resize(total);

  size_t offset = 0;
  for (const auto & src : sources) {
    Eigen::Affine3f tf = src.transform.cast<float>();
    for (size_t i = 0; i < src.cloud->size(); ++i) {
      const auto & p = (*src.cloud)[i];
      auto & o = (*output)[offset + i];
      Eigen::Vector3f out = tf * Eigen::Vector3f(p.x, p.y, p.z);
      o.x = out.x();
      o.y = out.y();
      o.z = out.z();
      o.intensity = p.intensity;
    }
    offset += src.cloud->size();
  }

  output->width = static_cast<uint32_t>(total);
  output->height = 1;
  output->is_dense = true;
  return output;
}

}  // namespace polka
