#ifndef POLKA__MERGE_ENGINE__I_MERGE_ENGINE_HPP
#define POLKA__MERGE_ENGINE__I_MERGE_ENGINE_HPP

#include "polka/types.hpp"
#include <vector>
#include <Eigen/Geometry>

namespace polka {

struct MergeInput {
  CloudT::ConstPtr cloud;
  Eigen::Isometry3d transform;
  FilterParams filter_params;
};

class IMergeEngine {
public:
  virtual CloudT::Ptr merge(const std::vector<MergeInput> & sources) = 0;
  virtual bool is_gpu() const = 0;
  virtual ~IMergeEngine() = default;
};

}  // namespace polka

#endif  // POLKA__MERGE_ENGINE__I_MERGE_ENGINE_HPP
