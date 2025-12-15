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

#ifndef POLKA__MERGE_ENGINE__CPU_MERGE_ENGINE_HPP_
#define POLKA__MERGE_ENGINE__CPU_MERGE_ENGINE_HPP_

#include "polka/merge_engine/i_merge_engine.hpp"

namespace polka {

class CpuMergeEngine : public IMergeEngine {
public:
  CloudT::Ptr merge(const std::vector<MergeInput> & sources) override;
  bool is_gpu() const override { return false; }
};

}  // namespace polka

#endif  // POLKA__MERGE_ENGINE__CPU_MERGE_ENGINE_HPP_
