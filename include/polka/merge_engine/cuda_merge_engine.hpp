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

#ifndef POLKA__MERGE_ENGINE__CUDA_MERGE_ENGINE_HPP_
#define POLKA__MERGE_ENGINE__CUDA_MERGE_ENGINE_HPP_

#ifdef POLKA_CUDA_ENABLED

#include <cuda_runtime_api.h>

#include <cstdint>
#include <vector>

#include "polka/merge_engine/i_merge_engine.hpp"

namespace polka
{

class CudaMergeEngine : public IMergeEngine
{
public:
  explicit CudaMergeEngine(const MergeConfig & config);
  ~CudaMergeEngine() override;

  CloudT::Ptr merge(const std::vector<MergeInput> & sources) override;
  bool is_gpu() const override {return true;}

  PipelineResult merge_pipeline(
    const std::vector<MergeInput> & sources,
    const PipelineConfig & config) override;

  // Device-side output (outputs.cloud.gpu_buffer). The last merge_pipeline()
  // result stays on the GPU until the next call; pack_result() writes it to 'dst'
  // in the packed layout (types.hpp), 'time' minus 'time_base', queued on stream().
  cudaStream_t stream() const;
  void pack_result(uint8_t * dst, PointTimeField time, double time_base);

private:
  struct Impl;
  Impl * impl_;
};

}  // namespace polka

#endif  // POLKA_CUDA_ENABLED
#endif  // POLKA__MERGE_ENGINE__CUDA_MERGE_ENGINE_HPP_
