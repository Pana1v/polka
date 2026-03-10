#ifdef POLKA_CUDA_ENABLED

#include "polka/merge_engine/cuda_merge_engine.hpp"
#include "polka/merge_engine/cuda_types.cuh"
#include <cuda_runtime.h>
#include <cstring>
#include <algorithm>

namespace polka {

// --- Kernels ---

__device__ bool pass_range(float4 p, const GpuFilterParams & f) {
  float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
  return r2 >= f.min_range_sq && r2 <= f.max_range_sq;
}

__device__ bool pass_angular(float4 p, const GpuFilterParams & f) {
  if (!f.angular_enabled) return true;
  float deg = atan2f(p.y, p.x) * (180.0f / 3.14159265f);
  if (deg < 0.0f) deg += 360.0f;
  bool match = false;
  for (int i = 0; i < f.n_angular_ranges; ++i) {
    float lo = f.angular_ranges[i].x, hi = f.angular_ranges[i].y;
    if (lo <= hi) { if (deg >= lo && deg <= hi) { match = true; break; } }
    else { if (deg >= lo || deg <= hi) { match = true; break; } }
  }
  return f.invert ? !match : match;
}

__device__ bool pass_box(float4 p, const GpuFilterParams & f) {
  if (!f.box_enabled) return true;
  return p.x >= f.box_min.x && p.x <= f.box_max.x &&
         p.y >= f.box_min.y && p.y <= f.box_max.y &&
         p.z >= f.box_min.z && p.z <= f.box_max.z;
}

__global__ void fused_transform_filter_kernel(
  const float4 * __restrict__ input,
  float4 * __restrict__ output,
  int * __restrict__ output_count,
  const float * __restrict__ tf,
  GpuFilterParams filter,
  int n_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_points) return;

  float4 p = input[idx];
  if (!pass_range(p, filter)) return;
  if (!pass_angular(p, filter)) return;
  if (!pass_box(p, filter)) return;

  float ox = tf[0]*p.x + tf[1]*p.y + tf[2]*p.z  + tf[3];
  float oy = tf[4]*p.x + tf[5]*p.y + tf[6]*p.z  + tf[7];
  float oz = tf[8]*p.x + tf[9]*p.y + tf[10]*p.z + tf[11];

  int slot = atomicAdd(output_count, 1);
  output[slot] = make_float4(ox, oy, oz, p.w);
}

// --- Pimpl ---

struct CudaMergeEngine::Impl {
  size_t max_points_per_source = 200000;
  size_t max_total_points = 0;

  float4 * d_input = nullptr;
  float4 * d_output = nullptr;
  int * d_output_count = nullptr;
  float * d_tf = nullptr;

  cudaStream_t stream;

  GpuFilterParams to_gpu_filter(const FilterParams & fp) {
    GpuFilterParams gf{};
    gf.min_range_sq = static_cast<float>(fp.min_range_sq);
    gf.max_range_sq = static_cast<float>(fp.max_range_sq);
    gf.angular_enabled = fp.angular_filter_enabled;
    gf.invert = fp.angular_invert;
    gf.n_angular_ranges = static_cast<int>(
      std::min(fp.angular_ranges.size(), static_cast<size_t>(POLKA_MAX_ANGULAR_RANGES)));
    for (int i = 0; i < gf.n_angular_ranges; ++i)
      gf.angular_ranges[i] = make_float2(
        static_cast<float>(fp.angular_ranges[i].first),
        static_cast<float>(fp.angular_ranges[i].second));
    gf.box_enabled = fp.box_filter_enabled;
    gf.box_min = make_float3(fp.box_min.x(), fp.box_min.y(), fp.box_min.z());
    gf.box_max = make_float3(fp.box_max.x(), fp.box_max.y(), fp.box_max.z());
    return gf;
  }
};

CudaMergeEngine::CudaMergeEngine(const MergeConfig & config)
: impl_(new Impl)
{
  impl_->max_total_points = impl_->max_points_per_source * config.sources.size();

  cudaMalloc(&impl_->d_input, impl_->max_total_points * sizeof(float4));
  cudaMalloc(&impl_->d_output, impl_->max_total_points * sizeof(float4));
  cudaMalloc(&impl_->d_output_count, sizeof(int));
  cudaMalloc(&impl_->d_tf, 16 * sizeof(float));
  cudaStreamCreate(&impl_->stream);
}

CudaMergeEngine::~CudaMergeEngine()
{
  cudaFree(impl_->d_input);
  cudaFree(impl_->d_output);
  cudaFree(impl_->d_output_count);
  cudaFree(impl_->d_tf);
  cudaStreamDestroy(impl_->stream);
  delete impl_;
}

CloudT::Ptr CudaMergeEngine::merge(const std::vector<MergeInput> & sources)
{
  auto & s = impl_->stream;
  cudaMemsetAsync(impl_->d_output_count, 0, sizeof(int), s);

  size_t input_offset = 0;
  for (const auto & src : sources) {
    size_t n = std::min(src.cloud->size(), impl_->max_points_per_source);
    cudaMemcpyAsync(impl_->d_input + input_offset, src.cloud->data(),
      n * sizeof(float4), cudaMemcpyHostToDevice, s);

    Eigen::Matrix4f mat = src.transform.cast<float>().matrix();
    float tf[16];
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        tf[r * 4 + c] = mat(r, c);
    cudaMemcpyAsync(impl_->d_tf, tf, 16 * sizeof(float), cudaMemcpyHostToDevice, s);

    GpuFilterParams gf = impl_->to_gpu_filter(src.filter_params);
    int blocks = (static_cast<int>(n) + 255) / 256;
    fused_transform_filter_kernel<<<blocks, 256, 0, s>>>(
      impl_->d_input + input_offset, impl_->d_output, impl_->d_output_count,
      impl_->d_tf, gf, static_cast<int>(n));

    input_offset += n;
  }

  int count = 0;
  cudaMemcpyAsync(&count, impl_->d_output_count, sizeof(int), cudaMemcpyDeviceToHost, s);
  cudaStreamSynchronize(s);

  auto output = std::make_shared<CloudT>();
  if (count > 0) {
    output->resize(count);
    cudaMemcpy(output->data(), impl_->d_output, count * sizeof(float4), cudaMemcpyDeviceToHost);
  }
  output->width = count;
  output->height = 1;
  output->is_dense = true;
  return output;
}

}  // namespace polka

#endif  // POLKA_CUDA_ENABLED
