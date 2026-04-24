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

#ifdef POLKA_CUDA_ENABLED

#include "polka/merge_engine/cuda_merge_engine.hpp"
#include "polka/merge_engine/cuda_types.cuh"
#include <cuda_runtime.h>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#define POLKA_CUDA_CHECK(call)                                          \
  do {                                                                  \
    cudaError_t err = (call);                                           \
    if (err != cudaSuccess) {                                           \
      fprintf(stderr, "[polka] CUDA error at %s:%d: %s\n",             \
              __FILE__, __LINE__, cudaGetErrorString(err));             \
    }                                                                   \
  } while (0)

#define POLKA_CUDA_CHECK_KERNEL()                                       \
  do {                                                                  \
    cudaError_t err = cudaGetLastError();                               \
    if (err != cudaSuccess) {                                           \
      fprintf(stderr, "[polka] CUDA kernel launch error at %s:%d: %s\n", \
              __FILE__, __LINE__, cudaGetErrorString(err));             \
    }                                                                   \
  } while (0)

namespace polka {

__device__ bool pass_range(float r2, const GpuFilterParams & f) {
  return r2 >= f.min_range_sq && r2 <= f.max_range_sq;
}

// Cross-product angular test (avoids atan2f)
__device__ bool pass_angular(float x, float y, const GpuFilterParams & f) {
  if (!f.angular_enabled) return true;
  bool match = false;
  for (int i = 0; i < f.n_angular_ranges; ++i) {
    float4 b = f.angular_bounds[i];
    float cross_lo = b.x * y - b.y * x;
    float cross_hi = b.z * y - b.w * x;
    bool inside = f.angular_wide[i]
      ? (cross_lo >= 0.0f || cross_hi <= 0.0f)
      : (cross_lo >= 0.0f && cross_hi <= 0.0f);
    if (inside) { match = true; break; }
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
  __shared__ int local_count;
  __shared__ int output_base;
  if (threadIdx.x == 0) local_count = 0;
  __syncthreads();

  int idx = blockIdx.x * blockDim.x + threadIdx.x;

  float4 out_point;
  bool keep = false;

  if (idx < n_points) {
    float4 p = input[idx];
    float r2 = p.x * p.x + p.y * p.y + p.z * p.z;

    if (pass_range(r2, filter)) {
      if (pass_angular(p.x, p.y, filter) && pass_box(p, filter)) {
        float ox = tf[0]*p.x + tf[1]*p.y + tf[2]*p.z  + tf[3];
        float oy = tf[4]*p.x + tf[5]*p.y + tf[6]*p.z  + tf[7];
        float oz = tf[8]*p.x + tf[9]*p.y + tf[10]*p.z + tf[11];
        out_point = make_float4(ox, oy, oz, p.w);
        keep = true;
      }
    }
  }

  int local_idx = 0;
  if (keep) local_idx = atomicAdd(&local_count, 1);
  __syncthreads();

  if (threadIdx.x == 0 && local_count > 0)
    output_base = atomicAdd(output_count, local_count);
  __syncthreads();

  if (keep) output[output_base + local_idx] = out_point;
}

__global__ void output_filter_kernel(
  const float4 * __restrict__ input,
  float4 * __restrict__ output,
  int * __restrict__ output_count,
  GpuOutputFilterParams params,
  int n_points)
{
  __shared__ int local_count;
  __shared__ int output_base;
  if (threadIdx.x == 0) local_count = 0;
  __syncthreads();

  int idx = blockIdx.x * blockDim.x + threadIdx.x;

  float4 p_out;
  bool keep = false;

  if (idx < n_points) {
    float4 p = input[idx];

    if (params.range_enabled) {
      float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
      if (r2 < params.min_range_sq || r2 > params.max_range_sq) keep = false;
      else keep = true;
    } else {
      keep = true;
    }

    if (keep && params.height_cap_enabled) {
      if (p.z < params.z_min || p.z > params.z_max) keep = false;
    }

    if (keep && params.box_enabled) {
      if (p.x < params.box_min.x || p.x > params.box_max.x ||
          p.y < params.box_min.y || p.y > params.box_max.y ||
          p.z < params.box_min.z || p.z > params.box_max.z) keep = false;
    }

    if (keep) {
      for (int i = 0; i < params.n_self_boxes; ++i) {
        if (p.x >= params.self_boxes_min[i].x && p.x <= params.self_boxes_max[i].x &&
            p.y >= params.self_boxes_min[i].y && p.y <= params.self_boxes_max[i].y &&
            p.z >= params.self_boxes_min[i].z && p.z <= params.self_boxes_max[i].z) {
          keep = false;
          break;
        }
      }
    }

    if (keep && params.angular_enabled) {
      bool match = false;
      for (int i = 0; i < params.n_angular_ranges; ++i) {
        float4 b = params.angular_bounds[i];
        float cross_lo = b.x * p.y - b.y * p.x;
        float cross_hi = b.z * p.y - b.w * p.x;
        bool inside = params.angular_wide[i]
          ? (cross_lo >= 0.0f || cross_hi <= 0.0f)
          : (cross_lo >= 0.0f && cross_hi <= 0.0f);
        if (inside) { match = true; break; }
      }
      if (params.angular_invert ? match : !match) keep = false;
    }

    if (keep) p_out = p;
  }

  int local_idx = 0;
  if (keep) local_idx = atomicAdd(&local_count, 1);
  __syncthreads();

  if (threadIdx.x == 0 && local_count > 0)
    output_base = atomicAdd(output_count, local_count);
  __syncthreads();

  if (keep) output[output_base + local_idx] = p_out;
}

__global__ void voxel_insert_kernel(
  const float4 * __restrict__ points,
  int * __restrict__ voxel_keys,
  float4 * __restrict__ voxel_points,
  float inv_lx, float inv_ly, float inv_lz,
  int n_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_points) return;

  float4 p = points[idx];
  int ix = __float2int_rd(p.x * inv_lx);
  int iy = __float2int_rd(p.y * inv_ly);
  int iz = __float2int_rd(p.z * inv_lz);

  unsigned int ux = static_cast<unsigned int>(ix);
  unsigned int uy = static_cast<unsigned int>(iy);
  unsigned int uz = static_cast<unsigned int>(iz);
  int key = static_cast<int>((ux * 2654435761u) ^ (uy * 2246822519u) ^ (uz * 3266489917u));
  if (key == 0) key = 1;

  unsigned int h = (static_cast<unsigned int>(key) & 0x7FFFFFFF) % POLKA_VOXEL_TABLE_SIZE;

  for (int i = 0; i < 16; ++i) {
    unsigned int probe_dist = i * i;
    unsigned int s = (h + probe_dist) % POLKA_VOXEL_TABLE_SIZE;
    int old = atomicCAS(&voxel_keys[s], 0, key);
    if (old == 0) {
      voxel_points[s] = p;
      return;
    }
    if (old == key) return;
  }
}

__global__ void voxel_compact_kernel(
  const int * __restrict__ voxel_keys,
  const float4 * __restrict__ voxel_points,
  float4 * __restrict__ output,
  int * __restrict__ output_count)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= POLKA_VOXEL_TABLE_SIZE) return;

  if (voxel_keys[idx] != 0) {
    int slot = atomicAdd(output_count, 1);
    output[slot] = voxel_points[idx];
  }
}

// IEEE 754 positive floats maintain ordering when reinterpreted as ints.
__global__ void flatten_kernel(
  const float4 * __restrict__ points,
  int * __restrict__ scan_ranges_int,
  GpuFlattenParams fp,
  int n_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_points) return;

  float4 p = points[idx];
  if (p.z < fp.z_min || p.z > fp.z_max) return;

  float range = sqrtf(p.x * p.x + p.y * p.y);
  if (range < fp.r_min || range > fp.r_max) return;

  float az = atan2f(p.y, p.x);
  if (az < fp.a_min || az > fp.a_max) return;

  int bin = __float2int_rd((az - fp.a_min) / fp.a_inc);
  if (bin < 0 || bin >= fp.n_bins) return;

  atomicMin(&scan_ranges_int[bin], __float_as_int(range));
}

__global__ void scan_decode_kernel(
  const int * __restrict__ scan_ranges_int,
  float * __restrict__ scan_ranges_float,
  float r_max,
  int n_bins)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_bins) return;

  int val = scan_ranges_int[idx];
  int sentinel = __float_as_int(r_max);
  scan_ranges_float[idx] = (val == sentinel) ? __int_as_float(0x7F800000) : __int_as_float(val);
}

struct CudaMergeEngine::Impl {
  size_t max_points_per_source = 200000;
  size_t max_total_points = 0;

  float4 * d_buf_a = nullptr;
  float4 * d_buf_b = nullptr;
  int * d_count_a = nullptr;
  int * d_count_b = nullptr;
  float * d_tf = nullptr;
  int * d_voxel_keys = nullptr;
  float4 * d_voxel_points = nullptr;
  int * d_scan_ranges_int = nullptr;
  float * d_scan_ranges_float = nullptr;

  cudaStream_t stream;

  GpuFilterParams to_gpu_filter(const FilterParams & fp) {
    GpuFilterParams gf{};
    gf.min_range_sq = static_cast<float>(fp.min_range_sq);
    gf.max_range_sq = static_cast<float>(fp.max_range_sq);
    gf.angular_enabled = fp.angular_filter_enabled;
    gf.invert = fp.angular_invert;
    if (fp.angular_ranges.size() > static_cast<size_t>(POLKA_MAX_ANGULAR_RANGES)) {
      fprintf(stderr, "[polka] CUDA: angular_ranges count %zu exceeds max %d, truncating\n",
        fp.angular_ranges.size(), POLKA_MAX_ANGULAR_RANGES);
    }
    gf.n_angular_ranges = static_cast<int>(
      std::min(fp.angular_ranges.size(), static_cast<size_t>(POLKA_MAX_ANGULAR_RANGES)));
    for (int i = 0; i < gf.n_angular_ranges; ++i) {
      float lo_deg = static_cast<float>(fp.angular_ranges[i].first);
      float hi_deg = static_cast<float>(fp.angular_ranges[i].second);
      float lo_rad = lo_deg * (M_PI / 180.0f);
      float hi_rad = hi_deg * (M_PI / 180.0f);
      gf.angular_bounds[i] = make_float4(cosf(lo_rad), sinf(lo_rad),
                                          cosf(hi_rad), sinf(hi_rad));
      float span = (lo_deg <= hi_deg) ? (hi_deg - lo_deg) : (360.0f - lo_deg + hi_deg);
      gf.angular_wide[i] = (span > 180.0f);
    }
    gf.box_enabled = fp.box_filter_enabled;
    gf.box_min = make_float3(fp.box_min.x(), fp.box_min.y(), fp.box_min.z());
    gf.box_max = make_float3(fp.box_max.x(), fp.box_max.y(), fp.box_max.z());
    return gf;
  }

  GpuOutputFilterParams to_gpu_output_filter(const PipelineConfig & cfg) {
    GpuOutputFilterParams gf{};
    const auto & fp = cfg.output_filters;

    gf.range_enabled = fp.range_filter_enabled;
    gf.min_range_sq = static_cast<float>(fp.min_range_sq);
    gf.max_range_sq = static_cast<float>(fp.max_range_sq);

    gf.angular_enabled = fp.angular_filter_enabled;
    gf.angular_invert = fp.angular_invert;
    if (fp.angular_ranges.size() > static_cast<size_t>(POLKA_MAX_ANGULAR_RANGES)) {
      fprintf(stderr, "[polka] CUDA: output angular_ranges count %zu exceeds max %d, truncating\n",
        fp.angular_ranges.size(), POLKA_MAX_ANGULAR_RANGES);
    }
    gf.n_angular_ranges = static_cast<int>(
      std::min(fp.angular_ranges.size(), static_cast<size_t>(POLKA_MAX_ANGULAR_RANGES)));
    for (int i = 0; i < gf.n_angular_ranges; ++i) {
      float lo_deg = static_cast<float>(fp.angular_ranges[i].first);
      float hi_deg = static_cast<float>(fp.angular_ranges[i].second);
      float lo_rad = lo_deg * (M_PI / 180.0f);
      float hi_rad = hi_deg * (M_PI / 180.0f);
      gf.angular_bounds[i] = make_float4(cosf(lo_rad), sinf(lo_rad),
                                          cosf(hi_rad), sinf(hi_rad));
      float span = (lo_deg <= hi_deg) ? (hi_deg - lo_deg) : (360.0f - lo_deg + hi_deg);
      gf.angular_wide[i] = (span > 180.0f);
    }

    gf.box_enabled = fp.box_filter_enabled;
    gf.box_min = make_float3(fp.box_min.x(), fp.box_min.y(), fp.box_min.z());
    gf.box_max = make_float3(fp.box_max.x(), fp.box_max.y(), fp.box_max.z());

    gf.n_self_boxes = 0;
    if (cfg.self_filter_enabled) {
      if (cfg.self_filter_boxes.size() > static_cast<size_t>(POLKA_MAX_SELF_BOXES)) {
        fprintf(stderr, "[polka] CUDA: self_filter box count %zu exceeds max %d, truncating\n",
          cfg.self_filter_boxes.size(), POLKA_MAX_SELF_BOXES);
      }
      gf.n_self_boxes = static_cast<int>(
        std::min(cfg.self_filter_boxes.size(), static_cast<size_t>(POLKA_MAX_SELF_BOXES)));
      for (int i = 0; i < gf.n_self_boxes; ++i) {
        const auto & b = cfg.self_filter_boxes[i];
        gf.self_boxes_min[i] = make_float3(b.min.x(), b.min.y(), b.min.z());
        gf.self_boxes_max[i] = make_float3(b.max.x(), b.max.y(), b.max.z());
      }
    }

    gf.height_cap_enabled = cfg.height_cap.enabled;
    gf.z_min = static_cast<float>(cfg.height_cap.z_min);
    gf.z_max = static_cast<float>(cfg.height_cap.z_max);

    return gf;
  }

  GpuFlattenParams to_gpu_flatten(const FlattenParams & fp) {
    GpuFlattenParams gf{};
    gf.z_min = static_cast<float>(fp.z_min);
    gf.z_max = static_cast<float>(fp.z_max);
    gf.a_min = static_cast<float>(fp.angle_min);
    gf.a_max = static_cast<float>(fp.angle_max);
    gf.a_inc = static_cast<float>(fp.angle_increment);
    gf.r_min = static_cast<float>(fp.range_min);
    gf.r_max = static_cast<float>(fp.range_max);
    gf.n_bins = fp.n_bins;
    return gf;
  }
};

CudaMergeEngine::CudaMergeEngine(const MergeConfig & config)
: impl_(new Impl)
{
  impl_->max_total_points = impl_->max_points_per_source * config.sources.size();

  POLKA_CUDA_CHECK(cudaMalloc(&impl_->d_buf_a, impl_->max_total_points * sizeof(float4)));
  POLKA_CUDA_CHECK(cudaMalloc(&impl_->d_buf_b, impl_->max_total_points * sizeof(float4)));
  POLKA_CUDA_CHECK(cudaMalloc(&impl_->d_count_a, sizeof(int)));
  POLKA_CUDA_CHECK(cudaMalloc(&impl_->d_count_b, sizeof(int)));
  POLKA_CUDA_CHECK(cudaMalloc(&impl_->d_tf, 16 * sizeof(float)));

  POLKA_CUDA_CHECK(cudaMalloc(&impl_->d_voxel_keys, POLKA_VOXEL_TABLE_SIZE * sizeof(int)));
  POLKA_CUDA_CHECK(cudaMalloc(&impl_->d_voxel_points, POLKA_VOXEL_TABLE_SIZE * sizeof(float4)));

  POLKA_CUDA_CHECK(cudaMalloc(&impl_->d_scan_ranges_int, POLKA_MAX_SCAN_BINS * sizeof(int)));
  POLKA_CUDA_CHECK(cudaMalloc(&impl_->d_scan_ranges_float, POLKA_MAX_SCAN_BINS * sizeof(float)));

  POLKA_CUDA_CHECK(cudaStreamCreate(&impl_->stream));
}

CudaMergeEngine::~CudaMergeEngine()
{
  cudaFree(impl_->d_buf_a);
  cudaFree(impl_->d_buf_b);
  cudaFree(impl_->d_count_a);
  cudaFree(impl_->d_count_b);
  cudaFree(impl_->d_tf);
  cudaFree(impl_->d_voxel_keys);
  cudaFree(impl_->d_voxel_points);
  cudaFree(impl_->d_scan_ranges_int);
  cudaFree(impl_->d_scan_ranges_float);
  cudaStreamDestroy(impl_->stream);
  delete impl_;
}

CloudT::Ptr CudaMergeEngine::merge(const std::vector<MergeInput> & sources)
{
  auto & s = impl_->stream;
  POLKA_CUDA_CHECK(cudaMemsetAsync(impl_->d_count_b, 0, sizeof(int), s));

  size_t input_offset = 0;
  for (const auto & src : sources) {
    if (src.cloud->size() > impl_->max_points_per_source) {
      fprintf(stderr, "[polka] CUDA: source has %zu points, exceeds max %zu, truncating\n",
        src.cloud->size(), impl_->max_points_per_source);
    }
    size_t n = std::min(src.cloud->size(), impl_->max_points_per_source);

    std::vector<float4> packed(n);
    for (size_t i = 0; i < n; ++i) {
      const auto & pt = src.cloud->points[i];
      packed[i] = make_float4(pt.x, pt.y, pt.z, pt.intensity);
    }
    POLKA_CUDA_CHECK(cudaMemcpyAsync(impl_->d_buf_a + input_offset, packed.data(),
      n * sizeof(float4), cudaMemcpyHostToDevice, s));

    Eigen::Matrix4f mat = src.transform.cast<float>().matrix();
    float tf[16];
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        tf[r * 4 + c] = mat(r, c);
    POLKA_CUDA_CHECK(cudaMemcpyAsync(impl_->d_tf, tf, 16 * sizeof(float), cudaMemcpyHostToDevice, s));

    GpuFilterParams gf = impl_->to_gpu_filter(src.filter_params);
    int blocks = (static_cast<int>(n) + 255) / 256;
    fused_transform_filter_kernel<<<blocks, 256, 0, s>>>(
      impl_->d_buf_a + input_offset, impl_->d_buf_b, impl_->d_count_b,
      impl_->d_tf, gf, static_cast<int>(n));
    POLKA_CUDA_CHECK_KERNEL();

    input_offset += n;
  }

  int count = 0;
  POLKA_CUDA_CHECK(cudaMemcpyAsync(&count, impl_->d_count_b, sizeof(int), cudaMemcpyDeviceToHost, s));
  POLKA_CUDA_CHECK(cudaStreamSynchronize(s));

  auto output = std::make_shared<CloudT>();
  if (count > 0) {
    output->resize(count);
    std::vector<float4> packed_out(count);
    POLKA_CUDA_CHECK(cudaMemcpy(packed_out.data(), impl_->d_buf_b, count * sizeof(float4), cudaMemcpyDeviceToHost));
    for (int i = 0; i < count; ++i) {
      auto & pt = output->points[i];
      pt.x = packed_out[i].x;
      pt.y = packed_out[i].y;
      pt.z = packed_out[i].z;
      pt.intensity = packed_out[i].w;
    }
  }
  output->width = count;
  output->height = 1;
  output->is_dense = true;
  return output;
}

PipelineResult CudaMergeEngine::merge_pipeline(
  const std::vector<MergeInput> & sources,
  const PipelineConfig & config)
{
  auto & s = impl_->stream;

  POLKA_CUDA_CHECK(cudaMemsetAsync(impl_->d_count_b, 0, sizeof(int), s));

  size_t input_offset = 0;
  for (const auto & src : sources) {
    if (src.cloud->size() > impl_->max_points_per_source) {
      fprintf(stderr, "[polka] CUDA: source has %zu points, exceeds max %zu, truncating\n",
        src.cloud->size(), impl_->max_points_per_source);
    }
    size_t n = std::min(src.cloud->size(), impl_->max_points_per_source);

    std::vector<float4> packed(n);
    for (size_t i = 0; i < n; ++i) {
      const auto & pt = src.cloud->points[i];
      packed[i] = make_float4(pt.x, pt.y, pt.z, pt.intensity);
    }
    POLKA_CUDA_CHECK(cudaMemcpyAsync(impl_->d_buf_a + input_offset, packed.data(),
      n * sizeof(float4), cudaMemcpyHostToDevice, s));

    Eigen::Matrix4f mat = src.transform.cast<float>().matrix();
    float tf[16];
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        tf[r * 4 + c] = mat(r, c);
    POLKA_CUDA_CHECK(cudaMemcpyAsync(impl_->d_tf, tf, 16 * sizeof(float), cudaMemcpyHostToDevice, s));

    GpuFilterParams gf = impl_->to_gpu_filter(src.filter_params);
    int blocks = (static_cast<int>(n) + 255) / 256;
    fused_transform_filter_kernel<<<blocks, 256, 0, s>>>(
      impl_->d_buf_a + input_offset, impl_->d_buf_b, impl_->d_count_b,
      impl_->d_tf, gf, static_cast<int>(n));
    POLKA_CUDA_CHECK_KERNEL();

    input_offset += n;
  }

  int merge_count = 0;
  POLKA_CUDA_CHECK(cudaMemcpyAsync(&merge_count, impl_->d_count_b, sizeof(int), cudaMemcpyDeviceToHost, s));
  POLKA_CUDA_CHECK(cudaStreamSynchronize(s));

  if (merge_count <= 0) return {std::make_shared<CloudT>(), {}};

  float4 * cur_data = impl_->d_buf_b;
  int cur_count = merge_count;

  GpuOutputFilterParams ofp = impl_->to_gpu_output_filter(config);
  bool any_output_filter = ofp.range_enabled || ofp.angular_enabled ||
    ofp.box_enabled || ofp.n_self_boxes > 0 || ofp.height_cap_enabled;

  if (any_output_filter) {
    POLKA_CUDA_CHECK(cudaMemsetAsync(impl_->d_count_a, 0, sizeof(int), s));
    int blocks = (cur_count + 255) / 256;
    output_filter_kernel<<<blocks, 256, 0, s>>>(
      impl_->d_buf_b, impl_->d_buf_a, impl_->d_count_a, ofp, cur_count);
    POLKA_CUDA_CHECK_KERNEL();

    POLKA_CUDA_CHECK(cudaMemcpyAsync(&cur_count, impl_->d_count_a, sizeof(int), cudaMemcpyDeviceToHost, s));
    POLKA_CUDA_CHECK(cudaStreamSynchronize(s));

    cur_data = impl_->d_buf_a;
    if (cur_count <= 0) return {std::make_shared<CloudT>(), {}};
  }

  if (config.voxel.enabled && config.voxel.leaf_x > 0.0f) {
    POLKA_CUDA_CHECK(cudaMemsetAsync(impl_->d_voxel_keys, 0,
      POLKA_VOXEL_TABLE_SIZE * sizeof(int), s));

    float inv_lx = 1.0f / config.voxel.leaf_x;
    float inv_ly = 1.0f / config.voxel.leaf_y;
    float inv_lz = 1.0f / config.voxel.leaf_z;

    int blocks = (cur_count + 255) / 256;
    voxel_insert_kernel<<<blocks, 256, 0, s>>>(
      cur_data, impl_->d_voxel_keys, impl_->d_voxel_points,
      inv_lx, inv_ly, inv_lz, cur_count);
    POLKA_CUDA_CHECK_KERNEL();

    float4 * voxel_out = (cur_data == impl_->d_buf_a) ? impl_->d_buf_b : impl_->d_buf_a;
    int * voxel_cnt = (cur_data == impl_->d_buf_a) ? impl_->d_count_b : impl_->d_count_a;

    POLKA_CUDA_CHECK(cudaMemsetAsync(voxel_cnt, 0, sizeof(int), s));
    int compact_blocks = (POLKA_VOXEL_TABLE_SIZE + 255) / 256;
    voxel_compact_kernel<<<compact_blocks, 256, 0, s>>>(
      impl_->d_voxel_keys, impl_->d_voxel_points,
      voxel_out, voxel_cnt);
    POLKA_CUDA_CHECK_KERNEL();

    POLKA_CUDA_CHECK(cudaMemcpyAsync(&cur_count, voxel_cnt, sizeof(int), cudaMemcpyDeviceToHost, s));
    POLKA_CUDA_CHECK(cudaStreamSynchronize(s));

    cur_data = voxel_out;
    if (cur_count <= 0) return {std::make_shared<CloudT>(), {}};
  }

  std::vector<float> scan_ranges;
  if (config.scan_enabled && config.flatten.n_bins > 0) {
    GpuFlattenParams gfp = impl_->to_gpu_flatten(config.flatten);
    int n_bins = std::min(gfp.n_bins, POLKA_MAX_SCAN_BINS);
    gfp.n_bins = n_bins;

    int r_max_int;
    std::memcpy(&r_max_int, &gfp.r_max, sizeof(int));
    std::vector<int> init_vals(n_bins, r_max_int);
    POLKA_CUDA_CHECK(cudaMemcpyAsync(impl_->d_scan_ranges_int, init_vals.data(),
      n_bins * sizeof(int), cudaMemcpyHostToDevice, s));

    int blocks = (cur_count + 255) / 256;
    flatten_kernel<<<blocks, 256, 0, s>>>(
      cur_data, impl_->d_scan_ranges_int, gfp, cur_count);
    POLKA_CUDA_CHECK_KERNEL();

    int decode_blocks = (n_bins + 255) / 256;
    scan_decode_kernel<<<decode_blocks, 256, 0, s>>>(
      impl_->d_scan_ranges_int, impl_->d_scan_ranges_float, gfp.r_max, n_bins);
    POLKA_CUDA_CHECK_KERNEL();

    scan_ranges.resize(n_bins);
  }

  auto output = std::make_shared<CloudT>();
  std::vector<float4> packed_out;
  if (cur_count > 0) {
    packed_out.resize(cur_count);
    POLKA_CUDA_CHECK(cudaMemcpyAsync(packed_out.data(), cur_data,
      cur_count * sizeof(float4), cudaMemcpyDeviceToHost, s));
  }

  if (config.scan_enabled && !scan_ranges.empty()) {
    POLKA_CUDA_CHECK(cudaMemcpyAsync(scan_ranges.data(), impl_->d_scan_ranges_float,
      scan_ranges.size() * sizeof(float), cudaMemcpyDeviceToHost, s));
  }

  POLKA_CUDA_CHECK(cudaStreamSynchronize(s));

  if (cur_count > 0) {
    output->resize(cur_count);
    for (int i = 0; i < cur_count; ++i) {
      auto & pt = output->points[i];
      pt.x = packed_out[i].x;
      pt.y = packed_out[i].y;
      pt.z = packed_out[i].z;
      pt.intensity = packed_out[i].w;
    }
  }
  output->width = static_cast<uint32_t>(cur_count);
  output->height = 1;
  output->is_dense = true;

  return {output, std::move(scan_ranges)};
}

}  // namespace polka

#endif  // POLKA_CUDA_ENABLED
