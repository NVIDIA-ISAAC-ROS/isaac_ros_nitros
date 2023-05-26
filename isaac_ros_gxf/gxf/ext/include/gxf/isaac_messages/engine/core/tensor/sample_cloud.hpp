/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/tensor/tensor.hpp"

namespace isaac {

// -------------------------------------------------------------------------------------------------

namespace sample_cloud_details {

// Helper type to find the correct tensor type based on the number of channels.
template <typename K, int N, typename Buffer>
struct SampleCloudImpl {
  using type = TensorBase<K, tensor::Dimensions<tensor::kDynamic, N>, Buffer>;
};
template <typename K, typename Buffer>
struct SampleCloudImpl<K, 1, Buffer> {
  using type = TensorBase<K, tensor::Rank<1>, Buffer>;
};

}  // namespace sample_cloud_details

// A sample cloud is identical to a (X, N) tensor.
template <typename K, int N, typename Buffer>
using SampleCloudBase = typename sample_cloud_details::SampleCloudImpl<K, N, Buffer>::type;

// -------------------------------------------------------------------------------------------------

template <typename K, int Channels>
using SampleCloud = SampleCloudBase<K, Channels, CpuBuffer>;

template <typename K, int Channels>
using SampleCloudView = SampleCloudBase<K, Channels, CpuBufferView>;

template <typename K, int Channels>
using SampleCloudConstView = SampleCloudBase<K, Channels, CpuBufferConstView>;

#define ISAAC_DECLARE_SAMPLE_CLOUD_TYPES_IMPL(N, T, S)            \
  using SampleCloud##N##S          = SampleCloud<T, N>;           \
  using SampleCloudView##N##S      = SampleCloudView<T, N>;       \
  using SampleCloudConstView##N##S = SampleCloudConstView<T, N>;  \

#define ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(N)                                         \
  template <typename K> using SampleCloud##N          = SampleCloud<K, N>;          \
  template <typename K> using SampleCloudView##N      = SampleCloudView<K, N>;      \
  template <typename K> using SampleCloudConstView##N = SampleCloudConstView<K, N>; \
  ISAAC_DECLARE_SAMPLE_CLOUD_TYPES_IMPL(N, uint8_t, ub)       \
  ISAAC_DECLARE_SAMPLE_CLOUD_TYPES_IMPL(N, uint16_t, ui16)    \
  ISAAC_DECLARE_SAMPLE_CLOUD_TYPES_IMPL(N, int, i)            \
  ISAAC_DECLARE_SAMPLE_CLOUD_TYPES_IMPL(N, double, d)         \
  ISAAC_DECLARE_SAMPLE_CLOUD_TYPES_IMPL(N, float, f)          \

ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(1)
ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(2)
ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(3)
ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(4)
ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(5)
ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(6)
ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(7)
ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(8)
ISAAC_DECLARE_SAMPLE_CLOUD_TYPES(9)

#undef ISAAC_DECLARE_SAMPLE_CLOUD_TYPES
#undef ISAAC_DECLARE_SAMPLE_CLOUD_TYPES_IMPL

// -------------------------------------------------------------------------------------------------

template <typename K, size_t Channels>
using CudaSampleCloud = SampleCloudBase<K, Channels, CudaBuffer>;

template <typename K, size_t Channels>
using CudaSampleCloudView = SampleCloudBase<K, Channels, CudaBufferView>;

template <typename K, size_t Channels>
using CudaSampleCloudConstView = SampleCloudBase<K, Channels, CudaBufferConstView>;

#define ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES_IMPL(N, T, S)               \
  using CudaSampleCloud##N##S          = CudaSampleCloud<T, N>;           \
  using CudaSampleCloudView##N##S      = CudaSampleCloudView<T, N>;       \
  using CudaSampleCloudConstView##N##S = CudaSampleCloudConstView<T, N>;  \

#define ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(N)                                             \
  template <typename K> using CudaSampleCloud##N          = CudaSampleCloud<K, N>;           \
  template <typename K> using CudaSampleCloudView##N      = CudaSampleCloudView<K, N>;       \
  template <typename K> using CudaSampleCloudConstView##N = CudaSampleCloudConstView<K, N>;  \
  ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES_IMPL(N, uint8_t, ub)           \
  ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES_IMPL(N, uint16_t, ui16)        \
  ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES_IMPL(N, int, i)                \
  ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES_IMPL(N, double, d)             \
  ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES_IMPL(N, float, f)              \

ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(1)
ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(2)
ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(3)
ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(4)
ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(5)
ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(6)
ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(7)
ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(8)
ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES(9)

#undef ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES
#undef ISAAC_DECLARE_CUDA_SAMPLE_CLOUD_TYPES_IMPL

// -------------------------------------------------------------------------------------------------

}  // namespace isaac
