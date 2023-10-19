// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2019-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "engine/core/tensor/tensor.hpp"

namespace nvidia {
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
}  // namespace nvidia
