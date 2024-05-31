// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "gems/core/image/image.hpp"
#include "gems/core/math/types.hpp"
#include "gems/core/tensor/tensor.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/tensor.hpp"

namespace nvidia {
namespace isaac {

// Creates an Isaac image view of a GXF host tensor
// Note that N here is the number of CHANNELS, not the number of dimensions.
// N = 1: single-channel image, N = 3: 3-channel image, etc.
template <typename K, int N>
gxf::Expected<::nvidia::isaac::ImageView<K, N>> ToIsaacImageView(gxf::Tensor& tensor) {
  if (tensor.storage_type() != gxf::MemoryStorageType::kHost) {
    return gxf::Unexpected{GXF_INVALID_DATA_FORMAT};
  }

  // Check if shape is compatible
  if (N < 1) {
    GXF_LOG_ERROR("Channel count (%d) must be at least 1", N);
    return gxf::Unexpected{GXF_INVALID_DATA_FORMAT};
  } else if (N == 1) {
    if (tensor.rank() != 2 && !(tensor.rank() == 3 && tensor.shape().dimension(2) == 1)) {
      GXF_LOG_ERROR("Either rank is 2, or rank is 3 and the number of channels is 1. "
                    "Channel count: %d, rank: %d", N, tensor.rank());
      return gxf::Unexpected{GXF_INVALID_DATA_FORMAT};
    }
  } else {  // N > 1
    if (tensor.rank() != 3) {
      GXF_LOG_ERROR("If channel count is greater than 1, rank must be 3."
                    "Channel count: %d, rank: %d", N, tensor.rank());
      return gxf::Unexpected{GXF_INVALID_DATA_FORMAT};
    }
  }
  const size_t rows = tensor.shape().dimension(0);
  const size_t cols = tensor.shape().dimension(1);

  auto maybe_ptr = tensor.data<K>();
  if (!maybe_ptr) {
    return gxf::ForwardError(maybe_ptr);
  }

  return ::nvidia::isaac::CreateImageView<K, N>(maybe_ptr.value(), rows, cols);
}

// Creates an Isaac image const view of a GXF host tensor
// Note that N here is the number of CHANNELS, not the number of dimensions.
// N = 1: single-channel image, N = 3: 3-channel image, etc.
template <typename K, int N>
gxf::Expected<::nvidia::isaac::ImageConstView<K, N>> ToIsaacImageView(const gxf::Tensor& tensor) {
  if (tensor.storage_type() != gxf::MemoryStorageType::kHost) {
    return gxf::Unexpected{GXF_INVALID_DATA_FORMAT};
  }
  if ((N == 1 && tensor.rank() != 2) || (N > 1 && tensor.rank() != 3)) {
    GXF_LOG_ERROR("N: %d Tensor rank: %d", N, tensor.rank());
    return gxf::Unexpected{GXF_INVALID_DATA_FORMAT};
  }
  const size_t rows = tensor.shape().dimension(0);
  const size_t cols = tensor.shape().dimension(1);

  auto maybe_ptr = tensor.data<K>();
  if (!maybe_ptr) {
    return gxf::ForwardError(maybe_ptr);
  }

  return ::nvidia::isaac::CreateImageConstView<K, N>(maybe_ptr.value(), rows, cols);
}

}  // namespace isaac
}  // namespace nvidia
