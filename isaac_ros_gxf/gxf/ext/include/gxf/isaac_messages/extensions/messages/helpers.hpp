// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2020-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <string>
#include <utility>

#include "engine/core/tensor/tensor.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/allocator.hpp"
#include "gxf/std/tensor.hpp"

namespace nvidia {
namespace isaac {

// Adds a tensor with a given `name` to the given `entity`. The tensor is created with the specified
// characteristics (`storage_type` and `dimensions`), using the passed `allocator` instance. This
// method returns an object of type `::nvidia::isaac::CpuTensorView` (of value type `K`
// and rank `N`) that allows access to the underlying `gxf::Tensor`'s data.
template <typename K, int N>
gxf::Expected<::nvidia::isaac::CpuTensorView<K, N>> AddTensorToEntity(
    gxf::Entity& entity, gxf::Handle<gxf::Allocator> allocator, const char* name,
    gxf::MemoryStorageType storage_type, const ::nvidia::isaac::Vector<int, N>& dimensions) {
  if (!allocator) {
    return gxf::Unexpected{GXF_ARGUMENT_NULL};
  }
  // Add Tensor to entity
  auto maybe_tensor = entity.add<gxf::Tensor>(name);
  if (!maybe_tensor) {
    return gxf::ForwardError(maybe_tensor);
  }
  auto& tensor = maybe_tensor.value();
  // Prepare shape object
  std::array<int32_t, gxf::Shape::kMaxRank> shape_entries;
  for (int dimension_index = 0; dimension_index < N; ++dimension_index) {
    shape_entries[dimension_index] = dimensions[dimension_index];
  }
  const gxf::Shape shape(shape_entries, N);
  // Reshape tensor
  auto reshape_result = tensor->reshape<K>(shape, storage_type, allocator);
  if (!reshape_result) {
    return gxf::ForwardError(reshape_result);
  }
  auto maybe_ptr = tensor->data<K>();
  if (!maybe_ptr) {
    return gxf::ForwardError(maybe_ptr);
  }
  return ::nvidia::isaac::CreateCpuTensorViewFromData<K, N>(
      maybe_ptr.value(), shape.size(), dimensions);
}

// Gets a view to the tensor with a given `name` in the given `entity`. This method returns an
// object of type `::nvidia::isaac::CpuTensorView` (of value type `K` and rank `N`) that
// allows access to the underlying `gxf::Tensor`'s data.
template <typename K, int N>
gxf::Expected<::nvidia::isaac::CpuTensorView<K, N>> GetTensorViewFromEntity(gxf::Entity& entity,
                                                                 const char* name) {
  const auto& maybe_tensor = entity.get<gxf::Tensor>(name);
  if (!maybe_tensor) {
    return gxf::ForwardError(maybe_tensor);
  }
  auto maybe_data = maybe_tensor.value()->data<K>();
  if (!maybe_data) {
    return gxf::ForwardError(maybe_data);
  }
  K* ptr_data = maybe_data.value();
  // Memory storage should be CPU or system
  if (maybe_tensor.value()->storage_type() == gxf::MemoryStorageType::kDevice) {
    return gxf::Unexpected{GXF_MEMORY_INVALID_STORAGE_MODE};
  }
  const auto& shape = maybe_tensor.value()->shape();
  if (shape.rank() != N) {
    return gxf::Unexpected{GXF_FAILURE};
  }
  ::nvidia::isaac::Vector<int, N> dimensions;
  for (int i = 0; i < N; i++) {
    dimensions[i] = shape.dimension(i);
  }
  return ::nvidia::isaac::CreateCpuTensorViewFromData<K, N>(ptr_data, shape.size(), dimensions);
}

}  // namespace isaac
}  // namespace nvidia
