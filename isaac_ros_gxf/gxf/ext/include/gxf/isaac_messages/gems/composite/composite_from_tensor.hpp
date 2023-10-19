// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "gems/composite/composite_view.hpp"
#include "gems/composite/schema_tensor_archive.hpp"
#include "gxf/core/expected.hpp"

namespace nvidia {
namespace isaac {

namespace from_tensor_details {

// Check to see if we actually derive from a composite view
std::false_type composite_view_match(const void*);
template <typename K, int32_t N>
std::true_type composite_view_match(const CompositeContainerConstPointer<K, N>*);
template <typename K, int32_t N>
std::true_type composite_view_match(const CompositeContainerPointer<K, N>*);

// Check to see if we actually derive from a composite view
std::false_type composite_data_match(const void*);
template <typename K, int32_t N>
std::true_type composite_data_match(const CompositeContainerArray<K, N>*);
template <typename K, int32_t N>
std::true_type composite_data_match(const CompositeContainerEigen<K, N>*);

// Define helper types.
template <typename T>
using is_composite_view = decltype(composite_view_match(std::declval<T*>()));
template <typename T>
using is_composite_data = decltype(composite_data_match(std::declval<T*>()));

}  // namespace from_tensor_details

// Constructs CompositeConstView from TensorConstView
template <typename T>
gxf::Expected<T> CompositeFromTensor(
    ::nvidia::isaac::CpuTensorConstView<typename T::scalar_t, 1> tensor) {
  static_assert(from_tensor_details::is_composite_view<T>::value, "Type needs to be a composite");
  if (tensor.dimensions()[0] != T::kDimension) {
    return gxf::Unexpected{GXF_INVALID_DATA_FORMAT};
  }
  T derived;
  derived.pointer = tensor.element_wise_begin();
  return derived;
}

// Constructs CompositeView or CompositeConstView from TensorView
template <typename T>
gxf::Expected<T> CompositeFromTensor(
    ::nvidia::isaac::CpuTensorView<typename T::scalar_t, 1> tensor) {
  static_assert(from_tensor_details::is_composite_view<T>::value, "Type needs to be a composite");
  if (tensor.dimensions()[0] != T::kDimension) {
    return gxf::Unexpected{GXF_INVALID_DATA_FORMAT};
  }
  T derived;
  derived.pointer = tensor.element_wise_begin();
  return derived;
}

// Constructs CompositeView or CompositeConstView from Tensor
template <typename T>
gxf::Expected<T> CompositeFromTensor(::nvidia::isaac::CpuTensor<typename T::scalar_t, 1>& tensor) {
  return CompositeFromTensor<T>(tensor.view());
}

// Constructs CompositeArray or CompositeEigen from TensorConstView and schemas
template <typename T>
gxf::Expected<T> CompositeFromTensor(
    ::nvidia::isaac::CpuTensorConstView<typename T::scalar_t, 1> tensor,
    const composite::Schema& tensor_schema,
    const composite::Schema& composite_schema) {
  static_assert(from_tensor_details::is_composite_data<T>::value, "Type must be a composite data "
                "container, since we cannot guarantee parameter order");
  const auto maybe_index_map = CompositeIndexMap::Create(composite_schema, tensor_schema);
  if (!maybe_index_map) {
    return gxf::Unexpected{GXF_INVALID_DATA_FORMAT};
  }

  T derived;
  FromSchemaTensor(tensor, *maybe_index_map, derived);
  return derived;
}

// Constructs CompositeArray or CompositeEigen from Tensor and schemas
template <typename T>
gxf::Expected<T> CompositeFromTensor(::nvidia::isaac::CpuTensor<typename T::scalar_t, 1>& tensor,
    const composite::Schema& tensor_schema,
    const composite::Schema& composite_schema) {
  return CompositeFromTensor<T>(tensor.const_view(), tensor_schema, composite_schema);
}

}  // namespace isaac
}  // namespace nvidia
