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

#include "cuda_runtime.h"

namespace nvidia {
namespace isaac {

// A CUDA-friendly pointer with a stride. Can be used to work with data which uses a grid layout
// with a non non-trivial offset between rows. This means rows are longer than they would need to be
// in order to enable higher performance.
template <typename K>
struct StridePointer {
  // Pointer to the first element
  K* pointer;
  // Offset in bytes between consecutive rows.
  size_t stride;

  // Gets a pointer to the given row
  inline __host__ __device__
  K* row_pointer(int row) const {
    // TODO Properly cast pointer using reinterpret_cast with either char* or const char*
    return reinterpret_cast<K*>((char*)(pointer) + row * stride);  // NOLINT
  }

  // Accesses the element at the coordinate (row, col)
  inline __host__ __device__
  const K& at(int row, int col) const { return row_pointer(row)[col]; }

  // Accesses the element at the coordinate (row, col)
  inline __host__ __device__
  K& at(int row, int col) { return row_pointer(row)[col]; }

  // Accesses the element at the given coordinates
  inline __host__ __device__
  const K& at(int2 coordinates) const { return at(coordinates.x, coordinates.y); }

  // Accesses the element at the given coordinates
  inline __host__ __device__
  K& at(int2 coordinates) { return at(coordinates.x, coordinates.y); }

  // Identical to `at`
  inline __host__ __device__
  const K& operator()(int row, int col) const { return row_pointer(row)[col]; }

  // Identical to `at`
  inline __host__ __device__
  K& operator()(int row, int col) { return row_pointer(row)[col]; }

  // Identical to `at`
  inline __host__ __device__
  const K& operator()(int2 coordinates) const { return at(coordinates.x, coordinates.y); }

  // Identical to `at`
  inline __host__ __device__
  K& operator()(int2 coordinates) { return at(coordinates.x, coordinates.y); }

  // Casts to a given element type. This can for example be used to cast between a
  // StridePointer<float> used on the CPU and a StridePointer<float4> used on the GPU.
  template <typename S>
  __host__ __device__
  StridePointer<S> cast() const { return {reinterpret_cast<S*>(pointer), stride}; }
};

}  // namespace isaac
}  // namespace nvidia
