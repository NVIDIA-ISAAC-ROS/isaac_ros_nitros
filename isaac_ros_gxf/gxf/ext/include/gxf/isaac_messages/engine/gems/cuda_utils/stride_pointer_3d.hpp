// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_ISAAC_ENGINE_GEMS_CUDA_UTILS_STRIDE_POINTER_3D_HPP_
#define NVIDIA_ISAAC_ENGINE_GEMS_CUDA_UTILS_STRIDE_POINTER_3D_HPP_

#include <array>
#include <cstddef>

#include "cuda_runtime.h"

namespace nvidia {
namespace isaac {

// TODO(kchinniah): move memory layout & shape defns inside StridePointer3D class

// TODO(kpatzwaldt): merge all the multidimensional indexing methods (ISAFETY-1363)

/// The memory layout of a 3D pointer
enum class StridePointer3D_MemoryLayout {
  kNCHW,  ///< batch_size x channel x height x width
  kHWC,   ///< height x width x channel
  kNHWC,  ///< batch_size x height x width x channel
};

/// The shape of a 3D pointer
struct StridePointer3D_Shape {
  /// The height
  uint32_t height{};
  /// The width
  uint32_t width{};
  /// The number of channels
  uint32_t channels{};
};

// A stride pointer that's suitable for indexing 3D data.
// In particular, this is useful for processing tensors.
// If no strides are specified, the strides are computed assuming no offsets / padding.
// WARNING: the NCHW and NHWC drop the N (batch size) dimension, hence
// assuming that the batch size is 1.
// and are only included to make converting tensors easier.
// Additionally, this pointer does not have ownership of the passed pointer
// Access to this must be given as: (row, col, depth) or (height, width, channel)
// Bound checking is responsible on the caller of the methods of this class
template <typename T>
class StridePointer3D {
 private:
  /// The maximum allowable dimension
  static constexpr size_t kMaxDims{3};

  // Ensure that the received data has a size greater than 0
  static_assert(sizeof(T) > 0, "Error: received type who's size is 0");

 public:
  StridePointer3D() : memory_buffer_{nullptr}, shape_{}, memory_layout_{}, strides_{0, 0, 0} {}

  /**
  Creates a 3D Stride Pointer using the raw memory, shape and memory layout specified.

  This will compute trivial strides for the strides

  @param T The type of the memory buffer
  @param memory_buffer The memory buffer that will be traversed
  @param shape The shape of the memory buffer
  @param memory_layout The memory layout of the memory buffer
  **/
  StridePointer3D(
      T* memory_buffer, StridePointer3D_Shape shape, StridePointer3D_MemoryLayout memory_layout)
      : memory_buffer_{memory_buffer}, shape_{shape}, memory_layout_{memory_layout} {
    computeStrides();
    setStrideIndices();
  }

  /**
  Creates a 3D Stride Pointer using the raw memory, shape, memory layout and strides specified.

  The provided strides must be given in terms of bytes. These will be divided by sizeof(T) for
  indexing

  The byte_strides must be formatted as:
  - byte_strides[2] = The amount of bytes necessary to traverse one unit of the least significant
  dim
  - byte_strides[1] = The amount of bytes necessary to traverse one unit of the 2nd least
  significant dim
  - byte_strides[0] = The amount of bytes necessary to traverse one unit of the 3rd least
  significant dim

  For example, suppose the memory layout is MemoryLayout::kHWC and that there is no extra padding or
  offsets,
  - byte_strides[2] should be sizeof(T) since it requires sizeof(T) units to traverse along the
  channel axis
  - byte_strides[1] should be sizeof(T) * shape.channels since this is the amount of bytes required
  to traverse along the width axis
    - For example, if there are 3 channels and sizeof(T) = 1, then to go from width = 0 -> width = 1
  requires traversal of 3 units
  - byte_strides[0] should be sizeof(T) * shape.channels * shape.width since this is the amount of
  bytes required to traverse along the height axis
    - For example, if there are 3 channels, sizeof(T) = 1 and the width is 5, then to go from width
  = 0 -> 1 requires sizeof(T) * 3 channels, then since we would like to traverse from height 0 ->
  height 1, we must traverse 5 widths worth of memory, hence sizeof(T) * 3 channels * 5 width units
  is required

  @param T The type of the memory buffer
  @param memory_buffer The memory buffer that will be traversed
  @param shape The shape of the memory buffer
  @param memory_layout The memory layout of the memory buffer
  @param byte_strides The strides of each dimension in bytes
  **/
  StridePointer3D(
      T* memory_buffer, StridePointer3D_Shape shape, StridePointer3D_MemoryLayout memory_layout,
      const std::array<size_t, kMaxDims>& byte_strides)
      : memory_buffer_{memory_buffer}, shape_{shape}, memory_layout_{memory_layout} {
    for (size_t i = 0; i < kMaxDims; ++i) {
      strides_[i] = byte_strides[i] / sizeof(T);
    }
    setStrideIndices();
  }

  /**
  This is identical to the std::array version of the constructor
  **/
  StridePointer3D(
      T* memory_buffer, StridePointer3D_Shape shape, StridePointer3D_MemoryLayout memory_layout,
      size_t byte_strides[kMaxDims])
      : memory_buffer_{memory_buffer}, shape_{shape}, memory_layout_{memory_layout} {
    for (size_t i = 0; i < kMaxDims; ++i) {
      strides_[i] = byte_strides[i] / sizeof(T);
    }
    setStrideIndices();
  }

  /**
  Returns a mutable reference at the specified (row, col, depth).

  NOTE: (row, col, depth) corresponds to (height, width, channel) indexing

  WARNING: this method does not do bounds checking

  @param row The row (height) index
  @param col The column (width) index
  @param depth The depth (channel) index
  **/
  __device__ __host__ T& operator()(uint32_t row, uint32_t col, uint32_t depth) {
    return memory_buffer_[computeFlattenedIndex(row, col, depth)];
  }

  /**
  Returns a mutable reference at the specified (row, col). This assumes the depth is 0.

  This is useful for processing, grayscale images, for example

  NOTE: (row, col) corresponds to (height, width) indexing

  WARNING: this method does not do bounds checking

  @param row The row (height) index
  @param col The column (width) index
  **/
  __device__ __host__ T& operator()(uint32_t row, uint32_t col) {
    return memory_buffer_[computeFlattenedIndex(row, col, 0)];
  }

  /**
  Returns a constant reference at the specified (row, col, depth).

  NOTE: (row, col, depth) corresponds to (height, width, channel) indexing

  WARNING: this method does not do bounds checking

  @param row The row (height) index
  @param col The column (width) index
  @param depth The depth (channel) index
  **/
  __device__ __host__ const T& operator()(uint32_t row, uint32_t col, uint32_t depth) const {
    return memory_buffer_[computeFlattenedIndex(row, col, depth)];
  }

  /**
  Returns a constant reference at the specified (row, col). This assumes the depth is 0.

  This is useful for processing, grayscale images, for example

  WARNING: this method does not do bounds checking

  NOTE: (row, col) corresponds to (height, width) indexing

  @param row The row (height) index
  @param col The column (width) index
  **/
  __device__ __host__ const T& operator()(uint32_t row, uint32_t col) const {
    return memory_buffer_[computeFlattenedIndex(row, col, 0)];
  }

  /// Returns the memory layout
  __device__ __host__ StridePointer3D_MemoryLayout memory_layout() const { return memory_layout_; }

  /// Returns the shape
  __device__ __host__ StridePointer3D_Shape shape() const { return shape_; }

  /// Returns the memory buffer
  __device__ __host__ T* pointer() const { return memory_buffer_; }

  /**
  Returns the memory buffer at the specified (row, col, depth).

  NOTE: (row, col, depth) corresponds to (height, width, channel) indexing

  WARNING: this method does not do bounds checking

  @param row The row (height) index
  @param col The column (width) index
  @param depth The depth (channel) index
  **/
  __device__ __host__ T* pointerAt(uint32_t row, uint32_t col, uint32_t depth) const {
    return memory_buffer_ + computeFlattenedIndex(row, col, depth);
  }

  /**
  Returns the memory buffer at the specified (row, col). This assumes the depth is 0.

  This is useful for processing, grayscale images, for example

  NOTE: (row, col) corresponds to (height, width) indexing

  WARNING: this method does not do bounds checking

  @param row The row (height) index
  @param col The column (width) index
  **/
  __device__ __host__ T* pointerAt(uint32_t row, uint32_t col) const {
    return memory_buffer_ + computeFlattenedIndex(row, col, 0);
  }

  /**
  Returns the flattened index at the specified (row, col, depth).

  NOTE: (row, col, depth) corresponds to (height, width, channel) indexing

  @param row The row (height) index
  @param col The column (width) index
  @param depth The depth (channel) index
  **/
  __device__ __host__ size_t indexAt(uint32_t row, uint32_t col, uint32_t depth) const {
    return computeFlattenedIndex(row, col, depth);
  }

  /**
  Returns the flattened index at the specified (row, col). This assumes the depth is 0.

  This is useful for processing, grayscale images, for example

  NOTE: (row, col) corresponds to (height, width) indexing

  @param row The row (height) index
  @param col The column (width) index
  **/
  __device__ __host__ size_t indexAt(uint32_t row, uint32_t col) const {
    return computeFlattenedIndex(row, col, 0);
  }

 private:
  /// The raw memory buffer
  T* memory_buffer_{};

  /// The shape of the memory buffer
  StridePointer3D_Shape shape_{};

  /// The memory layout of the memory buffer
  StridePointer3D_MemoryLayout memory_layout_;

  /// The stride
  size_t strides_[kMaxDims];

  /// Maps a dimension to the stride array
  struct StrideIndexMapping {
    /// The height index mapping
    size_t height_idx{};

    /// The width index mapping
    size_t width_idx{};

    /// The channel index mapping
    size_t channel_idx{};
  } stride_index_mapping_;

  /**
  Computes the strides of the data based on the memory layout,
  assuming no extra memory offset or padding

  NOTE: the strides are divided by sizeof(T).
  This is why, for example, strides_[2] = 1

  The computed strides can be interpreted as follows:
  - strides_[2] -> the stride required to traverse along the least significant dimension
  - strides_[1] -> the stride required to traverse along the 2nd least significant dimension
  - strides_[0] -> the stride required to traverse along the 3rd least significant dimension
  **/
  void computeStrides() {
    switch (memory_layout_) {
      case StridePointer3D_MemoryLayout::kNHWC:
      case StridePointer3D_MemoryLayout::kHWC:
        strides_[2] = 1;
        strides_[1] = shape_.channels * strides_[2];
        strides_[0] = shape_.width * strides_[1];
        break;
      case StridePointer3D_MemoryLayout::kNCHW:
        strides_[2] = 1;
        strides_[1] = shape_.width * strides_[2];
        strides_[0] = shape_.height * strides_[1];
        break;
    }
  }

  /**
  Sets the index stride mapping based on the memory layout.

  For NHWC and HWC, this is height = 0, width = 1, channel = 2
  For NCHW, this is channel = 0, width = 1, height = 2
  **/
  void setStrideIndices() {
    switch (memory_layout_) {
      case StridePointer3D_MemoryLayout::kNHWC:
      case StridePointer3D_MemoryLayout::kHWC:
        stride_index_mapping_.height_idx = 0;
        stride_index_mapping_.width_idx = 1;
        stride_index_mapping_.channel_idx = 2;
        break;
      case StridePointer3D_MemoryLayout::kNCHW:
        stride_index_mapping_.channel_idx = 0;
        stride_index_mapping_.height_idx = 1;
        stride_index_mapping_.width_idx = 2;
        break;
    }
  }

  /**
  Flattens a 3 dimensional index into a 1 dimension one

  @param row The row (height) index
  @param col the col (width) index
  @param depth The depth (channel) index
  **/
  __device__ __host__ size_t
  computeFlattenedIndex(uint32_t row, uint32_t col, uint32_t depth) const {
    return row * strides_[stride_index_mapping_.height_idx] +
           col * strides_[stride_index_mapping_.width_idx] +
           depth * strides_[stride_index_mapping_.channel_idx];
  }
};

#define ISAAC_DECLARE_STRIDE_POINTER_3D_TYPES(T, S) using StridePointer3D##S = StridePointer3D<T>;

ISAAC_DECLARE_STRIDE_POINTER_3D_TYPES(uint8_t, ub)
ISAAC_DECLARE_STRIDE_POINTER_3D_TYPES(uint16_t, ui16)
ISAAC_DECLARE_STRIDE_POINTER_3D_TYPES(int, i)
ISAAC_DECLARE_STRIDE_POINTER_3D_TYPES(double, d)
ISAAC_DECLARE_STRIDE_POINTER_3D_TYPES(float, f)

#undef ISAAC_DECLARE_STRIDE_POINTER_3D_TYPES

}  // namespace isaac
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_ENGINE_GEMS_CUDA_UTILS_STRIDE_POINTER_3D_HPP_
