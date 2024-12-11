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

#include <cuda_fp16.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/byte.hpp"
#include "dlpack/dlpack.h"
#include "gxf/core/component.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/complex.hpp"
#include "gxf/std/dlpack_utils.hpp"
#include "gxf/std/memory_buffer.hpp"

namespace nvidia {
namespace gxf {

// Type of parameters and other primitives
enum class PrimitiveType : int32_t {
  kCustom,
  kInt8,
  kUnsigned8,
  kInt16,
  kUnsigned16,
  kInt32,
  kUnsigned32,
  kInt64,
  kUnsigned64,
  kFloat32,
  kFloat64,
  kComplex64,
  kComplex128,
  kFloat16,
};

// Returns the size of each element of specific PrimitiveType as number of bytes.
// Returns 0 for kCustom.
uint64_t PrimitiveTypeSize(PrimitiveType primitive);

template <typename T>
struct PrimitiveTypeTraits;

#define GXF_PRIMITIVE_TYPE_TRAITS(TYPE, ENUM)                                                      \
  template <> struct PrimitiveTypeTraits<TYPE> {                                                   \
    static constexpr PrimitiveType value = PrimitiveType::ENUM;                                    \
    static constexpr size_t size = sizeof(TYPE);                                                   \
  };                                                                                               \

GXF_PRIMITIVE_TYPE_TRAITS(int8_t, kInt8);
GXF_PRIMITIVE_TYPE_TRAITS(uint8_t, kUnsigned8);
GXF_PRIMITIVE_TYPE_TRAITS(int16_t, kInt16);
GXF_PRIMITIVE_TYPE_TRAITS(uint16_t, kUnsigned16);
GXF_PRIMITIVE_TYPE_TRAITS(int32_t, kInt32);
GXF_PRIMITIVE_TYPE_TRAITS(uint32_t, kUnsigned32);
GXF_PRIMITIVE_TYPE_TRAITS(int64_t, kInt64);
GXF_PRIMITIVE_TYPE_TRAITS(uint64_t, kUnsigned64);
GXF_PRIMITIVE_TYPE_TRAITS(__half, kFloat16);
GXF_PRIMITIVE_TYPE_TRAITS(float, kFloat32);
GXF_PRIMITIVE_TYPE_TRAITS(double, kFloat64);
GXF_PRIMITIVE_TYPE_TRAITS(complex64, kComplex64);
GXF_PRIMITIVE_TYPE_TRAITS(complex128, kComplex128);

// Type to hold the shape of a tensor
class Shape {
 public:
  // The maximum possible rank of the tensor.
  static constexpr uint32_t kMaxRank = 8;

  // Initializes an empty rank-0 tensor.
  Shape() : rank_(0) {}

  // Initializes a shape object with the given dimensions.
  Shape(std::initializer_list<int32_t> dimensions)
      : rank_(0) {
    for (int32_t dimension : dimensions) {
      if (rank_ == kMaxRank) {
        return;
      }
      dimensions_[rank_++] = dimension;
    }
  }

  // Creates shape from vector
  Shape(const std::vector<int32_t>& dimensions)
      : rank_(0) {
    for (int32_t dimension : dimensions) {
      if (rank_ == kMaxRank) {
        return;
      }
      dimensions_[rank_++] = dimension;
    }
  }

  // Creates shape from array
  Shape(const std::array<int32_t, Shape::kMaxRank>& dims, uint32_t rank)
    : rank_(rank), dimensions_(dims) {
  }

  // Creates shape from array with correct rank
  template <size_t N>
  Shape(const std::array<int32_t, N>& dims) : rank_(N) {
    static_assert(N < kMaxRank, "Invalid rank");
    for (size_t i = 0; i < N; i++) {
      dimensions_[i] = dims[i];
    }
  }

  // The rank of the tensor
  uint32_t rank() const { return rank_; }

  // The total number of elements in the tensor. Note: this is not the same as the number of bytes
  // required in memory.
  uint64_t size() const {
    uint64_t element_count = 1;
    for (size_t i = 0; i < rank_; i++) {
      element_count *= dimensions_[i];
    }
    return rank_ == 0 ? 0 : element_count;
  }

  // Gets the i-th dimension of the tensor.
  // Special cases:
  //  If the rank is 0 the function always returns 0.
  //  If 'index' is greater or equal than the rank the function returns 1.
  int32_t dimension(uint32_t index) const {
    if (rank_ == 0) {
      return 0;
    } else if (index >= rank_) {
      return 1;
    } else {
      return dimensions_[index];
    }
  }

  bool operator== (const Shape& other) const {
    if (rank_ != other.rank_) { return false; }
    for (uint32_t i = 0; i < rank_; ++i) {
      if (dimensions_[i] != other.dimensions_[i]) { return false; }
    }
    return true;
  }

  bool operator!= (const Shape& other) const {
    return !(*this == other);
  }

  // Check whether shape is valid
  bool valid() const {
    for (uint32_t i = 0; i < rank_; ++i) {
      if (dimensions_[i] <= 0) { return false; }
    }
    return true;
  }

 private:
  uint32_t rank_ = 0;
  std::array<int32_t, kMaxRank> dimensions_;
};

/**
 * @brief Class that wraps a DLManagedTensor with a memory data reference.
 *
 * This class is used to wrap a DLManagedTensor (`tensor`) with a shared pointer to the memory
 * data (`memory_ref`). This class also holds shape and strides data in the integer type needed by
 * the DLTensor object since this does not exist as contiguous int64_t on the Tensor object
 * itself. It should also be noted that DLPack `dl_strides` stored here are in number of elements
 * while those returned by `Tensor::stride` are in bytes rather than elements.
 *
 * The DLPack protocol is designed so that the producer (GXF in the case of `toDLPack` or
 * `toDLManagedTensorContext`) continues to own the data. When the borrowing framework no longer
 * needs the tensor, it should call the deleter to notify the producer that the resource is no
 * longer needed.
 *
 * See: https://dmlc.github.io/dlpack/latest/c_api.html#_CPPv415DLManagedTensor
 */
struct DLManagedTensorContext {
  DLManagedTensor tensor;            ///< The DLManagedTensor to wrap.
  std::shared_ptr<void> memory_ref;  ///< The memory data reference.

  std::vector<int64_t> dl_shape;    ///< Shape of the DLTensor.
  std::vector<int64_t> dl_strides;  ///< Strides of the DLTensor.
};

/**
 * @brief Class to wrap the deleter of a DLManagedTensor.
 *
 * This class is used with DLManagedTensorContext class to wrap the DLManagedTensor.
 *
 * A shared pointer to this class in DLManagedTensorContext class is used as the deleter of the
 * DLManagedTensorContext::memory_ref.
 *
 * When the last reference to the DLManagedTensorContext object is released,
 * DLManagedTensorContext::memory_ref will also be destroyed, which will call the deleter function
 * of the DLManagedTensor object. This allows setting release_func to nullptr when calling
 * wrapTensor within fromDLPack.
 *
 */
class DLManagedMemoryBuffer {
 public:
  explicit DLManagedMemoryBuffer(DLManagedTensor* self);
  ~DLManagedMemoryBuffer();

 private:
  DLManagedTensor* self_ = nullptr;
};

// A component which holds a single tensor. Multiple tensors can be added to one
// entity to create a map of tensors. The component name can be used as key.
class Tensor {
 public:
  typedef std::array<uint64_t, Shape::kMaxRank> stride_array_t;

  Tensor() = default;

  ~Tensor() {
    memory_buffer_.freeBuffer();  // FIXME(V2) error code?
    dl_ctx_.reset();
    element_count_ = 0;
    shape_ = Shape();
  }

  Tensor(const Tensor&) = delete;

  Tensor(Tensor&& other) {
    *this = std::move(other);
  }

  // zero-copy initialization from an existing DLPack DLManagedTensor (C API struct)
  explicit Tensor(const DLManagedTensor* dl_managed_tensor_ptr);

  // zero-copy initialization from an existing DLManagedTensorContext (C++-style DLPack wrapper)
  explicit Tensor(std::shared_ptr<DLManagedTensorContext> dl_ctx);

  Tensor& operator=(const Tensor&) = delete;

  Tensor& operator=(Tensor&& other) {
    shape_ = other.shape_;
    element_count_ = other.element_count_;
    element_type_ = other.element_type_;
    bytes_per_element_ = other.bytes_per_element_;
    strides_ = std::move(other.strides_);
    memory_buffer_ = std::move(other.memory_buffer_);
    dl_ctx_ = std::move(other.dl_ctx_);

    return *this;
  }

  // The type of memory where the tensor data is stored.
  MemoryStorageType storage_type() const { return memory_buffer_.storage_type(); }

  // The shape of the dimensions holds the rank and the dimensions.
  const Shape& shape() const { return shape_; }

  // The rank of the tensor.
  uint32_t rank() const { return shape_.rank(); }

  // The scalar type of elements stored in the tensor
  PrimitiveType element_type() const { return element_type_; }

  // Number of bytes stored per element
  uint64_t bytes_per_element() const { return bytes_per_element_; }

  // Total number of elements stored in the tensor.
  uint64_t element_count() const { return element_count_; }

  // Size of tensor contents in bytes
  size_t size() const { return memory_buffer_.size(); }

  // Raw pointer to the first byte of elements stored in the tensor.
  byte* pointer() const { return memory_buffer_.pointer(); }

  // Move the memory buffer
  MemoryBuffer move_buffer() { return std::move(memory_buffer_); }

  // Gets a pointer to the first element stored in this tensor. Requested type must match the
  // tensor element type.
  template <typename T>
  Expected<T*> data() {
    if (element_type_ != PrimitiveType::kCustom && PrimitiveTypeTraits<T>::value != element_type_) {
      return Unexpected{GXF_INVALID_DATA_FORMAT};
    }
    return reinterpret_cast<T*>(memory_buffer_.pointer());
  }

  // Gets a pointer to the first element stored in this tensor. Requested type must match the
  // tensor element type.
  template <typename T>
  Expected<const T*> data() const {
    if (element_type_ != PrimitiveType::kCustom && PrimitiveTypeTraits<T>::value != element_type_) {
      return Unexpected{GXF_INVALID_DATA_FORMAT};
    }
    return reinterpret_cast<const T*>(memory_buffer_.pointer());
  }

  // Changes the shape and type of the tensor. Uses a primitive type and dense memory layot.
  // Memory will be allocated with the given allocator.
  template <typename T>
  Expected<void> reshape(const Shape& shape, MemoryStorageType storage_type,
                         Handle<Allocator> allocator) {
    return reshapeCustom(shape, PrimitiveTypeTraits<T>::value, PrimitiveTypeTraits<T>::size,
                         Unexpected{GXF_UNINITIALIZED_VALUE}, storage_type, allocator);
  }
  // Changes the shape and type of the tensor. Memory will be allocated with the given allocator
  // strides: The number of bytes that each slide takes for each dimension (alignment).
  //          Use ComputeStrides() to calculate it.
  Expected<void> reshapeCustom(const Shape& shape,
                               PrimitiveType element_type, uint64_t bytes_per_element,
                               Expected<stride_array_t> strides,
                               MemoryStorageType storage_type, Handle<Allocator> allocator);

  // Type of the callback function to release memory passed to the tensor using the
  // wrapMemory method
  using release_function_t = MemoryBuffer::release_function_t;

  // Wrap existing memory inside the tensor. A callback function of type release_function_t
  // may be passed that will be called when the Tensor wants to release the memory.
  Expected<void> wrapMemory(const Shape& shape,
                            PrimitiveType element_type, uint64_t bytes_per_element,
                            Expected<stride_array_t> strides,
                            MemoryStorageType storage_type, void* pointer,
                            release_function_t release_func, bool reset_dlpack = true);

  // Wraps an existing memory buffer element into the current tensor
  Expected<void> wrapMemoryBuffer(const Shape& shape,
                                        PrimitiveType element_type, uint64_t bytes_per_element,
                                        Expected<stride_array_t> strides,
                                        MemoryBuffer memory_buffer);

  // Permute the axes of a tensor.
  // Number of axes must match rank of tensor.
  // Note that since no data is moved, permuting a tensor can be
  // detrimental to performance (data not accessed from the same cache line).
  Expected<void> permute(const std::initializer_list<int32_t>& axes);

  // Attempts to reshape the array in memory by modifying the strides
  // Assumes tensor data is stored in row-major order.
  // The product of the element of the new shape must equal the product
  // of the tensor's dimensions
  // Adapted from Numpy:
  // https://github.com/numpy/numpy/blob/45bc13e6d922690eea43b9d807d476e0f243f836/
  // numpy/core/src/multiarray/shape.c#L371
  Expected<void> noCopyReshape(const std::initializer_list<int32_t>& new_shape);

  // Insert singleton dimension at specified dimension
  Expected<void> insertSingletonDim(uint32_t dimension);

  // Is tensor memory access contiguous?
  Expected<bool> isContiguous();

  // The size of data in bytes
  uint64_t bytes_size() {
    return shape_.dimension(0) * strides_[0];
  }

  // The stride of specified rank in bytes
  uint64_t stride(uint32_t index) const {
    if (index >= shape_.rank()) {
      return 0;
    }
    return strides_[index];
  }

  // Get a new DLManagedTensor* corresponding to the Tensor data.
  // An alternative is to use toDLManagedTensorContext instead which returns a shared pointer
  // that will automatically clean up resources once its reference count goes to zero.
  Expected<DLManagedTensor*> toDLPack();

  /**
   * @brief Get the internal DLManagedTensorContext of the Tensor.
   *
   * @return A shared pointer to the Tensor's DLManagedTensorContext.
   */
  Expected<std::shared_ptr<DLManagedTensorContext>&> toDLManagedTensorContext() {
    if (dl_ctx_ == nullptr) {
      auto status = initializeDLContext();
      if (!status) {
        GXF_LOG_ERROR(
            "Failed to initialize DLManagedTensorContext with code: %s, returning nullptr",
            GxfResultStr(status.error()));
        ForwardError(status);
      }
    }
    return dl_ctx_;
  }

  // Wrap an existing DLPack managed tensor as a Tensor.
  Expected<void> fromDLPack(const DLManagedTensor* dl_managed_tensor_ptr);

  // Wrap an existing shared C++ DLManagedTensorContext as a Tensor.
  Expected<void> fromDLPack(std::shared_ptr<DLManagedTensorContext> dl_ctx);

 protected:
  // This member will be populated when memory_buffer_ is initialized or updated, allowing it to
  // be used to provide the DLPack interface.
  std::shared_ptr<DLManagedTensorContext> dl_ctx_;  ///< The DLManagedTensorContext object.

 private:
  Shape shape_;
  uint64_t element_count_ = 0;
  PrimitiveType element_type_ = PrimitiveType::kUnsigned8;
  uint64_t bytes_per_element_ = 1;
  stride_array_t strides_;
  MemoryBuffer memory_buffer_;

  /**
   * @brief Get DLDevice object from the GXF Tensor.
   *
   * @return DLDevice object.
   */
  Expected<DLDevice> device() const;

  // populate DLPack data structures corresponding to the current Tensor
  Expected<void> initializeDLContext();

  // If dl_ctx_ has already been initialized, reset it and initialize a new one
  Expected<void> updateDLContext();

  // call wrapTensor using data from an existing DLManagedTensor
  Expected<void> wrapDLPack(const DLManagedTensor* dl_managed_tensor_ptr,
                            MemoryBuffer::release_function_t release_func = nullptr);
};

// Helper function to compute strides from Tensor shape, element size and non-trivial
// alignment step size for row dimension.
// The third rank from the end is assumed to be the row dimension.
Expected<Tensor::stride_array_t> ComputeRowStrides(const Shape& shape, uint32_t row_step_size,
                                                   const uint32_t bytes_per_element);

// Helper function to compute trivial strides from Tensor shape and element size
Tensor::stride_array_t ComputeTrivialStrides(const Shape& shape, const uint32_t bytes_per_element);

// Helper function to compute strides from steps (minimal number of bytes per slice on each rank)
Tensor::stride_array_t ComputeStrides(const Shape& shape,
                                      const Tensor::stride_array_t& stride_steps);

// Type to description a tensor used by 'CreateTensorMap'
struct TensorDescription {
  std::string name;
  MemoryStorageType storage_type;
  Shape shape;
  PrimitiveType element_type;
  uint64_t bytes_per_element;
  // array providing number of bytes for each slice on each rank
  Expected<Tensor::stride_array_t> strides = Unexpected{GXF_UNINITIALIZED_VALUE};
};

// Creates a new entity with a collection of named tensors
Expected<Entity> CreateTensorMap(gxf_context_t context, Handle<Allocator> pool,
                                 std::initializer_list<TensorDescription> descriptions,
                                 bool activate = true);

// Determine tensor shape from a DLTensor struct
Expected<Shape> ShapeFromDLTensor(const DLTensor* dl_tensor);

// Determine tensor strides from a DLTensor struct
Expected<Tensor::stride_array_t> StridesFromDLTensor(const DLTensor* dl_tensor);

// Determine the tensor memory storage type from a DLTensor struct
Expected<MemoryStorageType> MemoryStorageTypeFromDLTensor(const DLTensor* dl_tensor);

// Determine the tensor primitive data type from a DLDataType struct
Expected<PrimitiveType> PrimitiveTypeFromDLDataType(const DLDataType& dtype);

// Convert PrimitiveType to its corresponding DLDataType
Expected<DLDataType> PrimitiveTypeToDLDataType(const PrimitiveType& element_type,
                                               uint16_t lanes = 1);

}  // namespace gxf
}  // namespace nvidia
