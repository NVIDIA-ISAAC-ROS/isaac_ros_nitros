/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstddef>

#include "engine/core/byte.hpp"

namespace isaac {

// Base class for allocators which allocate and deallocate blocks of memory.
class AllocatorBase {
 public:
  // Type for a raw pointer to a block of memory
  using pointer_t = byte*;

  virtual ~AllocatorBase() = default;

  // Allocates memory for at least N instances of the given type
  template <typename T>
  T* allocate(size_t count) {
    return reinterpret_cast<T*>(allocateBytes(sizeof(T) * count));
  }

  // Deallocates a block of memory previously allocated with `allocate`. Need to be passed the same
  // instance count used during the call to `allocate`.
  template <typename T>
  void deallocate(T* pointer, size_t count) {
    deallocateBytes(reinterpret_cast<pointer_t>(pointer), sizeof(T) * count);
  }

  // Allocates memory for at least the given number of bytes
  virtual pointer_t allocateBytes(size_t size) = 0;

  // Deallocates a block of memory previously allocated with `allocate`. Need to be passed the same
  // byte count used during the call to `allocate`.
  virtual void deallocateBytes(pointer_t pointer, size_t size) = 0;
};

}  // namespace isaac
