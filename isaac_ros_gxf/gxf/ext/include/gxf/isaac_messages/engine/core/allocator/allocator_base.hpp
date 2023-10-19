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

#include <cstddef>

#include "engine/core/byte.hpp"

namespace nvidia {
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
}  // namespace nvidia
