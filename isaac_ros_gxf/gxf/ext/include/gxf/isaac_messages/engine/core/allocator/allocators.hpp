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

#include "engine/core/allocator/allocator_base.hpp"
#include "engine/core/byte.hpp"

namespace nvidia {
namespace isaac {

// Provides a globally available allocator to allocate CPU memory.
struct CpuAllocator {
  static byte* Allocate(size_t size);
  static void Deallocate(byte* pointer, size_t size);
};

// Gets the currently used allocator for CPU memory allocation.
AllocatorBase* GetCpuAllocator();

// Provides a globally available allocator to allocate CUDA memory.
struct CudaAllocator {
  static byte* Allocate(size_t size);
  static void Deallocate(byte* pointer, size_t size);
};

// Gets the currently used allocator for CUDA memory allocation.
AllocatorBase* GetCudaAllocator();

}  // namespace isaac
}  // namespace nvidia
