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

#include "engine/core/allocator/allocator_base.hpp"
#include "engine/core/byte.hpp"

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
