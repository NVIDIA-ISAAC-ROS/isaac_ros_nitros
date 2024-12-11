// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_CUDA_ALLOCATOR_HPP
#define NVIDIA_GXF_CUDA_ALLOCATOR_HPP

#include <cuda_runtime.h>
#include <string>

#include "gxf/std/allocator.hpp"

namespace nvidia {
namespace gxf {

// Provides allocation and deallocation of memory.
class CudaAllocator : public Allocator {
 public:
  virtual Expected<byte*> allocate_async(uint64_t size, cudaStream_t stream);
  virtual Expected<void> free_async(byte* pointer, cudaStream_t stream);
  virtual gxf_result_t allocate_async_abi(uint64_t, void**, cudaStream_t) = 0;
  virtual gxf_result_t free_async_abi(void*, cudaStream_t) = 0;
  virtual Expected<size_t> get_pool_size(MemoryStorageType type) const = 0;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_CUDA_ALLOCATOR_HPP
