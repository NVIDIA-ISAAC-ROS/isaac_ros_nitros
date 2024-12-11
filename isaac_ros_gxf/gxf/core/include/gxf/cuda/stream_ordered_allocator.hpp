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
#ifndef NVIDIA_GXF_STREAM_ORDERED_ALLOCATOR_HPP_
#define NVIDIA_GXF_STREAM_ORDERED_ALLOCATOR_HPP_

#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#include <string>
#include <unordered_map>

#include "common/assert.hpp"
#include "common/logger.hpp"
#include "gxf/core/parameter.hpp"
#include "gxf/cuda/cuda_allocator.hpp"
#include "gxf/std/resources.hpp"

namespace nvidia {
namespace gxf {

// An allocator which uses cudaMalloc/cudaMallocHost dynamically without a pool.
// Does not provide bounded execution times.
class StreamOrderedAllocator : public CudaAllocator {
 public:
  StreamOrderedAllocator() = default;
  ~StreamOrderedAllocator() = default;

  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t is_available_abi(uint64_t size) override;
  gxf_result_t allocate_abi(uint64_t size, int32_t storage_type, void** pointer) override;
  gxf_result_t allocate_async_abi(uint64_t size, void** pointer,
                                  cudaStream_t stream) override;
  gxf_result_t free_async_abi(void* pointer, cudaStream_t stream) override;
  gxf_result_t free_abi(void* pointer) override;

  Expected<size_t> get_pool_size(MemoryStorageType type) const override;

 private:
  Resource<Handle<GPUDevice>> gpu_device_;
  Parameter<std::string> release_threshold_;
  Parameter<std::string> device_memory_initial_size_;
  Parameter<std::string> device_memory_max_size_;

  std::unordered_map<void*, size_t> pool_map_;

  AllocatorStage stage_{AllocatorStage::kUninitialized};
  std::mutex mutex_;
  cudaStream_t stream_;
  cudaMemPool_t memory_pool_;
  cudaMemPoolProps pool_props_ = {};
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STREAM_ORDERED_ALLOCATOR_HPP_
