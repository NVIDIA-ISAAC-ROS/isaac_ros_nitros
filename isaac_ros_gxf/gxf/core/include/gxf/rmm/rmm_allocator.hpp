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
#ifndef NVIDIA_GXF_RMM_RMM_ALLOCATOR_HPP_
#define NVIDIA_GXF_RMM_RMM_ALLOCATOR_HPP_

#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "common/assert.hpp"
#include "common/logger.hpp"
#include "gxf/core/parameter.hpp"
#include "gxf/cuda/cuda_allocator.hpp"
#include "gxf/std/resources.hpp"
#include "rmm/device_buffer.hpp"
#include "rmm/mr/device/cuda_async_memory_resource.hpp"
#include "rmm/mr/device/pool_memory_resource.hpp"
#include "rmm/mr/host/pinned_memory_resource.hpp"
namespace nvidia {
namespace gxf {

// RMM based device memory allocator
class RMMAllocator : public CudaAllocator {
 public:
  RMMAllocator() = default;
  ~RMMAllocator() = default;

  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t is_available_abi(uint64_t size) override;
  gxf_result_t is_rmm_available_abi(uint64_t size, MemoryStorageType type);
  gxf_result_t allocate_abi(uint64_t size, int32_t storage_type, void** pointer) override;
  gxf_result_t allocate_async_abi(uint64_t size, void** pointer,
                                  cudaStream_t stream) override;
  gxf_result_t free_async_abi(void* pointer, cudaStream_t stream) override;
  gxf_result_t free_abi(void* pointer) override;

  Expected<size_t> get_pool_size(MemoryStorageType type) const override;

 private:
  Parameter<std::string> device_memory_initial_size_;
  Parameter<std::string> device_memory_max_size_;
  Parameter<std::string> host_memory_initial_size_;
  Parameter<std::string> host_memory_max_size_;
  Resource<Handle<GPUDevice>> gpu_device_;

  AllocatorStage stage_{AllocatorStage::kUninitialized};
  std::mutex mutex_;
  cudaStream_t stream_ = {};
  size_t device_max_memory_pool_size_;
  size_t host_max_memory_pool_size_;

  // Create a CUDA memory resource
  std::unique_ptr<rmm::mr::cuda_memory_resource> device_mr;
  std::unique_ptr<rmm::mr::pool_memory_resource<rmm::mr::cuda_memory_resource>> pool_mr_device;
  std::unique_ptr<rmm::mr::pinned_memory_resource> pinned_mr;
  std::unique_ptr<rmm::mr::pool_memory_resource<rmm::mr::pinned_memory_resource>> pool_mr_host;

  std::unordered_map<void*, std::pair<std::size_t, MemoryStorageType>> pool_map = {};
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_RMM_RMM_ALLOCATOR_HPP_
