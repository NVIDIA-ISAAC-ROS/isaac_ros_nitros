// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2020-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_BLOCK_MEMORY_POOL_HPP
#define NVIDIA_GXF_STD_BLOCK_MEMORY_POOL_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>

#include "gxf/std/allocator.hpp"
#include "gxf/std/resources.hpp"

namespace nvidia {
namespace gxf {

class FixedPoolUint64;

// A memory pools which provides a maximum number of equally sized blocks of
// memory.
class BlockMemoryPool : public Allocator {
 public:
  BlockMemoryPool();
  ~BlockMemoryPool();

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t is_available_abi(uint64_t size) override;
  gxf_result_t allocate_abi(uint64_t size, int32_t type, void** pointer) override;
  gxf_result_t free_abi(void* pointer) override;
  gxf_result_t deinitialize() override;
  uint64_t block_size_abi() const override;

  // Returns the storage type of the memory blocks
  MemoryStorageType storage_type() const {
    return static_cast<MemoryStorageType>(storage_type_.get());
  }

  // Returns the total number of blocks
  uint64_t num_blocks() const {
    return num_blocks_.get();
  }

 private:
  Parameter<int32_t> storage_type_;
  Parameter<uint64_t> block_size_;
  Parameter<uint64_t> num_blocks_;
  Resource<Handle<GPUDevice>> gpu_device_;

  void* pointer_;
  std::unique_ptr<FixedPoolUint64> stack_;
  std::mutex stack_mutex_;
  // Holds lifecycle stage of the allocator
  std::atomic<AllocatorStage> stage_{AllocatorStage::kUninitialized};
  int32_t dev_id_ = -1;
};

}  // namespace gxf
}  // namespace nvidia

#endif
