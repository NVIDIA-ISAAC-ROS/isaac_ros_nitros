/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef NVIDIA_GXF_TEST_COMPONENTS_MOCK_ALLOCATOR_HPP_
#define NVIDIA_GXF_TEST_COMPONENTS_MOCK_ALLOCATOR_HPP_

#include <shared_mutex>
#include <unordered_map>

#include "gxf/std/allocator.hpp"

namespace nvidia {
namespace gxf {
namespace test {

// Memory allocator used to facilitate testing
class MockAllocator : public Allocator {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t is_available_abi(uint64_t size) override;
  gxf_result_t allocate_abi(uint64_t size, int32_t type, void** pointer) override;
  gxf_result_t free_abi(void* pointer) override;

 private:
  // Structure for organizing blocks of allocated memory
  struct MemoryBlock {
    MemoryStorageType storage_type;
    size_t size;
  };
  // Structure to organize performance metrics
  struct Metrics {
    size_t blocks;
    size_t allocation;
  };

  // Verifies memory allocation is within bounds
  Expected<void> verifyAllocation(size_t size, MemoryStorageType type);
  // Looks up memory storage type for memory deallocation
  Expected<MemoryStorageType> verifyDeallocation(void* pointer);
  // Checks if all memory was deallocated
  Expected<void> checkForMemoryLeaks();
  // Prints the number of bytes allocated
  void printMemoryUsage();

  Parameter<bool> ignore_memory_leak_;
  Parameter<bool> fail_on_free_;
  Parameter<size_t> max_block_size_;
  Parameter<size_t> max_host_allocation_;
  Parameter<size_t> max_device_allocation_;
  Parameter<size_t> max_system_allocation_;

  // Table for tracking storage type and size for each memory block
  std::unordered_map<void*, MemoryBlock> memory_blocks_;
  // Performance metrics
  std::unordered_map<MemoryStorageType, Metrics> metrics_;
  // Mutex for guarding concurrent access to table and metrics
  std::shared_timed_mutex mutex_;
};

}  // namespace test
}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_TEST_COMPONENTS_MOCK_ALLOCATOR_HPP_
