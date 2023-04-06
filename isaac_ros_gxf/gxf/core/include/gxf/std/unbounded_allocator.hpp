/*
 * SPDX-FileCopyrightText: Copyright (c) 2020 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef NVIDIA_GXF_STD_UNBOUNDED_ALLOCATOR_HPP_
#define NVIDIA_GXF_STD_UNBOUNDED_ALLOCATOR_HPP_

#include <mutex>
#include <set>

#include "gxf/std/allocator.hpp"

namespace nvidia {
namespace gxf {

// An allocater which uses cudaMalloc/cudaMallocHost dynamically without a pool. Does not provide
// bounded execution times.
class UnboundedAllocator : public Allocator {
 public:
  UnboundedAllocator() = default;
  ~UnboundedAllocator() = default;

  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t is_available_abi(uint64_t size) override;
  gxf_result_t allocate_abi(uint64_t size, int32_t storage_type, void** pointer) override;
  gxf_result_t free_abi(void* pointer) override;

 private:
  // Mutex to protect cuda_blocks_
  std::mutex mutex_;
  // Remember the blocks so that we know how to delete them
  std::set<void*> cuda_blocks_;
  std::set<void*> cuda_host_blocks_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_UNBOUNDED_ALLOCATOR_HPP_
