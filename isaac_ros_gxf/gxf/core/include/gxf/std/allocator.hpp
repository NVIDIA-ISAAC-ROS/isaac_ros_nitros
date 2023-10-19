// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_STD_ALLOCATOR_HPP
#define NVIDIA_GXF_STD_ALLOCATOR_HPP

#include "common/byte.hpp"
#include "gxf/core/component.hpp"

namespace nvidia {
namespace gxf {

enum struct MemoryStorageType { kHost = 0, kDevice = 1, kSystem = 2 };

// Lifecycle stages of an allocator
enum struct AllocatorStage : uint8_t {
  kUninitialized = 0,
  kInitializationInProgress = 1,
  kInitialized = 2,
  kDeinitializationInProgress = 3,
};

// Provides allocation and deallocation of memory.
struct Allocator : public Component {
  virtual ~Allocator() = default;

  virtual gxf_result_t is_available_abi(uint64_t size) = 0;
  virtual gxf_result_t allocate_abi(uint64_t size, int32_t type, void** pointer) = 0;
  virtual gxf_result_t free_abi(void* pointer) = 0;
  virtual uint64_t block_size_abi() const;

  // Returns true if the allocator can provide a memory block with the given size.
  bool is_available(uint64_t size);

  // Allocates a memory block with the given size.
  Expected<byte*> allocate(uint64_t size, MemoryStorageType type);

  // Frees the given memory block.
  Expected<void> free(byte* pointer);

  // Get the block size of this allocator, defaults to 1 for byte-based allocators
  uint64_t block_size() const;

  // Get the string value of allocator status
  const char* allocator_stage_str(AllocatorStage stage) const;
};

}  // namespace gxf
}  // namespace nvidia

#endif
