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
#ifndef NVIDIA_GXF_STD_MEMORY_BUFFER_HPP_
#define NVIDIA_GXF_STD_MEMORY_BUFFER_HPP_

#include <utility>

#include "common/byte.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/handle.hpp"
#include "gxf/std/allocator.hpp"

namespace nvidia {
namespace gxf {

class MemoryBuffer {
 public:
  MemoryBuffer() = default;
  MemoryBuffer(const MemoryBuffer&) = delete;
  MemoryBuffer& operator=(const MemoryBuffer&) = delete;

  MemoryBuffer(MemoryBuffer&& other) { *this = std::move(other); }

  MemoryBuffer& operator=(MemoryBuffer&& other) {
    size_ = other.size_;
    storage_type_ = other.storage_type_;
    pointer_ = other.pointer_;
    release_func_ =  std::move(other.release_func_);

    other.pointer_ = nullptr;
    other.release_func_ = nullptr;

    return *this;
  }

  // Type of the callback function to release memory passed to the MemoryBuffer
  // using the wrapMemory method
  using release_function_t = std::function<Expected<void> (void* pointer)>;

  Expected<void> freeBuffer() {
    if (release_func_ && pointer_) {
      const Expected<void> result = release_func_(pointer_);
      if (!result) { return ForwardError(result); }

      release_func_ = nullptr;
      pointer_ = nullptr;
      size_ = 0;
    }

    return Success;
  }

  ~MemoryBuffer() { freeBuffer(); }

  Expected<void> resize(Handle<Allocator> allocator, uint64_t size,
                         MemoryStorageType storage_type) {
    const auto result = freeBuffer();
    if (!result) {
      GXF_LOG_ERROR("Failed to free memory. Error code: %s", GxfResultStr(result.error()));
      return ForwardError(result);
    }

    const auto maybe = allocator->allocate(size, storage_type);
    if (!maybe) {
      GXF_LOG_ERROR("%s Failed to allocate %d size of memory of type %d. Error code: %s",
                      allocator->name(), size, storage_type, GxfResultStr(maybe.error()));
      return ForwardError(maybe);
    }

    storage_type_ = storage_type;
    pointer_ = maybe.value();
    size_ = size;

    release_func_ = [allocator] (void *data) {
      return allocator->free(reinterpret_cast<byte*>(data));
    };

    return Success;
  }

  // Wrap existing memory inside the MemoryBuffer. A callback function of type
  // release_function_t may be passed that will be called when the MemoryBuffer
  // wants to release the memory.
  Expected<void> wrapMemory(void* pointer, uint64_t size,
                            MemoryStorageType storage_type,
                            release_function_t release_func) {
    const auto result = freeBuffer();
    if (!result) { return ForwardError(result); }

    storage_type_ = storage_type;
    pointer_ = reinterpret_cast<byte*>(pointer);
    size_ = size;
    release_func_ = release_func;

    return Success;
  }

  // The type of memory where the data is stored.
  MemoryStorageType storage_type() const { return storage_type_; }

  // Raw pointer to the first byte of elements stored in the buffer.
  byte* pointer() const { return pointer_; }

  // Size of buffer contents in bytes
  uint64_t size() const { return size_; }

 private:
  uint64_t size_ = 0;
  byte* pointer_ = nullptr;
  MemoryStorageType storage_type_ = MemoryStorageType::kHost;
  release_function_t release_func_ = nullptr;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_MEMORY_BUFFER_HPP_
