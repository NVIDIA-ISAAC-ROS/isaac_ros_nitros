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
#ifndef NVIDIA_GXF_SERIALIZATION_SERIALIZATION_BUFFER_HPP_
#define NVIDIA_GXF_SERIALIZATION_SERIALIZATION_BUFFER_HPP_

#include <mutex>

#include "gxf/serialization/endpoint.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/memory_buffer.hpp"

namespace nvidia {
namespace gxf {

// Buffer to hold serialized data
class SerializationBuffer : public Endpoint {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override { return ToResultCode(buffer_.freeBuffer()); }

  gxf_result_t write_abi(const void* data, size_t size, size_t* bytes_written) override;
  gxf_result_t read_abi(void* data, size_t size, size_t* bytes_read) override;

  // Resizes the buffer
  gxf::Expected<void> resize(size_t size);

  // Returns a read-only pointer to buffer data
  const byte* data() const { return buffer_.pointer(); }
  // Returns the capacity of the buffer
  size_t capacity() const { return buffer_.size(); }
  // Returns the number of bytes written to the buffer
  size_t size() const;
  // Resets buffer for sequential access
  void reset();

 private:
  Parameter<Handle<Allocator>> allocator_;
  Parameter<size_t> buffer_size_;

  // Data buffer
  MemoryBuffer buffer_;
  // Offset for sequential writes
  size_t write_offset_;
  // Offset for sequential reads
  size_t read_offset_;
  // Mutex to guard concurrent access
  mutable std::mutex mutex_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_SERIALIZATION_BUFFER_HPP_
