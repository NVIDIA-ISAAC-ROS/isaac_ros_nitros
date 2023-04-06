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
#ifndef NVIDIA_GXF_SERIALIZATION_ENDPOINT_HPP_
#define NVIDIA_GXF_SERIALIZATION_ENDPOINT_HPP_

#include "gxf/core/component.hpp"

namespace nvidia {
namespace gxf {

// Interface for exchanging data external to an application graph
class Endpoint : public Component {
 public:
  virtual ~Endpoint() = default;

  // Returns `GXF_SUCCESS` if endpoint can be written to
  virtual gxf_result_t is_write_available_abi() { return GXF_SUCCESS; }
  // Returns `GXF_SUCCESS` if endpoint can be read from
  virtual gxf_result_t is_read_available_abi() { return GXF_SUCCESS; }
  // Writes data to the endpoint and returns the number of bytes written
  virtual gxf_result_t write_abi(const void* data, size_t size, size_t* bytes_written) = 0;
  // Reads data from the endpoint and returns the number of bytes read
  virtual gxf_result_t read_abi(void* data, size_t size, size_t* bytes_read) = 0;

  // C++ API wrappers
  bool isWriteAvailable();
  bool isReadAvailable();
  Expected<size_t> write(const void* data, size_t size);
  Expected<size_t> read(void* data, size_t size);

  // Writes an object of type T to the endpoint
  template <typename T>
  Expected<size_t> writeTrivialType(const T* object) { return write(object, sizeof(T)); }

  // Reads an object of type T from the endpoint
  template <typename T>
  Expected<size_t> readTrivialType(T* object) { return read(object, sizeof(T)); }
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_ENDPOINT_HPP_
