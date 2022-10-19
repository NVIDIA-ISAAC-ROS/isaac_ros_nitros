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
#ifndef NVIDIA_GXF_SERIALIZATION_STD_COMPONENT_SERIALIZER_HPP_
#define NVIDIA_GXF_SERIALIZATION_STD_COMPONENT_SERIALIZER_HPP_

#include "common/endian.hpp"
#include "gxf/serialization/component_serializer.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace gxf {

// Serializer that supports serializaing Timestamps, Tensors, and integer components
// Valid for sharing data between devices with the same endianness
class StdComponentSerializer : public ComponentSerializer {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override { return GXF_SUCCESS; }

 private:
  // Configures all serializer functions
  Expected<void> configureSerializers();
  // Configures all deserializer functions
  Expected<void> configureDeserializers();
  // Serializes a nvidia::gxf::Timestamp
  Expected<size_t> serializeTimestamp(Timestamp timestamp, Endpoint* endpoint);
  // Deserializes a nvidia::gxf::Timestamp
  Expected<Timestamp> deserializeTimestamp(Endpoint* endpoint);
  // Serializes a nvidia::gxf::Tensor
  Expected<size_t> serializeTensor(const Tensor& tensor, Endpoint* endpoint);
  // Deserializes a nvidia::gxf::Tensor
  Expected<Tensor> deserializeTensor(Endpoint* endpoint);
  // Serializes an integer
  template <typename T>
  Expected<size_t> serializeInteger(T value, Endpoint* endpoint) {
    if (!endpoint) {
      return Unexpected{GXF_ARGUMENT_NULL};
    }
    T encoded = EncodeLittleEndian(value);
    return endpoint->writeTrivialType(&encoded);
  }
  // Deserializes an integer
  template <typename T>
  Expected<T> deserializeInteger(Endpoint* endpoint) {
    if (!endpoint) {
      return Unexpected{GXF_ARGUMENT_NULL};
    }
    T encoded;
    return endpoint->readTrivialType(&encoded)
        .and_then([&]() { return DecodeLittleEndian(encoded); });
  }

  Parameter<Handle<Allocator>> allocator_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_STD_COMPONENT_SERIALIZER_HPP_
