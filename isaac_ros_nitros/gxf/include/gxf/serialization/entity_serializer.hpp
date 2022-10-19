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
#ifndef NVIDIA_GXF_SERIALIZATION_ENTITY_SERIALIZER_HPP_
#define NVIDIA_GXF_SERIALIZATION_ENTITY_SERIALIZER_HPP_

#include "gxf/core/component.hpp"
#include "gxf/serialization/endpoint.hpp"

namespace nvidia {
namespace gxf {

// Interface for serializing entities
class EntitySerializer : public Component {
 public:
  virtual ~EntitySerializer() = default;

  // Deserializes a received entity.
  // Returns the deserialized entity.
  virtual Expected<Entity> deserialize_entity_header_abi(
      Endpoint* endpoint) = 0;

  // Serializes entity and writes to endpoint
  // Returns the size of the serialized entity in bytes
  virtual gxf_result_t serialize_entity_abi(gxf_uid_t eid, Endpoint* endpoint, uint64_t* size) = 0;
  // Reads from endpoint and deserializes entity
  virtual gxf_result_t deserialize_entity_abi(gxf_uid_t eid, Endpoint* endpoint) = 0;

  // C++ API wrappers
  Expected<size_t> serializeEntity(Entity entity, Endpoint* endpoint);
  Expected<void> deserializeEntity(Entity entity, Endpoint* endpoint);

  Expected<Entity> deserializeEntity(gxf_context_t context, Endpoint* endpoint);
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_ENTITY_SERIALIZER_HPP_
