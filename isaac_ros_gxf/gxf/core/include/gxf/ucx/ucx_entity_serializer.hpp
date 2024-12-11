// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_GXF_SERIALIZATION_UCX_ENTITY_SERIALIZER_HPP_
#define NVIDIA_GXF_SERIALIZATION_UCX_ENTITY_SERIALIZER_HPP_

#define kMaxTempComponents 1024

#include <unordered_map>

#include "common/fixed_vector.hpp"
#include "gxf/core/gxf.h"
#include "gxf/serialization/component_serializer.hpp"
#include "gxf/serialization/entity_serializer.hpp"
#include "gxf/serialization/tid_hash.hpp"

namespace nvidia {
namespace gxf {

// Serializes and deserializes entities with the provided component serializers
// Little-endian is used over big-endian for better performance on x86 and arm platforms
// Entities are serialized in the following format:
//
//   | Entity Header | Component Header | Component Metadata | Component Header | ... | ... |
//
// Components will be serialized in the order they are added to the entity
// Components without serializers will be skipped
// Each component will be preceded by a component header and the name of the component
// The component itself will be serialized with a component serializer
// An entity header will be added at the beginning
class UcxEntitySerializer : public EntitySerializer {
 public:
  #pragma pack(push, 1)
  // Header preceding entities
  struct EntityHeader {
    uint64_t serialized_size;  // Size of the serialized entity in bytes
    uint64_t sequence_number;  // Sequence number of the message
    uint64_t component_count;  // Number of components in the entity
  };
  #pragma pack(pop)

  #pragma pack(push, 1)
  // Header preceding components
  struct ComponentHeader {
    uint64_t serialized_size;  // Size of the serialized component in bytes
    gxf_tid_t tid;             // Type ID of the component
    char name[kMaxComponentNameSize];            // Component name
  };
  #pragma pack(pop)

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override { return GXF_SUCCESS; }
  gxf_result_t deinitialize() override { return GXF_SUCCESS; }

  gxf_result_t serialize_entity_abi(gxf_uid_t eid, Endpoint* endpoint, uint64_t* size) override;
  gxf_result_t deserialize_entity_abi(gxf_uid_t eid, Endpoint* endpoint) override;
  Expected<Entity> deserialize_entity_header_abi(Endpoint* endpoint) override;

  Expected<void> add_serializer(const Handle<ComponentSerializer>& serializer);

 private:
  // Structure used to organize serializable components
  struct ComponentEntry;

  using ComponentSerializerList = FixedVector<Handle<ComponentSerializer>, kMaxTempComponents>;

  // Populates a list of component entries using a list of component handles
  Expected<FixedVector<ComponentEntry, kMaxTempComponents>> createComponentEntries(
      const FixedVector<UntypedHandle, kMaxComponents>& components);
  // Serializes a list of components and writes them to an endpoint
  // Returns the total number of bytes serialized
  Expected<size_t> serializeComponents(const FixedVector<ComponentEntry,
                                       kMaxTempComponents>& entries,
                                       Endpoint* endpoint);
  // Reads from an endpoint and deserializes a list of components
  Expected<void> deserializeComponents(size_t component_count, Entity entity, Endpoint* endpoint);
  // Searches for a component serializer that supports the given type ID
  // Uses the first valid serializer found and caches it for subsequent lookups
  // Returns an Unexpected if no valid serializer is found
  Expected<Handle<ComponentSerializer>> findComponentSerializer(gxf_tid_t tid);

  Parameter<ComponentSerializerList> component_serializers_;
  Parameter<bool> verbose_warning_;

  // Table that caches type ID with a valid component serializer
  std::unordered_map<gxf_tid_t, Handle<ComponentSerializer>, TidHash> serializer_cache_;
  // Sequence number for outgoing messages
  uint64_t outgoing_sequence_number_{0};
  // Sequence number for incoming messages
  uint64_t incoming_sequence_number_{0};
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_STD_ENTITY_SERIALIZER_HPP_
