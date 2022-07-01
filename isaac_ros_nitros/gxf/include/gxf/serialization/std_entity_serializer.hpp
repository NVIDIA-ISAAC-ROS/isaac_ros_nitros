/*
Copyright (c) 2021-2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_SERIALIZATION_STD_ENTITY_SERIALIZER_HPP_
#define NVIDIA_GXF_SERIALIZATION_STD_ENTITY_SERIALIZER_HPP_

#include <unordered_map>

#include "common/fixed_vector.hpp"
#include "gxf/serialization/component_serializer.hpp"
#include "gxf/serialization/entity_serializer.hpp"
#include "gxf/serialization/tid_hash.hpp"

namespace nvidia {
namespace gxf {

// Serializes and deserializes entities with the provided component serializers
// Little-endian is used over big-endian for better performance on x86 and arm platforms
// Entities are serialized in the following format:
//
//   | Entity Header || Component Header | Component Name | Component | ... | ... | ... |
//
// Components will be serialized in the order they are added to the entity
// Components without serializers will be skipped
// Each component will be preceeded by a component header and the name of the component
// The component itself will be serialized with a component serializer
// An entity header will be added at the beginning
class StdEntitySerializer : EntitySerializer {
 public:
  #pragma pack(push, 1)
  // Header preceding entities
  struct EntityHeader {
    uint64_t serialized_size;  // Size of the serialized entity in bytes
    uint32_t checksum;         // Checksum to verify the integrity of the message
    uint64_t sequence_number;  // Sequence number of the message
    uint32_t flags;            // Flags to specify delivery options
    uint64_t component_count;  // Number of components in the entity
    uint64_t reserved;         // Bytes reserved for future use
  };
  #pragma pack(pop)

  #pragma pack(push, 1)
  // Header preceding components
  struct ComponentHeader {
    uint64_t serialized_size;  // Size of the serialized component in bytes
    gxf_tid_t tid;             // Type ID of the component
    uint64_t name_size;        // Size of the component name in bytes
  };
  #pragma pack(pop)

  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override { return GXF_SUCCESS; }
  gxf_result_t deinitialize() override { return GXF_SUCCESS; }

  gxf_result_t serialize_entity_abi(gxf_uid_t eid, Endpoint* endpoint, uint64_t* size) override;
  gxf_result_t deserialize_entity_abi(gxf_uid_t eid, Endpoint* endpoint) override;
  Expected<Entity> deserialize_entity_header_abi(Endpoint* endpoint) override;

 private:
  // Structure used to organize serializable components
  struct ComponentEntry;

  // Populates a list of component entries using a list of component handles
  Expected<FixedVector<ComponentEntry, kMaxComponents>> createComponentEntries(
      const FixedVectorBase<UntypedHandle>& components);
  // Serializes a list of components and writes them to an endpoint
  // Returns the total number of bytes serialized
  Expected<size_t> serializeComponents(const FixedVectorBase<ComponentEntry>& entries,
                                       Endpoint* endpoint);
  // Reads from an endpoint and deserializes a list of components
  Expected<void> deserializeComponents(size_t component_count, Entity entity, Endpoint* endpoint);
  // Searches for a component serializer that supports the given type ID
  // Uses the first valid serializer found and caches it for subsequent lookups
  // Returns an Unexpected if no valid serializer is found
  Expected<Handle<ComponentSerializer>> findComponentSerializer(gxf_tid_t tid);

  Parameter<FixedVector<Handle<ComponentSerializer>, kMaxComponents>> component_serializers_;

  // Table that caches type ID with a valid component serializer
  std::unordered_map<gxf_tid_t, Handle<ComponentSerializer>, TidHash> serializer_cache_;
  // Sequence number for outgoing messages
  uint64_t outgoing_sequence_number_;
  // Sequence number for incoming messages
  uint64_t incoming_sequence_number_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_STD_ENTITY_SERIALIZER_HPP_
