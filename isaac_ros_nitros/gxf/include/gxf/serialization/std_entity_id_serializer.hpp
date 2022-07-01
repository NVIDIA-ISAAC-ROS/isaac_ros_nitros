/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#ifndef NVIDIA_GXF_SERIALIZATION_STD_ENTITY_ID_SERIALIZER_HPP_
#define NVIDIA_GXF_SERIALIZATION_STD_ENTITY_ID_SERIALIZER_HPP_

#include <unordered_map>

#include "common/fixed_vector.hpp"
#include "gxf/serialization/entity_serializer.hpp"

namespace nvidia {
namespace gxf {

// Serializes and deserializes entity id. No component serializer used.

class StdEntityIdSerializer : EntitySerializer {
 public:
#pragma pack(push, 1)
  struct EntityHeader {
    uint64_t entity_id;
    uint64_t sequence_number;
  };
#pragma pack(pop)

  gxf_result_t initialize() override { return GXF_SUCCESS; }
  gxf_result_t deinitialize() override { return GXF_SUCCESS; }

  Expected<Entity> deserialize_entity_header_abi(Endpoint* endpoint) override;

  gxf_result_t serialize_entity_abi(gxf_uid_t eid, Endpoint* endpoint,
                                    uint64_t* size) override;
  gxf_result_t deserialize_entity_abi(gxf_uid_t eid,
                                      Endpoint* endpoint) override;

 private:
  // Sequence number for outgoing messages
  uint64_t outgoing_sequence_number_;
  // Sequence number for incoming messages
  uint64_t incoming_sequence_number_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_SERIALIZATION_STD_ENTITY_ID_SERIALIZER_HPP_
