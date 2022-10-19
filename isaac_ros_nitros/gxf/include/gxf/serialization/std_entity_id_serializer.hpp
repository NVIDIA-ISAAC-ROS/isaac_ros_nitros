/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
