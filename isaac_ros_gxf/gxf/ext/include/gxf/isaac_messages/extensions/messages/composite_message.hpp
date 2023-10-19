// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2020-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#pragma once

#include <utility>

#include "engine/core/tensor/tensor.hpp"
#include "extensions/atlas/composite_schema_server.hpp"
#include "gems/common/composite_schema_uid.hpp"
#include "gems/common/pose_frame_uid.hpp"
#include "gems/composite/composite_from_tensor.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac {

// Stores the message and provides convenience views to it
struct CompositeMessageParts {
  // The message entity
  gxf::Entity message;
  // View to the data
  ::nvidia::isaac::CpuTensorView2d view;
  // The uid of the pose frame
  gxf::Handle<PoseFrameUid> pose_frame_uid;
  // The uid of the composite schema
  gxf::Handle<CompositeSchemaUid> composite_schema_uid;
  // Timestamp
  gxf::Handle<gxf::Timestamp> timestamp;
};

// Makes a copy of a composite message. The copy is added to the entity with entity-id
// message_eid.
gxf::Expected<CompositeMessageParts> MakeCopyCompositeMessage(
    gxf_context_t context, gxf_uid_t allocator_cid, gxf_uid_t message_eid,
    const CompositeMessageParts& input_parts);

// Creates a new entity and adds the components contained in a CompositeMessageParts to it and sets
// up convenience views/handles to it. Allocates memory for the contained tensor to be of shape
// num_states x state_size.
gxf::Expected<CompositeMessageParts> CreateCompositeMessage(
    gxf_context_t context, gxf::Handle<gxf::Allocator> allocator, int num_states, int state_size);

// Adds the components contained in a CompositeMessageParts to an existing entity and sets up
// convenience views/handles to it. Allocates memory for the contained tensor to be of shape
// num_states x state_size.
gxf::Expected<CompositeMessageParts> AddCompositeMessage(
    gxf::Entity entity, gxf::Handle<gxf::Allocator> allocator, int num_states, int state_size);

// Extracts a CompositeMessageParts from an entity and sets up convenience views/handles.
gxf::Expected<CompositeMessageParts> GetCompositeMessage(gxf::Entity message);

// Deprecated: Use AddCompositeMessage() instead.
// Same as AddCompositeMessage() but using different signature. To be consistent with other message
// parts structs use AddCompositeMessage() if possible.
gxf::Expected<CompositeMessageParts> PrepareCompositeMessage(
        gxf_context_t context, gxf_uid_t allocator_cid, gxf_uid_t message_eid, int num_states,
        int state_size);

// Deprecated: Use GetCompositeMessage() instead.
// Same as GetCompositeMessage(). To be consistent with other message parts structs use
// GetCompositeMessage() if possible.
gxf::Expected<CompositeMessageParts> ParseCompositeMessage(gxf::Entity message);

// Extract a CompositeArray or CompositeEigen from a composite message using the schema server.
// Parameter `slice` is used to select the slice of the tensor which is used to populate the
// composite.
template <typename Composite>
gxf::Expected<Composite> ExtractComposite(
    const CompositeMessageParts& message_parts, const CompositeSchemaServer& schema_server,
    const composite::Schema& desired_schema, int slice = 0) {
  // Retrieve the schema of the composite message from the schema server.
  const auto maybe_schema = schema_server.get(message_parts.composite_schema_uid->uid);
  if (!maybe_schema) {
    GXF_LOG_ERROR("Failed to get schema from schema server.");
    return gxf::Unexpected(maybe_schema.error());
  }
  const composite::Schema& current_schema = maybe_schema.value();

  return CompositeFromTensor<Composite>(message_parts.view.const_slice(slice),
      current_schema, desired_schema);
}

// Convenience wrapper around function above. Allows to directly pass a gxf::Entity instead of a
// CompositeMessageParts.
template <typename Composite>
gxf::Expected<Composite> ExtractComposite(
    gxf::Entity entity, const CompositeSchemaServer& schema_server,
    const composite::Schema& desired_schema, int slice = 0) {
  // Parse the entity to a composite message parts.
  const auto maybe_message_parts = GetCompositeMessage(entity);
  if (!maybe_message_parts) {
    GXF_LOG_ERROR("Failed to parse entity as CompositeMessageParts.");
    return gxf::Unexpected(maybe_message_parts.error());
  }
  const CompositeMessageParts& message_parts = maybe_message_parts.value();
  return ExtractComposite<Composite>(message_parts, schema_server, desired_schema, slice);
}

// Convenience wrapper around function above. Allows to directly pass the handle to a
// CompositeSchemaServer and checks that that the handle is not null.
template <typename Composite>
gxf::Expected<Composite> ExtractComposite(
    gxf::Entity entity, gxf::Handle<CompositeSchemaServer> schema_server_handle,
    const composite::Schema& desired_schema, int slice = 0) {
  if (!schema_server_handle) {
    GXF_LOG_ERROR("No schema server provided.");
    return gxf::Unexpected{GXF_FAILURE};
  }
  const CompositeSchemaServer& schema_server = *schema_server_handle;

  return ExtractComposite<Composite>(
      std::move(entity), schema_server, desired_schema, slice);
}

}  // namespace isaac
}  // namespace nvidia
