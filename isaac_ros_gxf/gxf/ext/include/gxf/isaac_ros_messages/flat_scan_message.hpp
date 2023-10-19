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
#ifndef NVIDIA_ISAAC_EXTENSIONS_MESSAGES_FLATSCAN_TENSOR_MESSAGE_HPP_
#define NVIDIA_ISAAC_EXTENSIONS_MESSAGES_FLATSCAN_TENSOR_MESSAGE_HPP_

#include "engine/core/tensor/tensor.hpp"
#include "gems/common/pose_frame_uid.hpp"
#include "gems/flatscan/flatscan_info.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia {
namespace isaac_ros {
namespace messages {

// Stores the flatscan message and provides convenience views to it.
struct FlatscanMessageParts {
  // The message entity
  gxf::Entity entity;
  // View to the beams data
  // This is a 2d tensor, where each row is a beam which defined by the schema in
  // sdk/gems/flatscan/flatscan_types.hpp
  gxf::Handle<gxf::Tensor> beams;
  // View to the nvidia::isaac::nvidia::isaac::FlatscanInfo instance holding meta information about this message
  gxf::Handle<nvidia::isaac::FlatscanInfo> info;
  // The uid of this scan's origin pose frame
  gxf::Handle<nvidia::isaac::PoseFrameUid> pose_frame_uid;
  // Timestamp
  gxf::Handle<gxf::Timestamp> timestamp;
};

// Allocates a message entity representing a flatscan message containing `beam_count` beams. The
// returned message is a struct of type `FlatscanMessageParts` consisting of an entity holding all
// data, two TensorView objects allowing easy access to individual message parts, and a reference to
// the FlatscanInfo object containing the message meta data.
// The entity has the following fields:
//  - "beams": Of type `gxf::Tensor` the definition please refer:
//  sdk/gems/flatscan/flatscan_types.hpp
//  - "info": Of type `isaac::FlatscanInfo`, contains the meta information describing the message's
//            content.
//  - "pose_frame_uid": Of type `isaac::PoseFrameUid`, contains the uid of the pose frame the
//                      flatscan originates from.
// If `activate` is `true`, the message will be activated after creation.
// The TensorView objects returned represent the "angles" and "ranges" tensors, respectively.
gxf::Expected<FlatscanMessageParts> CreateFlatscanMessage(gxf_context_t context,
                                                          gxf::Handle<gxf::Allocator> allocator,
                                                          int beam_count, bool activate = true);

// Returns a struct of type `FlatscanMessageParts` consisting of an entity holding all flatscan
// data: two TensorView objects allowing easy access to individual message parts, and a reference to
// the FlatscanInfo object containing the message meta data.
gxf::Expected<FlatscanMessageParts> GetFlatscanMessage(gxf::Entity message);

}  // namespace messages
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NVIDIA_ISAAC_EXTENSIONS_MESSAGES_FLATSCAN_TENSOR_MESSAGE_HPP_
