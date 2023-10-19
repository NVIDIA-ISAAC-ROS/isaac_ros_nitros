// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "detection3_d_array_message/detection3_d_array_message.hpp"

namespace nvidia
{
namespace isaac
{

namespace
{

constexpr char kPoseName[] = "pose";
constexpr char kBBoxSizeName[] = "bbox_size";
constexpr char kObjectHypothesisName[] = "object_hypothesis";
constexpr char kTimestampName[] = "timestamp";

}  // namespace

gxf::Expected<Detection3DListMessageParts> CreateDetection3DListMessage(
  gxf_context_t context, size_t detections)
{
  gxf::Entity entity;
  return gxf::Entity::New(context)
         .assign_to(entity)
         .and_then(
    [&]() {
      gxf::Expected<void> result;
      for (size_t i = 0; i < detections; ++i) {
        result = result & entity.add<::nvidia::isaac::Pose3d>(kPoseName) &
        entity.add<::nvidia::isaac::Vector3f>(kBBoxSizeName) &
        entity.add<::nvidia::isaac::ObjectHypothesis>(kObjectHypothesisName);
      }
      result = result & entity.add<gxf::Timestamp>(kTimestampName);
      return result;
    })
         .and_then([&]() {return GetDetection3DListMessage(entity);});
}

gxf::Expected<Detection3DListMessageParts> GetDetection3DListMessage(gxf::Entity entity)
{
  Detection3DListMessageParts parts;
  parts.entity = entity;
  auto result =
    parts.entity.findAll<::nvidia::isaac::Pose3d>(parts.poses)
    .and_then([&]() {parts.entity.findAll<::nvidia::isaac::Vector3f>(parts.bbox_sizes);})
    .and_then([&]() {parts.entity.findAll<ObjectHypothesis>(parts.hypothesis);})
    .and_then([&]() {return parts.entity.get<gxf::Timestamp>(kTimestampName);})
    .log_error("Entity does not contain component Timestamp %s.", kTimestampName)
    .assign_to(parts.timestamp);
  if (!result) {
    return gxf::ForwardError(result);
  }
  if (parts.poses.size() != parts.bbox_sizes.size() ||
    parts.poses.size() != parts.hypothesis.size())
  {
    return gxf::Unexpected{GXF_FAILURE};
  }
  parts.count = parts.poses.size();
  return parts;
}

}  // namespace isaac
}  // namespace nvidia
