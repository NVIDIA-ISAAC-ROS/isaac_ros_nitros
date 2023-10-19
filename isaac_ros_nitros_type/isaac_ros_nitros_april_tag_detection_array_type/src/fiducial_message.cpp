// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "extensions/fiducials/messages/fiducial_message.hpp"

namespace nvidia
{
namespace isaac
{

namespace
{

// Name for the FiducialInfo in message
constexpr char kNameInfo[] = "info";
// Name for the Pose3 in message
constexpr char kNamePose[] = "pose";
// Name for the Tensor in message
constexpr char kNameKeypoints[] = "keypoints";

}  // namespace

gxf::Expected<FiducialMessageParts> CreateFiducialMessage(gxf_context_t context)
{
  FiducialMessageParts parts;
  return gxf::Entity::New(context)
         .assign_to(parts.entity)
         .and_then([&]() {return parts.entity.add<FiducialInfo>(kNameInfo);})
         .assign_to(parts.info)
         .and_then([&]() {return parts.entity.add<::nvidia::isaac::Pose3d>(kNamePose);})
         .assign_to(parts.pose)
         .and_then([&]() {return parts.entity.add<gxf::Tensor>(kNameKeypoints);})
         .assign_to(parts.keypoints)
         .substitute(parts);
}

gxf::Expected<FiducialMessageParts> GetFiducialMessage(gxf::Entity message)
{
  FiducialMessageParts parts;
  parts.entity = message;
  return parts.entity.get<FiducialInfo>(kNameInfo)
         .assign_to(parts.info)
         .and_then([&]() {return parts.entity.get<::nvidia::isaac::Pose3d>(kNamePose);})
         .assign_to(parts.pose)
         .and_then([&]() {return parts.entity.get<gxf::Tensor>(kNameKeypoints);})
         .assign_to(parts.keypoints)
         .substitute(parts);
}

gxf::Expected<FiducialListMessageParts> CreateFiducialListMessage(
  gxf_context_t context,
  size_t count)
{
  gxf::Entity entity;
  return gxf::Entity::New(context)
         .assign_to(entity)
         .and_then(
    [&]() {
      gxf::Expected<void> result;
      for (size_t i = 0; i < count; i++) {
        result = result &
        entity.add<FiducialInfo>(kNameInfo) &
        entity.add<::nvidia::isaac::Pose3d>(kNamePose) &
        entity.add<gxf::Tensor>(kNameKeypoints);
      }
      return result;
    })
         .and_then([&]() {return GetFiducialListMessage(entity);});
}

gxf::Expected<FiducialListMessageParts> GetFiducialListMessage(gxf::Entity message)
{
  FiducialListMessageParts parts;
  parts.entity = message;

  auto result = parts.entity.findAll<FiducialInfo>(parts.info)
    .and_then([&]() {parts.entity.findAll<::nvidia::isaac::Pose3d>(parts.pose);})
    .and_then([&]() {parts.entity.findAll<gxf::Tensor>(parts.keypoints);});
  if (!result) {
    return gxf::ForwardError(result);
  }
  if (parts.info.size() != parts.pose.size() || parts.info.size() != parts.keypoints.size()) {
    return gxf::Unexpected{GXF_FAILURE};
  }
  parts.count = parts.info.size();
  return parts;
}

}  // namespace isaac
}  // namespace nvidia
