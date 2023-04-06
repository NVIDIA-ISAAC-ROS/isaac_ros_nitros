/*
Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
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

gxf::Expected<FiducialMessageParts> CreateFiducialMessage(gxf_context_t context, bool activate)
{
  FiducialMessageParts parts;
  return gxf::Entity::New(context)
         .assign_to(parts.entity)
         .and_then([&]() {return parts.entity.add<FiducialInfo>(kNameInfo);})
         .assign_to(parts.info)
         .and_then([&]() {return parts.entity.add<::isaac::Pose3d>(kNamePose);})
         .assign_to(parts.pose)
         .and_then([&]() {return parts.entity.add<gxf::Tensor>(kNameKeypoints);})
         .assign_to(parts.keypoints)
         .and_then([&]() {return activate ? parts.entity.activate() : gxf::Success;})
         .substitute(parts);
}

gxf::Expected<FiducialMessageParts> GetFiducialMessage(gxf::Entity message)
{
  FiducialMessageParts parts;
  parts.entity = message;
  return parts.entity.get<FiducialInfo>(kNameInfo)
         .assign_to(parts.info)
         .and_then([&]() {return parts.entity.get<::isaac::Pose3d>(kNamePose);})
         .assign_to(parts.pose)
         .and_then([&]() {return parts.entity.get<gxf::Tensor>(kNameKeypoints);})
         .assign_to(parts.keypoints)
         .substitute(parts);
}

gxf::Expected<FiducialListMessageParts> CreateFiducialListMessage(
  gxf_context_t context,
  size_t count,
  bool activate)
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
        entity.add<::isaac::Pose3d>(kNamePose) &
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
    .and_then([&]() {parts.entity.findAll<::isaac::Pose3d>(parts.pose);})
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
