/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "detection2_d_array_message.hpp"

#include <utility>

namespace nvidia
{
namespace isaac_ros
{

namespace
{
constexpr char const * kDetection2DArrayIdentifier = "detection2_d_array";
constexpr char const * kTimestampIdentifier = "timestamp";
}

gxf::Expected<Detection2DParts> CreateDetection2DList(gxf_context_t context)
{
  Detection2DParts parts;
  return gxf::Entity::New(context)
         .assign_to(parts.message)
         .and_then(
    [&]() {
      return parts.message.add<std::vector<nvidia::isaac_ros::Detection2D>>(
        kDetection2DArrayIdentifier);
    })
         .assign_to(parts.detection2_d_array)
         .and_then([&]() {return parts.message.add<gxf::Timestamp>(kTimestampIdentifier);})
         .assign_to(parts.timestamp)
         .substitute(parts);
}

gxf::Expected<Detection2DParts> GetDetection2DList(gxf::Entity message)
{
  Detection2DParts parts;
  parts.message = message;
  return parts.message.get<std::vector<nvidia::isaac_ros::Detection2D>>(kDetection2DArrayIdentifier)
         .log_error(
    "Entity does not contain component ExampleData %s.",
    kDetection2DArrayIdentifier)
         .assign_to(parts.detection2_d_array)
         .and_then([&]() {return parts.message.get<gxf::Timestamp>(kTimestampIdentifier);})
         .log_error("Entity does not contain component Timestamp %s.", kTimestampIdentifier)
         .assign_to(parts.timestamp)
         .substitute(parts);
}

}  // namespace isaac_ros
}  // namespace nvidia
