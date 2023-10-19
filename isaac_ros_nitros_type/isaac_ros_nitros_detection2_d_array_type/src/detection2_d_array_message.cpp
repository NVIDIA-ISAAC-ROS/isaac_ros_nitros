// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
