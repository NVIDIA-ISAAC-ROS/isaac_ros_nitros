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

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_VIEW_FACTORY_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_VIEW_FACTORY_HPP_

#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

namespace
{
constexpr uint64_t kNanosecondsInSeconds = 1e9;
}

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

/* *INDENT-OFF* */
// Factory macros that help define NITROS data type
// Mark the beginning of a factory for TYPE_NAME
#define MARK_PUBLIC_SECTION() \
public:
#define MARK_PROTECTED_SECTION() \
protected:
#define MARK_PRIVATE_SECTION() \
private:

#define ADD_COMMON_METHODS() \
MARK_PRIVATE_SECTION() \
int64_t  GetTimestamp() const { \
  auto timestamp = msg_entity_->get<gxf::Timestamp>("timestamp"); \
  if (!timestamp) { \
    timestamp = msg_entity_->get<gxf::Timestamp>(); \
  } \
  return timestamp.value()->acqtime; \
} \
MARK_PUBLIC_SECTION() \
int32_t GetTimestampSeconds() const {return GetTimestamp() / kNanosecondsInSeconds;} \
uint32_t GetTimestampNanoseconds() const {return GetTimestamp() % kNanosecondsInSeconds;} \
const std::string GetFrameId() const {return msg_.frame_id;}

#define NITROS_TYPE_VIEW_FACTORY_BEGIN(TYPE_NAME) \
class TYPE_NAME##View  \
{ \
MARK_PUBLIC_SECTION() \
void InitView(); \
explicit TYPE_NAME##View(const TYPE_NAME & in_msg) \
: msg_{in_msg}, \
  msg_entity_{gxf::Entity::Shared(GetTypeAdapterNitrosContext().getContext(), in_msg.handle)} { \
    InitView(); \
  } \
const TYPE_NAME##View & GetView() const {return *this;}  \
using BaseType = TYPE_NAME; \
ADD_COMMON_METHODS()

#define NITROS_TYPE_VIEW_FACTORY_END(TYPE_NAME) \
MARK_PRIVATE_SECTION() \
const TYPE_NAME & msg_; \
const gxf::Expected<gxf::Entity> msg_entity_; \
};
/* *INDENT-ON* */

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_VIEW_FACTORY_HPP_
