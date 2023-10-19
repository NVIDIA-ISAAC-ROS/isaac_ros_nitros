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

#ifndef ISAAC_ROS_NITROS__TYPES__TYPE_UTILITY_HPP_
#define ISAAC_ROS_NITROS__TYPES__TYPE_UTILITY_HPP_

#include <string>
#if defined(USE_NVTX)
  #include "nvToolsExt.h"
#endif

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

constexpr u_int CLR_RED = 0xFFFF0000;
constexpr u_int CLR_BLUE = 0xFF0000FF;
constexpr u_int CLR_GREEN = 0xFF008000;
constexpr u_int CLR_YELLOW = 0xFFFFFF00;
constexpr u_int CLR_CYAN = 0xFF00FFFF;
constexpr u_int CLR_MAGENTA = 0xFFFF00FF;
constexpr u_int CLR_GRAY = 0xFF808080;
constexpr u_int CLR_PURPLE = 0xFF800080;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static inline void nvtxRangePushWrapper(const char * range_title, u_int range_color)
{
  #if defined(USE_NVTX)
  nvtxEventAttributes_t eventAttrib = {
    NVTX_VERSION,  // version
    NVTX_EVENT_ATTRIB_STRUCT_SIZE,  // size
    0,  // category
    0,  // colorType
    0,  // color
    0,  // payloadType
    0,  // reseverd0
    0,  // payload
    0,  // messageType
    0,  // message
  };
  eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
  eventAttrib.colorType = NVTX_COLOR_ARGB;
  eventAttrib.color = range_color;
  eventAttrib.message.ascii = range_title;
  nvtxRangePushEx(&eventAttrib);
  #endif
}
#pragma GCC diagnostic pop

static inline void nvtxRangePopWrapper()
{
  #if defined(USE_NVTX)
  nvtxRangePop();
  #endif
}

static inline void nvtxMarkExWrapper(const char * range_title, u_int range_color)
{
  #if defined(USE_NVTX)
  nvtxEventAttributes_t eventAttrib = {
    NVTX_VERSION,  // version
    NVTX_EVENT_ATTRIB_STRUCT_SIZE,  // size
    0,  // category
    0,  // colorType
    0,  // color
    0,  // payloadType
    0,  // reseverd0
    0,  // payload
    0,  // messageType
    0,  // message
  };
  eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
  eventAttrib.colorType = NVTX_COLOR_ARGB;
  eventAttrib.color = range_color;
  eventAttrib.message.ascii = range_title;
  nvtxMarkEx(&eventAttrib);
  #endif
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__TYPE_UTILITY_HPP_
