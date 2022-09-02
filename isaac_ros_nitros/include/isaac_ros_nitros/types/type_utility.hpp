/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__TYPES__TYPE_UTILITY_HPP_
