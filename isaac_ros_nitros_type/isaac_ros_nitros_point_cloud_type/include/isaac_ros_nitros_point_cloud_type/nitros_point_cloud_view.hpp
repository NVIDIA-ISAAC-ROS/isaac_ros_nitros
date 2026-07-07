// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_VIEW_HPP_
#define ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_VIEW_HPP_

#include <string>

#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros/types/nitros_type_view_factory.hpp"
#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NITROS_TYPE_VIEW_FACTORY_BEGIN_NO_GXF(NitrosPointCloud)

MARK_PUBLIC_SECTION()
inline uint32_t GetWidth() const {return msg_.width;}
inline uint32_t GetHeight() const {return msg_.height;}
inline uint32_t GetPointCount() const {return msg_.width * msg_.height;}
inline bool HasColor() const {return msg_.use_color;}
inline uint32_t GetPointStep() const {return msg_.point_step;}
inline uint32_t GetRowStep() const {return msg_.row_step;}
inline bool IsBigEndian() const {return msg_.is_bigendian;}
inline bool IsDense() const {return false;}

inline const float * GetPointsData() const
{
  auto buffer = NitrosBufferAccessor<NitrosPointCloud>::get_buffer(msg_);
  if (!buffer) {
    return nullptr;
  }
  return reinterpret_cast<const float *>(buffer->get_data());
}

NITROS_TYPE_VIEW_FACTORY_END_NO_GXF(NitrosPointCloud)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_POINT_CLOUD_TYPE__NITROS_POINT_CLOUD_VIEW_HPP_
