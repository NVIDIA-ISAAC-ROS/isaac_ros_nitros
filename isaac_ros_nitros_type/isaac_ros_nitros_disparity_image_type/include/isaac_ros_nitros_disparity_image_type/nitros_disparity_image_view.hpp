// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_VIEW_HPP_
#define ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_VIEW_HPP_

#include <string>

#include "isaac_ros_nitros/types/nitros_type_view_factory.hpp"
#include "isaac_ros_nitros_disparity_image_type/nitros_disparity_image.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NITROS_TYPE_VIEW_FACTORY_BEGIN_NO_GXF(NitrosDisparityImage)

MARK_PUBLIC_SECTION()
// Public methods - backward compatible API using new class-based NitrosImage
inline uint32_t GetWidth() const {return msg_.width;}
inline uint32_t GetHeight() const {return msg_.height;}

inline uint32_t GetStep() const {return msg_.step;}

inline size_t GetSizeInBytes() const
{
  return static_cast<size_t>(msg_.step) * msg_.height;
}

inline const uint8_t * GetGpuData() const
{
  auto buffer = NitrosBufferAccessor<NitrosDisparityImage>::get_buffer(msg_);
  return buffer->get_data();
}
NITROS_TYPE_VIEW_FACTORY_END_NO_GXF(NitrosDisparityImage)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_VIEW_HPP_
