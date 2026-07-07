// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_VIEW_HPP_
#define ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_VIEW_HPP_

#include <string>

#include "isaac_ros_nitros/types/nitros_type_view_factory.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NITROS_TYPE_VIEW_FACTORY_BEGIN_NO_GXF(NitrosImage)

MARK_PUBLIC_SECTION()
// Public methods - backward compatible API using new class-based NitrosImage
inline uint32_t GetWidth() const {return msg_.width;}
inline uint32_t GetHeight() const {return msg_.height;}
inline uint32_t GetStride(const size_t plane_idx = 0) const
{
  if (plane_idx < msg_.num_planes()) {
    return msg_.get_plane(plane_idx).stride;
  }
  return msg_.step;
}
inline const std::string & GetEncoding() const {return msg_.encoding;}
inline size_t GetNumPlanes() const {return msg_.num_planes();}
inline const NitrosImage::ColorPlane & GetPlane(size_t i) const {return msg_.get_plane(i);}
inline size_t GetSizeInBytes() const
{
  // ROS image messages represent NV12/NV24 as compact buffers with no gap
  // between planes. Hardware-specific padding must be removed before
  // constructing the message.
  size_t total = static_cast<size_t>(msg_.step) * msg_.height;
  if (msg_.encoding == "nv12") {
    total = total * 3 / 2;
  } else if (msg_.encoding == "nv24") {
    total = total * 3;
  }
  return total;
}

inline const uint8_t * GetGpuData() const
{
  auto buffer = NitrosBufferAccessor<NitrosImage>::get_buffer(msg_);
  return buffer->get_data();
}

NITROS_TYPE_VIEW_FACTORY_END_NO_GXF(NitrosImage)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_VIEW_HPP_
