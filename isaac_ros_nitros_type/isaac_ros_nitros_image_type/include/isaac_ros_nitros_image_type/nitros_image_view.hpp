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

#ifndef ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_VIEW_HPP_
#define ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_VIEW_HPP_

#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/multimedia/video.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/types/nitros_type_view_factory.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NITROS_TYPE_VIEW_FACTORY_BEGIN(NitrosImage)

MARK_PUBLIC_SECTION()
// Public methods
inline uint64_t GetSizeInBytes() const {return image_->size();}
inline uint32_t GetWidth() const {return image_->video_frame_info().width;}
inline uint32_t GetHeight() const {return image_->video_frame_info().height;}
inline uint32_t GetStride(const size_t plane_idx = 0) const
{
  return image_->video_frame_info().color_planes[plane_idx].stride;
}
inline const unsigned char * GetGpuData() const {return image_->pointer();}
const std::string GetEncoding() const;

MARK_PUBLIC_SECTION()
gxf::VideoBuffer * image_{nullptr};

NITROS_TYPE_VIEW_FACTORY_END(NitrosImage)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_VIEW_HPP_
