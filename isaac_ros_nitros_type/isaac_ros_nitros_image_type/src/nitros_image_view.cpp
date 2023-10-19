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

#include <string>
#include <vector>

#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

void NitrosImageView::InitView()
{
  auto gxf_video_buffer = msg_entity_->get<gxf::VideoBuffer>();
  if (!gxf_video_buffer) {
    std::stringstream error_msg;
    error_msg <<
      "[NitrosImageView::InitView] failed to get GXF video buffer" <<
      GxfResultStr(gxf_video_buffer.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImageView"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  image_ = &(*gxf_video_buffer.value());
}

const std::string NitrosImageView::GetEncoding() const
{
  const auto encoding = g_gxf_to_ros_video_format.find(image_->video_frame_info().color_format);
  if (encoding == std::end(g_gxf_to_ros_video_format)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosImageView"),
      "[NitrosImageView::GetEncoding] Unsupported encoding from gxf [%d].",
      (int)image_->video_frame_info().color_format);
    throw std::runtime_error("[NitrosImageView::GetEncoding] Unsupported encoding from gxf .");
  } else {
    return encoding->second;
  }
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
