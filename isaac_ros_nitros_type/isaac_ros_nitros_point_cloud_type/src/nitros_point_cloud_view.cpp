// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud_view.hpp"

#include <sstream>

#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"
#include "messages/point_cloud_message.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

void NitrosPointCloudView::InitView()
{
  GetPointCloudData();
}

void NitrosPointCloudView::GetPointCloudData()
{
  auto maybe_point_cloud_parts = nvidia::isaac_ros::messages::GetPointCloudMessage(
    msg_entity_.value());
  if (!maybe_point_cloud_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[GetPointCloudData] Failed to get pointcloud message from message entity: " <<
      GxfResultStr(maybe_point_cloud_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloudView"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto point_cloud_parts = maybe_point_cloud_parts.value();

  // Extract point cloud data
  height_ = 1;
  width_ = static_cast<uint32_t>(point_cloud_parts.points->shape().dimension(0));
  point_count_ = width_ * height_;
  has_color_ = point_cloud_parts.info->use_color;
  is_bigendian_ = point_cloud_parts.info->is_bigendian;
  is_dense_ = false;

  // Calculate strides
  point_step_ = has_color_ ? 16 : 12;  // 4 or 3 floats * 4 bytes
  row_step_ = point_step_ * width_;

  // Get pointer to data
  auto maybe_points_data = point_cloud_parts.points->data<float>();
  if (!maybe_points_data) {
    std::stringstream error_msg;
    error_msg << "[GetPointCloudData] Failed to get float data from points tensor";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloudView"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  points_data_ = maybe_points_data.value();

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloudView"),
    "[GetPointCloudData] Point cloud data extracted: width=%d, height=%d, "
    "point_count=%d, has_color=%d, point_step=%d, row_step=%d",
    width_, height_, point_count_, has_color_, point_step_, row_step_);
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
