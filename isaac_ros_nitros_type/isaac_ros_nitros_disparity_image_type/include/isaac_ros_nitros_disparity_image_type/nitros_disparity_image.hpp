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

#ifndef ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_HPP_
#define ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosDisparityImage
 *   ROS type:    stereo_msgs::msg::DisparityImage
 */

#include <string>

#include "isaac_ros_nitros/types/nitros_format_agent.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "rclcpp/type_adapter.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type forward declaration
struct NitrosDisparityImage;

// Formats
struct nitros_disparity_image_32FC1_t
{
  using MsgT = NitrosDisparityImage;
  static const inline std::string supported_type_name = "nitros_disparity_image_32FC1";
};

// NITROS data type registration factory
NITROS_TYPE_FACTORY_BEGIN(NitrosDisparityImage)
// Supported data formats
NITROS_FORMAT_FACTORY_BEGIN()
NITROS_FORMAT_ADD(nitros_disparity_image_32FC1_t)
NITROS_FORMAT_FACTORY_END()
// Required extensions
NITROS_TYPE_EXTENSION_FACTORY_BEGIN()
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/std/libgxf_std.so")
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/multimedia/libgxf_multimedia.so")
// for nvidia::gxf::ComponentSerializer
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so")
// for nvidia::isaac::StringProvider
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/libgxf_gxf_helpers.so")
// for nvidia::isaac::SightReceiver
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/libgxf_sight.so")
// for nvidia::isaac::PoseTree
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/libgxf_atlas.so")
// for nvidia::isaac::PoseFrameUid
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/libgxf_isaac_messages.so")
NITROS_TYPE_EXTENSION_FACTORY_END()
NITROS_TYPE_FACTORY_END()

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia


template<>
struct rclcpp::TypeAdapter<nvidia::isaac_ros::nitros::NitrosDisparityImage,
  stereo_msgs::msg::DisparityImage>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosDisparityImage;
  using ros_message_type = stereo_msgs::msg::DisparityImage;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosDisparityImage,
  stereo_msgs::msg::DisparityImage);

#endif  // ISAAC_ROS_NITROS_DISPARITY_IMAGE_TYPE__NITROS_DISPARITY_IMAGE_HPP_
