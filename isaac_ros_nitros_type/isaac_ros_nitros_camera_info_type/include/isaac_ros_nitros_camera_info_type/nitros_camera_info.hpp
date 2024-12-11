// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS_CAMERA_INFO_TYPE__NITROS_CAMERA_INFO_HPP_
#define ISAAC_ROS_NITROS_CAMERA_INFO_TYPE__NITROS_CAMERA_INFO_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosCameraInfo
 *   ROS type:    sensor_msgs::msg::CameraInfo
 */

#include <string>

#include "isaac_ros_nitros/types/nitros_format_agent.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "gxf/core/gxf.h"
#include "gxf/core/entity.hpp"
#include "gxf/multimedia/camera.hpp"
#include "gxf/std/timestamp.hpp"

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type forward declaration
struct NitrosCameraInfo;

// Formats
struct nitros_camera_info_t
{
  using MsgT = NitrosCameraInfo;
  static const inline std::string supported_type_name = "nitros_camera_info";
};

// NITROS data type registration factory
NITROS_TYPE_FACTORY_BEGIN(NitrosCameraInfo)
// Supported data formats
NITROS_FORMAT_FACTORY_BEGIN()
NITROS_FORMAT_ADD(nitros_camera_info_t)
NITROS_FORMAT_FACTORY_END()
// Default compatible data format
NITROS_DEFAULT_COMPATIBLE_FORMAT(nitros_camera_info_t)
// Required extensions
NITROS_TYPE_EXTENSION_FACTORY_BEGIN()
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/multimedia/libgxf_multimedia.so")
NITROS_TYPE_EXTENSION_FACTORY_END()
NITROS_TYPE_FACTORY_END()

// Helper for convert_to_custom, resusable in other nodes
void copy_ros_to_gxf_camera_info(
  sensor_msgs::msg::CameraInfo source,
  nvidia::gxf::Expected<nvidia::gxf::Entity> & destination);

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia


template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosCameraInfo,
  sensor_msgs::msg::CameraInfo>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosCameraInfo;
  using ros_message_type = sensor_msgs::msg::CameraInfo;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosCameraInfo,
  sensor_msgs::msg::CameraInfo);

#endif  // ISAAC_ROS_NITROS_CAMERA_INFO_TYPE__NITROS_CAMERA_INFO_HPP_
