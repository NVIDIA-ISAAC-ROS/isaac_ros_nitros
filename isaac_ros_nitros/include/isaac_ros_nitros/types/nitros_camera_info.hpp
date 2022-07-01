/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_CAMERA_INFO_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_CAMERA_INFO_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosCameraInfo
 *   ROS type:    sensor_msgs::msg::CameraInfo
 */

#include <string>

#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type
struct NitrosCameraInfo : NitrosTypeBase
{
  using NitrosTypeBase::NitrosTypeBase;
};

// Formats
struct nitros_camera_info_t
{
  using MsgT = NitrosCameraInfo;
  static const inline std::string supported_type_name = "nitros_camera_info";
};

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

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_CAMERA_INFO_HPP_
