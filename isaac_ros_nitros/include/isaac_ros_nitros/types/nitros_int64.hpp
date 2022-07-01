/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_INT64_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_INT64_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosInt64
 *   ROS type:    std_msgs::msg::Int64
 */

#include <string>

#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "rclcpp/type_adapter.hpp"
#include "std_msgs/msg/int64.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type
struct NitrosInt64 : NitrosTypeBase
{
  using NitrosTypeBase::NitrosTypeBase;
};

// Formats
struct nitros_int64_t
{
  using MsgT = NitrosInt64;
  static const inline std::string supported_type_name = "nitros_int64";
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia


template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosInt64,
  std_msgs::msg::Int64>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosInt64;
  using ros_message_type = std_msgs::msg::Int64;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosInt64,
  std_msgs::msg::Int64);

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_INT64_HPP_
