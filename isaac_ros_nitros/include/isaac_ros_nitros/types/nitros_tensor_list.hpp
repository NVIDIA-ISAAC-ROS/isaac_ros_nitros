/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_TENSOR_LIST_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_TENSOR_LIST_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosTensorList
 *   ROS type:    isaac_ros_tensor_list_interfaces::msg::TensorList
 */

#include <string>

#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"

#include "rclcpp/type_adapter.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type
struct NitrosTensorList : NitrosTypeBase
{
  using NitrosTypeBase::NitrosTypeBase;
};

// Formats
struct nitros_tensor_list_nchw_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nchw";
};

struct nitros_tensor_list_nhwc_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nhwc";
};

struct nitros_tensor_list_nchw_rgb_f32_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nchw_rgb_f32";
};

struct nitros_tensor_list_nhwc_rgb_f32_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nhwc_rgb_f32";
};

struct nitros_tensor_list_nchw_bgr_f32_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nchw_bgr_f32";
};

struct nitros_tensor_list_nhwc_bgr_f32_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nhwc_bgr_f32";
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia


template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosTensorList,
  isaac_ros_tensor_list_interfaces::msg::TensorList>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosTensorList;
  using ros_message_type = isaac_ros_tensor_list_interfaces::msg::TensorList;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosTensorList,
  isaac_ros_tensor_list_interfaces::msg::TensorList);

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_TENSOR_LIST_HPP_
