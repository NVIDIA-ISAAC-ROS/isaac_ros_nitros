/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_IMAGE_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_IMAGE_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosImage
 *   ROS type:    sensor_msgs::msg::Image
 */

#include <string>

#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/image.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type
struct NitrosImage : NitrosTypeBase
{
  using NitrosTypeBase::NitrosTypeBase;
};

// Formats
struct nitros_image_rgb8_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_rgb8";
};

struct nitros_image_rgba8_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_rgba8";
};

struct nitros_image_rgb16_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_rgb16";
};

struct nitros_image_bgr8_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_bgr8";
};

struct nitros_image_bgra8_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_bgra8";
};

struct nitros_image_bgr16_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_bgr16";
};

struct nitros_image_mono8_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_mono8";
};

struct nitros_image_mono16_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_mono16";
};

struct nitros_image_nv24_t
{
  using MsgT = NitrosImage;
  static const inline std::string supported_type_name = "nitros_image_nv24";
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia


template<>
struct rclcpp::TypeAdapter<nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosImage;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosImage,
  sensor_msgs::msg::Image);

uint64_t calculate_image_size(const std::string image_type, uint32_t width, uint32_t height);
#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_IMAGE_HPP_
