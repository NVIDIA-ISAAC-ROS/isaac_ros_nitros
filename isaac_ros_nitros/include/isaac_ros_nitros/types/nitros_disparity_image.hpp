/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_DISPARITY_IMAGE_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_DISPARITY_IMAGE_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosDisparityImage
 *   ROS type:    stereo_msgs::msg::DisparityImage
 */

#include <string>

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type
struct NitrosDisparityImage : NitrosTypeBase
{
  using NitrosTypeBase::NitrosTypeBase;
};

// Formats
struct nitros_disparity_image_bgr8_t
{
  using MsgT = NitrosDisparityImage;
  static const inline std::string supported_type_name = "nitros_disparity_image_bgr8";
};

struct nitros_disparity_image_32FC1_t
{
  using MsgT = NitrosDisparityImage;
  static const inline std::string supported_type_name = "nitros_disparity_image_32FC1";
};

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

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_DISPARITY_IMAGE_HPP_
