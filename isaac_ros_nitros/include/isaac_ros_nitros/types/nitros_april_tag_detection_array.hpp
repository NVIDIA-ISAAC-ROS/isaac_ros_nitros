/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_APRIL_TAG_DETECTION_ARRAY_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_APRIL_TAG_DETECTION_ARRAY_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosAprilTagDetectionArray
 *   ROS type:    isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray
 */

#include <string>

#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/type_adapter.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type
struct NitrosAprilTagDetectionArray : NitrosTypeBase
{
  using NitrosTypeBase::NitrosTypeBase;
};

// Formats
struct nitros_april_tag_detection_array_t
{
  using MsgT = NitrosAprilTagDetectionArray;
  static const inline std::string supported_type_name = "nitros_april_tag_detection_array";
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia


template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosAprilTagDetectionArray,
  isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosAprilTagDetectionArray;
  using ros_message_type = isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosAprilTagDetectionArray,
  isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray);

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_APRIL_TAG_DETECTION_ARRAY_HPP_
