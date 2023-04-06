/**
 * Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS_IMU_TYPE__NITROS_IMU_HPP_
#define ISAAC_ROS_NITROS_IMU_TYPE__NITROS_IMU_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosImu
 *   ROS type:    sensor_msgs::msg::Imu
 */

#include <string>

#include "isaac_ros_nitros/types/nitros_format_agent.hpp"

#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/imu.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type forward declaration
struct NitrosImu;

// Formats
struct nitros_imu_t
{
  using MsgT = NitrosImu;
  static const inline std::string supported_type_name = "nitros_imu";
};

// NITROS data type registration factory
NITROS_TYPE_FACTORY_BEGIN(NitrosImu)
// Supported data formats
NITROS_FORMAT_FACTORY_BEGIN()
NITROS_FORMAT_ADD(nitros_imu_t)
NITROS_FORMAT_FACTORY_END()
// Required extensions
NITROS_TYPE_EXTENSION_FACTORY_BEGIN()
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so")
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/libgxf_isaac_messages.so")
NITROS_TYPE_EXTENSION_FACTORY_END()
NITROS_TYPE_FACTORY_END()

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia


template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosImu,
  sensor_msgs::msg::Imu>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosImu;
  using ros_message_type = sensor_msgs::msg::Imu;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosImu,
  sensor_msgs::msg::Imu);

#endif  // ISAAC_ROS_NITROS_IMU_TYPE__NITROS_IMU_HPP_
