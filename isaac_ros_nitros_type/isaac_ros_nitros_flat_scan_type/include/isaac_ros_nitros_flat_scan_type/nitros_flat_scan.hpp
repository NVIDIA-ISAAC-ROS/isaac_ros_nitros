/**
 * Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_NITROS_FLAT_SCAN_TYPE__NITROS_FLAT_SCAN_HPP_
#define ISAAC_ROS_NITROS_FLAT_SCAN_TYPE__NITROS_FLAT_SCAN_HPP_
/*
 * Type adaptation for:
 *   Nitros type: NitrosFlatScan
 *   ROS type:    isaac_ros_pointcloud_interfaces::msg::FlatScan
 */

#include <string>

#include "isaac_ros_nitros/types/nitros_format_agent.hpp"

#include "rclcpp/type_adapter.hpp"
#include "isaac_ros_pointcloud_interfaces/msg/flat_scan.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type forward declaration
struct NitrosFlatScan;

// Formats
struct nitros_flat_scan_t
{
  using MsgT = NitrosFlatScan;
  static const inline std::string supported_type_name = "nitros_flat_scan";
};

// NITROS data type registration factory
NITROS_TYPE_FACTORY_BEGIN(NitrosFlatScan)
// Supported data formats
NITROS_FORMAT_FACTORY_BEGIN()
NITROS_FORMAT_ADD(nitros_flat_scan_t)
NITROS_FORMAT_FACTORY_END()
// Required extensions
NITROS_TYPE_EXTENSION_FACTORY_BEGIN()
NITROS_TYPE_EXTENSION_ADD("isaac_ros_gxf", "gxf/lib/std/libgxf_std.so")
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
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosFlatScan,
  isaac_ros_pointcloud_interfaces::msg::FlatScan>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosFlatScan;
  using ros_message_type = isaac_ros_pointcloud_interfaces::msg::FlatScan;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosFlatScan,
  isaac_ros_pointcloud_interfaces::msg::FlatScan);

#endif  // ISAAC_ROS_NITROS_FLAT_SCAN_TYPE__NITROS_FLAT_SCAN_HPP_
