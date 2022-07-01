/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "isaac_ros_nitros/types/type_utility.hpp"
#include "isaac_ros_nitros/types/types.hpp"

#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

std::string getROSTypeNameByDataFormat(const std::string data_format)
{
  #define GET_ROS_TYPE_NAME_HELPER(DATA_TYPE) \
  if (data_format == DATA_TYPE::supported_type_name) { \
    using ROSMessageType = typename rclcpp::TypeAdapter<DATA_TYPE::MsgT>::ros_message_type; \
    return rosidl_generator_traits::name<ROSMessageType>(); \
  }
  FOREACH_NITROS_DATA_FORMAT(GET_ROS_TYPE_NAME_HELPER);
  return "";
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
