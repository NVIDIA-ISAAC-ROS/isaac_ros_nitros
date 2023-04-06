/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosTypeBase::NitrosTypeBase(
  const int64_t handle,
  const std::string data_format_name,
  const std::string compatible_data_format_name,
  const std::string frame_id)
: handle(handle),
  data_format_name(data_format_name),
  compatible_data_format_name(compatible_data_format_name),
  frame_id(frame_id)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTypeBase"),
    "[Constructor] Creating a Nitros-typed object for handle = %ld",
    handle);
  GxfEntityRefCountInc(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext(),
    handle);
}

NitrosTypeBase::NitrosTypeBase(const NitrosTypeBase & other)
: handle(other.handle),
  data_format_name(other.data_format_name),
  compatible_data_format_name(other.compatible_data_format_name),
  frame_id(other.frame_id)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTypeBase"),
    "[Copy Constructor] Copying a Nitros-typed object for handle = %ld",
    other.handle);
  GxfEntityRefCountInc(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext(),
    other.handle);
}

NitrosTypeBase::~NitrosTypeBase()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTypeBase"),
    "[Destructor]Dstroying a Nitros-typed object for handle = %ld",
    handle);
  GxfEntityRefCountDec(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext(),
    handle);
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
