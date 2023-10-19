// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

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
