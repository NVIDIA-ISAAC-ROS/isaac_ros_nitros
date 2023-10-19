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

#include <cuda_runtime.h>

#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_builder.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_builder.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NitrosTensorListBuilder::NitrosTensorListBuilder()
: nitros_tensor_list_{}
{
  auto message = nvidia::gxf::Entity::New(GetTypeAdapterNitrosContext().getContext());
  if (!message) {
    std::stringstream error_msg;
    error_msg <<
      "[constructor] Error initializing new message entity: " <<
      GxfResultStr(message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTensorListBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  nitros_tensor_list_.handle = message->eid();
  GxfEntityRefCountInc(
    nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext(), message->eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorListBuilder"),
    "[constructor] NitrosTensorList initialized");
}

NitrosTensorListBuilder & NitrosTensorListBuilder::WithHeader(std_msgs::msg::Header header)
{
  auto message = nvidia::gxf::Entity::Shared(
    GetTypeAdapterNitrosContext().getContext(), nitros_tensor_list_.handle);

  // Set timestamp
  auto output_timestamp = message->add<nvidia::gxf::Timestamp>("timestamp");
  if (!output_timestamp) {
    std::stringstream error_msg;
    error_msg << "[WithHeader] Failed to add a timestamp component to message: " <<
      GxfResultStr(output_timestamp.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTensorListBuilder"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  output_timestamp.value()->acqtime = header.stamp.sec * static_cast<uint64_t>(1e9) +
    header.stamp.nanosec;

  // Set frame ID
  nitros_tensor_list_.frame_id = header.frame_id;

  return *this;
}

NitrosTensorListBuilder & NitrosTensorListBuilder::AddTensor(std::string name, NitrosTensor tensor)
{
  // Get the gxf::Tensor attached to the NitrosTensor
  auto tensor_message = nvidia::gxf::Entity::Shared(
    GetTypeAdapterNitrosContext().getContext(), tensor.handle);

  auto gxf_tensor = tensor_message->get<nvidia::gxf::Tensor>();

  // Move the gxf::Tensor to the NitrosTensorList
  auto tensor_list_message = nvidia::gxf::Entity::Shared(
    GetTypeAdapterNitrosContext().getContext(), nitros_tensor_list_.handle);
  auto new_gxf_tensor = tensor_list_message->add<nvidia::gxf::Tensor>(name.c_str());
  *new_gxf_tensor.value() = std::move(*gxf_tensor.value());

  return *this;
}

NitrosTensorList NitrosTensorListBuilder::Build()
{
  return nitros_tensor_list_;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
