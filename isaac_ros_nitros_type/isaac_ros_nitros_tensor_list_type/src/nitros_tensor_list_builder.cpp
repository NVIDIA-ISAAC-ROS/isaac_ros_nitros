// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_builder.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_builder.hpp"
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
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosTensorListBuilder"),
    "[constructor] NitrosTensorList initialized");
}

NitrosTensorListBuilder & NitrosTensorListBuilder::WithHeader(std_msgs::msg::Header header)
{
  nitros_tensor_list_.set_timestamp_sec(header.stamp.sec);
  nitros_tensor_list_.set_timestamp_nsec(header.stamp.nanosec);
  nitros_tensor_list_.set_frame_id(header.frame_id);

  return *this;
}

NitrosTensorListBuilder & NitrosTensorListBuilder::AddTensor(NitrosTensor tensor)
{
  nitros_tensor_list_.add_tensor(tensor);
  return *this;
}

NitrosTensorListBuilder & NitrosTensorListBuilder::AddTensor(
  const std::string & name, NitrosTensor tensor)
{
  tensor.set_name(name);
  nitros_tensor_list_.add_tensor(std::move(tensor));
  return *this;
}

NitrosTensorList NitrosTensorListBuilder::Build()
{
  return nitros_tensor_list_;
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
