// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "custom_nitros_string/string_encoder_node.hpp"

#include <cuda_runtime.h>
#include <string>

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_builder.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_builder.hpp"

namespace custom_nitros_string
{

StringEncoderNode::StringEncoderNode(const rclcpp::NodeOptions options)
: rclcpp::Node("string_encoder_node", options),
  sub_{create_subscription<std_msgs::msg::String>(
      "string_input", 10, std::bind(&StringEncoderNode::InputCallback, this,
      std::placeholders::_1))},
  nitros_pub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
        nvidia::isaac_ros::nitros::NitrosTensorList>>(
      this, "encoded_tensor",
      nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name)},
  tensor_name_{declare_parameter<std::string>("tensor_name", "input_tensor")}
{}

StringEncoderNode::~StringEncoderNode() = default;

void StringEncoderNode::InputCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Input String message: '%s'", msg->data.c_str());

  // Get size of string, accounting for null terminator byte
  size_t buffer_size{msg->data.size() + 1};

  // Allocate CUDA buffer to encode string
  void * buffer;
  cudaMalloc(&buffer, buffer_size);

  // Copy string's bytes to CUDA buffer
  cudaMemcpy(buffer, msg->data.data(), buffer_size, cudaMemcpyDefault);

  // Adding header data
  std_msgs::msg::Header header;
  header.stamp.sec = 123456;
  header.stamp.nanosec = 789101112;
  header.frame_id = tensor_name_;

  // Create tensor list with tensor wrapping CUDA buffer
  nvidia::isaac_ros::nitros::NitrosTensorList tensor_list =
    nvidia::isaac_ros::nitros::NitrosTensorListBuilder()
    .WithHeader(header)
    .AddTensor(
    tensor_name_,
    (
      nvidia::isaac_ros::nitros::NitrosTensorBuilder()
      .WithShape({static_cast<int>(buffer_size)})
      .WithDataType(nvidia::isaac_ros::nitros::NitrosDataType::kUnsigned8)  // sizeof(uint8_t) == 1
      .WithData(buffer)
      .Build()
    )
    )
    .Build();

  RCLCPP_INFO(this->get_logger(), "Sending CUDA buffer with memory at: %p", buffer);

  nitros_pub_->publish(tensor_list);
}

}  // namespace custom_nitros_string

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(custom_nitros_string::StringEncoderNode)
