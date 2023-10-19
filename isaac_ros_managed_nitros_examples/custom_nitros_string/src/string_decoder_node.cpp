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

#include "custom_nitros_string/string_decoder_node.hpp"

#include <cuda_runtime.h>

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_view.hpp"

namespace custom_nitros_string
{

StringDecoderNode::StringDecoderNode(const rclcpp::NodeOptions options)
: rclcpp::Node("string_decoder_node", options),
  nitros_sub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosTensorListView>>(
      this,
      "encoded_tensor",
      nvidia::isaac_ros::nitros::nitros_tensor_list_nchw_rgb_f32_t::supported_type_name,
      std::bind(&StringDecoderNode::InputCallback, this,
      std::placeholders::_1))},
  pub_{create_publisher<std_msgs::msg::String>(
      "string_output", 10)},
  tensor_name_{declare_parameter<std::string>("tensor_name", "output_tensor")}
{}

StringDecoderNode::~StringDecoderNode() = default;

void StringDecoderNode::InputCallback(const nvidia::isaac_ros::nitros::NitrosTensorListView & msg)
{
  auto tensor = msg.GetNamedTensor(tensor_name_);

  RCLCPP_INFO(this->get_logger(), "Receiving CUDA buffer with memory at: %p", tensor.GetBuffer());
  RCLCPP_INFO(this->get_logger(), "FrameId: %s", msg.GetFrameId().c_str());
  RCLCPP_INFO(this->get_logger(), "Timestamp seconds: %d", msg.GetTimestampSeconds());
  RCLCPP_INFO(this->get_logger(), "Timestamp nanoseconds: %u", msg.GetTimestampNanoseconds());

  size_t buffer_size{tensor.GetTensorSize()};

  std::string buffer{};
  buffer.resize(buffer_size);

  cudaMemcpy(buffer.data(), tensor.GetBuffer(), buffer_size, cudaMemcpyDefault);

  RCLCPP_INFO(this->get_logger(), "String from CUDA buffer: '%s'", buffer.c_str());

  std_msgs::msg::String string_msg;
  string_msg.data = buffer;
  RCLCPP_INFO(this->get_logger(), "Output String message: '%s'", string_msg.data.c_str());

  pub_->publish(string_msg);
}

}  // namespace custom_nitros_string

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(custom_nitros_string::StringDecoderNode)
