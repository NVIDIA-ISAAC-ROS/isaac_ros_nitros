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

#include "custom_nitros_image/gpu_image_builder_node.hpp"

#include <cuda_runtime.h>

#include "isaac_ros_nitros_image_type/nitros_image_builder.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace custom_nitros_image
{

GpuImageBuilderNode::GpuImageBuilderNode(const rclcpp::NodeOptions options)
: rclcpp::Node("image_builder_node", options),
  sub_{create_subscription<sensor_msgs::msg::Image>(
      "image_input", 10,
      std::bind(&GpuImageBuilderNode::InputCallback, this, std::placeholders::_1))},
  nitros_pub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
        nvidia::isaac_ros::nitros::NitrosImage>>(
      this, "gpu_image",
      nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name)} {}

GpuImageBuilderNode::~GpuImageBuilderNode() = default;

void GpuImageBuilderNode::InputCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Get size of image
  size_t buffer_size{msg->step * msg->height};

  // Allocate CUDA buffer to store image
  void * buffer;
  cudaMalloc(&buffer, buffer_size);

  // Copy data bytes to CUDA buffer
  cudaMemcpy(buffer, msg->data.data(), buffer_size, cudaMemcpyDefault);

  // Adding header data
  std_msgs::msg::Header header;
  header.stamp.sec = 123456;
  header.stamp.nanosec = 789101112;
  header.frame_id = "cuda_image";

  // Create NitrosImage wrapping CUDA buffer
  nvidia::isaac_ros::nitros::NitrosImage nitros_image =
    nvidia::isaac_ros::nitros::NitrosImageBuilder()
    .WithHeader(header)
    .WithEncoding(img_encodings::RGB8)
    .WithDimensions(msg->height, msg->width)
    .WithGpuData(buffer)
    .Build();

  nitros_pub_->publish(nitros_image);
  RCLCPP_INFO(this->get_logger(), "Sent CUDA buffer with memory at: %p", buffer);
}

}  // namespace custom_nitros_image

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(custom_nitros_image::GpuImageBuilderNode)
