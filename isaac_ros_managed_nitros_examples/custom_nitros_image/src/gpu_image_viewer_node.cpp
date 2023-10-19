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

#include "custom_nitros_image/gpu_image_viewer_node.hpp"

#include <cuda_runtime.h>

#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"

namespace custom_nitros_image
{

GpuImageViewerNode::GpuImageViewerNode(const rclcpp::NodeOptions options)
: rclcpp::Node("image_viewer_node", options),
  nitros_sub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosImageView>>(
      this, "gpu_image", nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
      std::bind(&GpuImageViewerNode::InputCallback, this,
      std::placeholders::_1))},
  pub_{create_publisher<sensor_msgs::msg::Image>("image_output", 10)} {}

GpuImageViewerNode::~GpuImageViewerNode() = default;

void GpuImageViewerNode::InputCallback(const nvidia::isaac_ros::nitros::NitrosImageView & view)
{
  RCLCPP_INFO(this->get_logger(), "Receiving CUDA buffer with memory at: %p", view.GetGpuData());
  RCLCPP_INFO(this->get_logger(), "FrameId: %s", view.GetFrameId().c_str());
  RCLCPP_INFO(this->get_logger(), "Timestamp seconds: %d", view.GetTimestampSeconds());
  RCLCPP_INFO(this->get_logger(), "Timestamp nanoseconds: %u", view.GetTimestampNanoseconds());
  RCLCPP_INFO(this->get_logger(), "Size: %lu", view.GetSizeInBytes());

  sensor_msgs::msg::Image img_msg;
  img_msg.header.frame_id = view.GetFrameId();
  img_msg.header.stamp.sec = view.GetTimestampSeconds();
  img_msg.header.stamp.nanosec = view.GetTimestampNanoseconds();
  img_msg.height = view.GetHeight();
  img_msg.width = view.GetWidth();
  img_msg.encoding = view.GetEncoding();
  img_msg.step = view.GetSizeInBytes() / view.GetHeight();

  img_msg.data.resize(view.GetSizeInBytes());
  cudaMemcpy(img_msg.data.data(), view.GetGpuData(), view.GetSizeInBytes(), cudaMemcpyDefault);

  pub_->publish(std::move(img_msg));
}

}  // namespace custom_nitros_image

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(custom_nitros_image::GpuImageViewerNode)
