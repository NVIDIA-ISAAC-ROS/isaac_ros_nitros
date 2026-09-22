// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS_BRIDGE_ROS2__IMAGE_CONVERTER_NODE_HPP_
#define ISAAC_ROS_NITROS_BRIDGE_ROS2__IMAGE_CONVERTER_NODE_HPP_

#include <cuda_runtime_api.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "isaac_ros_common/qos.hpp"

#include "ipc_buffer_manager.hpp"
#include "isaac_ros_nitros_bridge_interfaces/msg/nitros_bridge_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

class ImageConverterNode : public rclcpp::Node {
public:
  explicit ImageConverterNode(
    const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~ImageConverterNode();

private:
  // Convert bridge message into a buffer-backed ROS image
  void BridgeToROSCallback(
    const isaac_ros_nitros_bridge_interfaces::msg::
    NitrosBridgeImage::SharedPtr msg);

  // Copy a buffer-backed ROS image into IPC memory and convert to a bridge
  // message
  void ROSToBridgeCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Publisher for output image messages
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  // Publisher for output bridge messages
  rclcpp::Publisher<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>::SharedPtr
    bridge_image_pub_;
  // Subscription to input image messages
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  // Subscription to input bridge messages
  rclcpp::Subscription<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>::SharedPtr
    bridge_image_sub_;

  // Number of blocks of the device memory pool
  int64_t num_blocks_;
  // Timeout in microsec to waiting for a buffer to be available
  int64_t timeout_;
  // Map between FD and device memory pointer
  std::map<int32_t, CUdeviceptr> handle_ptr_map_;
  // CUDA IPC memory pool manager
  std::shared_ptr<IPCBufferManager> ipc_buffer_manager_;
  // Payload size used to create the IPC pool
  size_t ipc_buffer_bytes_{0};
  // If received the first message
  bool first_msg_received_ = false;
  // CUDA driver context
  CUcontext ctx_;
  cudaStream_t cuda_stream_{nullptr};
  // QoS for NITROS bridge publishers and subscribers
  rclcpp::QoS bridge_pub_qos_;
  rclcpp::QoS bridge_sub_qos_;
  // QoS for image publishers and subscribers
  rclcpp::QoS image_pub_qos_;
  rclcpp::QoS image_sub_qos_;
};

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_BRIDGE_ROS2__IMAGE_CONVERTER_NODE_HPP_
