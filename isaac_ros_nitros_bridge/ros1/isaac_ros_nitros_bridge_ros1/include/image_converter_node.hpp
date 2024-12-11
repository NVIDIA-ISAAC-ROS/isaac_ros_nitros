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

#ifndef ISAAC_ROS_NITROS_BRIDGE_ROS1__IMAGE_CONVERTER_NODE_HPP_
#define ISAAC_ROS_NITROS_BRIDGE_ROS1__IMAGE_CONVERTER_NODE_HPP_

#include <map>
#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

#include "ros/ros.h"

#include "ipc_buffer_manager.hpp"
#include "sensor_msgs/Image.h"
#include "isaac_ros_nitros_bridge_msgs/NitrosBridgeImage.h"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

class ImageConverterNode : public nodelet::Nodelet
{
public:
  ImageConverterNode() {}

private:
  virtual void onInit() override;
  
  // Convert stub message into managed NITROS message
  void BridgeToROSCallback(
    const isaac_ros_nitros_bridge_msgs::NitrosBridgeImage::ConstPtr & msg);

  // Copy managed NITROS message data into IPC memory pool and convert to bridge message
  void ROSToBridgeCallback(const sensor_msgs::Image::ConstPtr & msg);

  // Publisher for output image messages
  ros::Publisher image_pub_;
  // Publisher for output bridge messages
  ros::Publisher bridge_image_pub_;
  // Subscription to input image messages
  ros::Subscriber image_sub_;
  // Subscription to input bridge messages
  ros::Subscriber bridge_image_sub_;

  // Number of blocks of the device memory pool
  int num_blocks_;
  // Timeout in microsec to waiting for a buffer to be available
  int timeout_;
  // Map between FD and device memory pointer
  std::map<int32_t, CUdeviceptr> handle_ptr_map_;
  // CUDA IPC memory pool manager
  std::shared_ptr<IPCBufferManager> ipc_buffer_manager_;
  // Shared memory based IPC buffer for refcount and UID
  std::shared_ptr<HostIPCBuffer> host_ipc_buffer_;
  // If received the first message
  bool first_msg_received_ = false;
  // CUDA driver context
  CUcontext ctx_;
  // CUDA IPC event handle sent by the sender
  cudaIpcEventHandle_t ipc_event_handle_;
  // CUDA event export from IPC event to synchronize the upstream
  cudaEvent_t event_;
};

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_BRIDGE_ROS1__IMAGE_CONVERTER_NODE_HPP_
