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

#ifndef CUSTOM_NITROS_IMAGE__GPU_IMAGE_VIEWER_NODE_HPP_
#define CUSTOM_NITROS_IMAGE__GPU_IMAGE_VIEWER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"

namespace custom_nitros_image
{

class GpuImageViewerNode : public rclcpp::Node
{
public:
  explicit GpuImageViewerNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~GpuImageViewerNode();

private:
  void InputCallback(const nvidia::isaac_ros::nitros::NitrosImageView & msg);

  // Subscription to input NitrosImage messages
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
      nvidia::isaac_ros::nitros::NitrosImageView>> nitros_sub_;

  // Publisher for output Image messages
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}  // namespace custom_nitros_image

#endif  // CUSTOM_NITROS_IMAGE__GPU_IMAGE_VIEWER_NODE_HPP_
