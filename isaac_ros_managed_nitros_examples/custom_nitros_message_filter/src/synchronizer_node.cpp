// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "custom_nitros_message_filter/synchronizer_node.hpp"

#include <cuda_runtime.h>

namespace custom_nitros_message_filter
{

SynchronizerNode::SynchronizerNode(const rclcpp::NodeOptions options)
: rclcpp::Node("synchronizer_node", options),
  sub_image_{},
  sub_camera_info_{},
  sync_{ExactPolicy{3}, sub_image_, sub_camera_info_},
  pub_{create_publisher<custom_nitros_message_filter_interfaces::msg::SyncStatus>(
      "status", 3)}
{
  sync_.registerCallback(
    std::bind(
      &SynchronizerNode::InputCallback, this, true,
      std::placeholders::_1, std::placeholders::_2));
  sync_.getPolicy()->registerDropCallback(
    std::bind(
      &SynchronizerNode::InputCallback, this, false,
      std::placeholders::_1, std::placeholders::_2));

  sub_image_.subscribe(this, "image");
  sub_camera_info_.subscribe(this, "camera_info");
}

SynchronizerNode::~SynchronizerNode() = default;

void SynchronizerNode::InputCallback(
  bool synchronized_inputs,
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & nitros_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info
)
{
  RCLCPP_DEBUG(
    this->get_logger(), "%s callback fired!",
    (synchronized_inputs ? "Synchronized" : "UNSYNCHRONIZED"));

  custom_nitros_message_filter_interfaces::msg::SyncStatus status{};
  status.messages_present = std::vector<bool>(2, false);
  status.exact_time_match = false;

  if (nitros_image && camera_info) {
    RCLCPP_DEBUG(this->get_logger(), "Received both Nitros Image and Camera Info!");
    status.messages_present.at(0) = true;
    status.messages_present.at(1) = true;

    auto nitros_image_view = nvidia::isaac_ros::nitros::NitrosImageView(*nitros_image);
    if (nitros_image_view.GetTimestampSeconds() == camera_info->header.stamp.sec &&
      nitros_image_view.GetTimestampNanoseconds() == camera_info->header.stamp.nanosec)
    {
      status.exact_time_match = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Both messages received, but timestamps didn't match!");
    }

    status.stamp = camera_info->header.stamp;
  } else if (nitros_image) {
    RCLCPP_DEBUG(this->get_logger(), "Received only Nitros Image!");
    status.messages_present.at(0) = true;

    auto nitros_image_view = nvidia::isaac_ros::nitros::NitrosImageView(*nitros_image);
    status.stamp.sec = nitros_image_view.GetTimestampSeconds();
    status.stamp.nanosec = nitros_image_view.GetTimestampNanoseconds();
  } else if (camera_info) {
    RCLCPP_DEBUG(this->get_logger(), "Received only Camera Info!");
    status.messages_present.at(1) = true;

    status.stamp = camera_info->header.stamp;
  }

  RCLCPP_DEBUG(
    this->get_logger(), "Sending Status message with stamp: %d.%u", status.stamp.sec,
    status.stamp.nanosec);

  pub_->publish(status);
}

}  // namespace custom_nitros_message_filter

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(custom_nitros_message_filter::SynchronizerNode)
