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

#ifndef CUSTOM_NITROS_MESSAGE_FILTER__SYNCHRONIZER_NODE_HPP_
#define CUSTOM_NITROS_MESSAGE_FILTER__SYNCHRONIZER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"

#include "custom_nitros_message_filter_interfaces/msg/sync_status.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_message_filters_subscriber.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


namespace custom_nitros_message_filter
{

class SynchronizerNode : public rclcpp::Node
{
public:
  explicit SynchronizerNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~SynchronizerNode();

private:
  void InputCallback(
    bool synchronized_inputs,
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & nitros_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info
  );

  // Synchronized NITROS Image and standard CameraInfo subscribers
  nvidia::isaac_ros::nitros::message_filters::Subscriber<nvidia::isaac_ros::nitros::NitrosImageView>
  sub_image_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_;

  using ExactPolicy = message_filters::sync_policies::ExactTime<
    nvidia::isaac_ros::nitros::NitrosImage,
    sensor_msgs::msg::CameraInfo
  >;
  message_filters::Synchronizer<ExactPolicy> sync_;

  // Publisher for output status messages
  rclcpp::Publisher<custom_nitros_message_filter_interfaces::msg::SyncStatus>::SharedPtr pub_;
};

}  // namespace custom_nitros_message_filter

#endif  // CUSTOM_NITROS_MESSAGE_FILTER__SYNCHRONIZER_NODE_HPP_
