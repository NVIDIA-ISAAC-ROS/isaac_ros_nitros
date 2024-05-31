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

#ifndef CUSTOM_NITROS_IMAGE__NITROS_IMAGE_SWITCH_NODE_HPP_
#define CUSTOM_NITROS_IMAGE__NITROS_IMAGE_SWITCH_NODE_HPP_

#include <memory>

#include "isaac_ros_common/qos.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_message_filters_subscriber.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace custom_nitros_image
{

class NitrosImageSwitchNode : public rclcpp::Node
{
public:
  explicit NitrosImageSwitchNode(
    const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~NitrosImageSwitchNode();

private:
  void InputCallback(
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & nitros_image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info);

  void TriggerSwitchService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  uint16_t sync_queue_size_;
  rclcpp::QoS output_qos_;

  nvidia::isaac_ros::nitros::message_filters::Subscriber<nvidia::isaac_ros::nitros::NitrosImageView>
  sub_image_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_;

  using ExactPolicy = message_filters::sync_policies::ExactTime<
    nvidia::isaac_ros::nitros::NitrosImage,
    sensor_msgs::msg::CameraInfo
  >;
  message_filters::Synchronizer<ExactPolicy> sync_;

  std::shared_ptr<::nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      ::nvidia::isaac_ros::nitros::NitrosImage>>
  pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr empty_service_;

  bool publish_{};
};

}  // namespace custom_nitros_image
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // CUSTOM_NITROS_IMAGE__NITROS_IMAGE_SWITCH_NODE_HPP_
