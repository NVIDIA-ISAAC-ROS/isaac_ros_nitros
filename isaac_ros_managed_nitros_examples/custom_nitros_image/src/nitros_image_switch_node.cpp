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

#include "custom_nitros_image/nitros_image_switch_node.hpp"

#include "isaac_ros_nitros_image_type/nitros_image_builder.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace custom_nitros_image
{

NitrosImageSwitchNode::NitrosImageSwitchNode(const rclcpp::NodeOptions options)
: rclcpp::Node("nitros_image_switch", options),
  sync_queue_size_{declare_parameter<uint16_t>("sync_queue_size", static_cast<uint16_t>(10))},
  output_qos_{
    ::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "output_qos")},
  sub_image_{},
  sub_camera_info_{},
  sync_{ExactPolicy{100}, sub_image_, sub_camera_info_},
  pub_image_{std::make_shared<
      nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
        nvidia::isaac_ros::nitros::NitrosImage>>(
      this, "switched_image",
      ::nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
      nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig{}, output_qos_)},
  pub_camera_info_{create_publisher<sensor_msgs::msg::CameraInfo>(
      "switched_camera_info",
      output_qos_)},
  empty_service_{create_service<std_srvs::srv::SetBool>(
      "switch",
      std::bind(&NitrosImageSwitchNode::TriggerSwitchService, this,
      std::placeholders::_1, std::placeholders::_2))},
  publish_{declare_parameter<bool>("initial_switch_state", true)}
{
  sync_.registerCallback(
    std::bind(
      &NitrosImageSwitchNode::InputCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  sub_image_.subscribe(this, "image");
  sub_camera_info_.subscribe(this, "camera_info");
}

void NitrosImageSwitchNode::InputCallback(
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & nitros_image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  if (publish_) {
    pub_image_->publish(*nitros_image);
    pub_camera_info_->publish(*camera_info);
  }
}

void NitrosImageSwitchNode::TriggerSwitchService(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  publish_ = request->data;
  response->success = true;
  response->message = "";
  RCLCPP_DEBUG(
    get_logger(),
    "Triggered service! Passthrough switch is set to: %s",
    publish_ ? "ON" : "OFF");
}

NitrosImageSwitchNode::~NitrosImageSwitchNode() = default;

}  // namespace custom_nitros_image
}  // namespace isaac_ros
}  // namespace nvidia

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::custom_nitros_image::NitrosImageSwitchNode)
