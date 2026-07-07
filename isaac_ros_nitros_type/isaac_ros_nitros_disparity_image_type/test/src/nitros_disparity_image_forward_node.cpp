// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros_disparity_image_type/nitros_disparity_image.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{
class NitrosDisparityImageForwardNode : public rclcpp::Node
{
public:
  explicit NitrosDisparityImageForwardNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("NitrosDisparityImageForwardNode", options)
  {
    RCLCPP_INFO(get_logger(), "NitrosDisparityImageForwardNode constructor");

    subscriber_ = create_subscription<nvidia::isaac_ros::nitros::NitrosDisparityImage>(
      "topic_forward_input", rclcpp::QoS(1), std::bind(&NitrosDisparityImageForwardNode::callback,
        this, std::placeholders::_1));
    publisher_ = create_publisher<nvidia::isaac_ros::nitros::NitrosDisparityImage>(
      "topic_forward_output", rclcpp::QoS(1));
  }

private:
  void callback(const nvidia::isaac_ros::nitros::NitrosDisparityImage & msg)
  {
    publisher_->publish(msg);
  }

  rclcpp::Subscription<nvidia::isaac_ros::nitros::NitrosDisparityImage>::SharedPtr subscriber_;
  rclcpp::Publisher<nvidia::isaac_ros::nitros::NitrosDisparityImage>::SharedPtr publisher_;
};


}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosDisparityImageForwardNode)
