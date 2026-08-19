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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/empty.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosEmptyForwardNode : public rclcpp::Node
{
public:
  explicit NitrosEmptyForwardNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("NitrosEmptyForwardNode", options)
  {
    auto qos = rclcpp::QoS(1);
    sub_ = create_subscription<std_msgs::msg::Empty>(
      "topic_forward_input", qos,
      std::bind(&NitrosEmptyForwardNode::on_empty, this, std::placeholders::_1));
    pub_ = create_publisher<std_msgs::msg::Empty>(
      "topic_forward_output", qos);
  }

private:
  void on_empty(const std_msgs::msg::Empty & msg)
  {
    pub_->publish(msg);
  }
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosEmptyForwardNode)
