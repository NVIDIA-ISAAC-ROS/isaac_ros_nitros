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

#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

constexpr char PACKAGE_NAME[] = "isaac_ros_nitros_point_cloud_type";
constexpr char FORWARD_FORMAT[] = "nitros_point_cloud";

class NitrosPointCloudForwardNode : public rclcpp::Node
{
public:
  explicit NitrosPointCloudForwardNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("NitrosPointCloudForwardNode", options)
  {
    RCLCPP_INFO(get_logger(), "NitrosPointCloudForwardNode constructor");
    subscriber_ = create_subscription<nvidia::isaac_ros::nitros::NitrosPointCloud>(
      "topic_forward_input", rclcpp::QoS(1), std::bind(&NitrosPointCloudForwardNode::callback,
        this, std::placeholders::_1));
    publisher_ = create_publisher<nvidia::isaac_ros::nitros::NitrosPointCloud>(
      "topic_forward_output", rclcpp::QoS(1));
  }

private:
  void callback(const nvidia::isaac_ros::nitros::NitrosPointCloud & msg)
  {
    publisher_->publish(msg);
  }

  rclcpp::Subscription<nvidia::isaac_ros::nitros::NitrosPointCloud>::SharedPtr subscriber_;
  rclcpp::Publisher<nvidia::isaac_ros::nitros::NitrosPointCloud>::SharedPtr publisher_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosPointCloudForwardNode)
