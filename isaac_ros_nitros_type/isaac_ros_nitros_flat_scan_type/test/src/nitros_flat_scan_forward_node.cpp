// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros_flat_scan_type/nitros_flat_scan.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

constexpr char PACKAGE_NAME[] = "isaac_ros_nitros_flat_scan_type";
constexpr char FORWARD_FORMAT[] = "nitros_flat_scan";
class NitrosFlatScanForwardNode : public rclcpp::Node
{
public:
  explicit NitrosFlatScanForwardNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("NitrosFlatScanForwardNode", options)
  {
    auto qos = rclcpp::QoS(1);
    sub_ = create_subscription<nvidia::isaac_ros::nitros::NitrosFlatScan>(
      "topic_forward_input", qos,
      std::bind(&NitrosFlatScanForwardNode::on_flat_scan, this, std::placeholders::_1));
    pub_ = create_publisher<nvidia::isaac_ros::nitros::NitrosFlatScan>(
      "topic_forward_output", qos);
  }

private:
  void on_flat_scan(const nvidia::isaac_ros::nitros::NitrosFlatScan & msg)
  {
    pub_->publish(msg);
  }

  rclcpp::Subscription<nvidia::isaac_ros::nitros::NitrosFlatScan>::SharedPtr sub_;
  rclcpp::Publisher<nvidia::isaac_ros::nitros::NitrosFlatScan>::SharedPtr pub_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosFlatScanForwardNode)
