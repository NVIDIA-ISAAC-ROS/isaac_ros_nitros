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

#include "isaac_ros_nitros_compressed_image_type/nitros_compressed_image.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosCompressedImageForwardNode : public rclcpp::Node
{
public:
  explicit NitrosCompressedImageForwardNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("NitrosCompressedImageForwardNode", options)
  {
    auto qos = rclcpp::QoS(1);
    sub_ = create_subscription<NitrosCompressedImage>(
      "topic_forward_input", qos,
      std::bind(&NitrosCompressedImageForwardNode::on_msg, this, std::placeholders::_1));
    pub_ = create_publisher<NitrosCompressedImage>(
      "topic_forward_output", qos);
  }

private:
  void on_msg(const NitrosCompressedImage & msg)
  {
    pub_->publish(msg);
  }

  rclcpp::Subscription<NitrosCompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<NitrosCompressedImage>::SharedPtr pub_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosCompressedImageForwardNode)
