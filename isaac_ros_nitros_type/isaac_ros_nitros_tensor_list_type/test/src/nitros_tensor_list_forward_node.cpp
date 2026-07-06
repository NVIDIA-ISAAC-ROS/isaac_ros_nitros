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

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_builder.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_builder.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{
class NitrosTensorListForwardNode : public rclcpp::Node
{
public:
  explicit NitrosTensorListForwardNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("NitrosTensorListForwardNode", options)
  {
    RCLCPP_DEBUG(get_logger(), "NitrosTensorListForwardNode constructor");

    tensor_list_sub_ = create_subscription<nvidia::isaac_ros::nitros::NitrosTensorList>(
      "tensor_list_input", 10, std::bind(&NitrosTensorListForwardNode::TensorListCallback,
        this, std::placeholders::_1));
    tensor_list_pub_ = create_publisher<nvidia::isaac_ros::nitros::NitrosTensorList>(
      "tensor_list_output", 10);
  }

  void TensorListCallback(const nvidia::isaac_ros::nitros::NitrosTensorList::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "TensorListCallback");

    // Create a new tensor list with updated names
    NitrosTensorListBuilder builder;
    std::string name = "tensor_list_output";
    for (size_t i = 0; i < msg->num_tensors(); i++) {
      nvidia::isaac_ros::nitros::NitrosTensor tensor = msg->get_tensor(i);
      tensor.set_name(name);
      builder.AddTensor(tensor);
    }
    std_msgs::msg::Header header = msg->get_header();
    builder.WithHeader(header);
    auto tensor_list = builder.Build();
    tensor_list_pub_->publish(tensor_list);
  }

private:
  rclcpp::Subscription<nvidia::isaac_ros::nitros::NitrosTensorList>::SharedPtr tensor_list_sub_;
  rclcpp::Publisher<nvidia::isaac_ros::nitros::NitrosTensorList>::SharedPtr tensor_list_pub_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosTensorListForwardNode)
