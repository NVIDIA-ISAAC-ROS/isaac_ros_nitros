// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_PUBLISHER_HPP_
#define ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "isaac_ros_nitros/types/nitros_type_view.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

template<typename T>
class ManagedNitrosPublisher
{
public:
  ManagedNitrosPublisher(
    rclcpp::Node * node,
    const std::string & topic,
    const std::string & format,
    const rclcpp::QoS qos = rclcpp::QoS(1))
  : node_{node}
  {
    if constexpr (IsNitrosBufferBased<T>::value) {
      rclcpp::PublisherOptions pub_options;
      pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
      ros_pub_ = node_->template create_publisher<T>(topic, qos, pub_options);

      RCLCPP_INFO(
        node_->get_logger().get_child("ManagedNitrosPublisher"),
        "Starting Managed Nitros Publisher (GXF-free)");
    } else {
      throw std::runtime_error("ManagedNitrosPublisher does not support non-buffer-based types");
    }
  }

  void publish(T msg)
  {
    if constexpr (IsNitrosBufferBased<T>::value) {
      ros_pub_->publish(msg);
    } else {
      throw std::runtime_error("ManagedNitrosPublisher does not support non-buffer-based types");
    }
  }

private:
  rclcpp::Node * node_;
  std::shared_ptr<rclcpp::Publisher<T>> ros_pub_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_PUBLISHER_HPP_
