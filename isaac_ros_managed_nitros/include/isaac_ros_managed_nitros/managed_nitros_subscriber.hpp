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

#ifndef ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_SUBSCRIBER_HPP_
#define ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_SUBSCRIBER_HPP_

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

template<typename NitrosMsgView>
class ManagedNitrosSubscriber
{
public:
  explicit ManagedNitrosSubscriber(
    rclcpp::Node * node,
    const std::string & topic_name,
    const std::string & format,
    std::function<void(const NitrosMsgView & msg_view)> callback = nullptr,
    const rclcpp::QoS qos = rclcpp::QoS(1)) __attribute__((deprecated(
      "Deprecated. Use rclcpp::create_subscription instead.")))
  : node_{node}, topic_{topic_name}
  {
    if constexpr (IsNitrosBufferBased<typename NitrosMsgView::BaseType>::value) {
      rclcpp::SubscriptionOptions sub_options;
      sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

      auto ros_callback =
        [callback](const std::shared_ptr<const typename NitrosMsgView::BaseType> msg) {
          const NitrosMsgView view(*msg);
          callback(view);
        };

      ros_sub_ = node_->template create_subscription<typename NitrosMsgView::BaseType>(
        topic_name, qos, ros_callback, sub_options);

      RCLCPP_INFO(
        node_->get_logger().get_child("ManagedNitrosSubscriber"),
        "Starting Managed Nitros Subscriber (GXF-free)");
    } else {
      throw std::runtime_error("ManagedNitrosSubscriber does not support non-buffer-based types");
    }
  }

private:
  rclcpp::Node * node_;
  std::string topic_;
  std::shared_ptr<rclcpp::Subscription<typename NitrosMsgView::BaseType>> ros_sub_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_SUBSCRIBER_HPP_
