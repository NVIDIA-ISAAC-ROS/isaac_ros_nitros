// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "message_compositor/message_relay.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "gxf/std/vault.hpp"
#pragma GCC diagnostic pop

#include "extensions/gxf_optimizer/core/optimizer.hpp"
#include "extensions/gxf_optimizer/exporter/graph_types.hpp"

#include "isaac_ros_nitros/nitros_publisher.hpp"
#include "isaac_ros_nitros/types/nitros_type_manager.hpp"
#include "isaac_ros_nitros/nitros_publisher_subscriber_group.hpp"
#include "isaac_ros_nitros/nitros_context.hpp"
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
    const NitrosStatisticsConfig & statistics_config = {})
  : node_{node},
    context_{GetTypeAdapterNitrosContext()},
    nitros_type_manager_{std::make_shared<NitrosTypeManager>(node_)}
  {
    nitros_type_manager_->registerSupportedType<T>();
    nitros_type_manager_->loadExtensions(format);

    std::vector<std::string> supported_data_formats{format};

    NitrosPublisherSubscriberConfig component_config{
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(1),
      .compatible_data_format = format,
      .topic_name = topic
    };

    nitros_pub_ = std::make_shared<NitrosPublisher>(
      *node_, GetTypeAdapterNitrosContext().getContext(), nitros_type_manager_,
      supported_data_formats, component_config, statistics_config);

    nitros_pub_->start();

    RCLCPP_INFO(
      node_->get_logger().get_child("ManagedNitrosPublisher"),
      "Starting Managed Nitros Publisher");
  }

  void publish(T msg)
  {
    nitros_pub_->publish(msg);
  }

private:
  rclcpp::Node * node_;
  NitrosContext context_;
  std::shared_ptr<NitrosTypeManager> nitros_type_manager_;
  std::shared_ptr<NitrosPublisher> nitros_pub_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_PUBLISHER_HPP_
