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

#ifndef ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_SUBSCRIBER_HPP_
#define ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_SUBSCRIBER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "extensions/gxf_optimizer/core/optimizer.hpp"
#include "isaac_ros_nitros/nitros_subscriber.hpp"
#include "isaac_ros_nitros/types/nitros_type_manager.hpp"
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
    const NitrosStatisticsConfig & statistics_config = {})
  : node_{node}, topic_{topic_name},
    nitros_type_manager_{std::make_shared<NitrosTypeManager>(node_)}
  {
    nitros_type_manager_->registerSupportedType<typename NitrosMsgView::BaseType>();
    nitros_type_manager_->loadExtensions(format);

    std::vector<std::string> supported_data_formats{format};

    NitrosPublisherSubscriberConfig component_config{
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(1),
      .compatible_data_format = format,
      .topic_name = topic_name,
      .callback = [callback](const gxf_context_t, NitrosTypeBase & msg) -> void {
          const NitrosMsgView view(*(static_cast<typename NitrosMsgView::BaseType *>(&msg)));
          callback(view);
        }
    };

    nitros_sub_ = std::make_shared<NitrosSubscriber>(
      *node_, GetTypeAdapterNitrosContext().getContext(), nitros_type_manager_,
      supported_data_formats, component_config, statistics_config);

    nitros_sub_->start();

    RCLCPP_INFO(
      node_->get_logger().get_child("ManagedNitrosSubscriber"),
      "Starting Managed Nitros Subscriber");
  }

private:
  rclcpp::Node * node_;
  std::string topic_;
  std::shared_ptr<NitrosTypeManager> nitros_type_manager_;
  std::shared_ptr<NitrosSubscriber> nitros_sub_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_MANAGED_NITROS__MANAGED_NITROS_SUBSCRIBER_HPP_
