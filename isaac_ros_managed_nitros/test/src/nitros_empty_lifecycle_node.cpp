// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <memory>
#include <string>

#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros/types/nitros_empty.hpp"
#include "isaac_ros_nitros/types/nitros_type_view_factory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Minimal view wrapper for NitrosEmpty, required by ManagedNitrosSubscriber.
NITROS_TYPE_VIEW_FACTORY_BEGIN(NitrosEmpty)
NITROS_TYPE_VIEW_FACTORY_END(NitrosEmpty)

void NitrosEmptyView::InitView() {}  // NitrosEmpty carries no GXF components

// Minimal lifecycle node that creates both a ManagedNitrosPublisher and a
// ManagedNitrosSubscriber in on_configure().  Validates that both types work
// correctly when owned by an rclcpp_lifecycle::LifecycleNode (issue #68).
class NitrosEmptyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NitrosEmptyLifecycleNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("nitros_empty_lifecycle_node", options)
  {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    nitros_pub_ = std::make_shared<
      ManagedNitrosPublisher<NitrosEmpty>>(
      this,
      "lifecycle_pub_output",
      nitros_empty::supported_type_name);

    nitros_sub_ = std::make_shared<
      ManagedNitrosSubscriber<NitrosEmptyView>>(
      this,
      "lifecycle_sub_input",
      nitros_empty::supported_type_name);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    nitros_pub_.reset();
    nitros_sub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    nitros_pub_.reset();
    nitros_sub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  std::shared_ptr<ManagedNitrosPublisher<NitrosEmpty>> nitros_pub_;
  std::shared_ptr<ManagedNitrosSubscriber<NitrosEmptyView>> nitros_sub_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosEmptyLifecycleNode)
