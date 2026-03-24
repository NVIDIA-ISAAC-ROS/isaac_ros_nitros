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

#ifndef ISAAC_ROS_NITROS__NITROS_NODE_INTERFACES_HPP_
#define ISAAC_ROS_NITROS__NITROS_NODE_INTERFACES_HPP_

#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_clock_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

/// @brief A "node-like" aggregate of the rclcpp node interfaces required by
///        NitrosPublisher / NitrosSubscriber.
///
/// This type is constructible from any node-like object (rclcpp::Node,
/// rclcpp_lifecycle::LifecycleNode, or any type that provides the standard
/// get_node_xxx_interface() accessors).  It can be passed directly to
/// rclcpp::create_publisher<T>(), rclcpp::create_subscription<T>(),
/// rclcpp::create_wall_timer(), and to the negotiated::NegotiatedPublisher /
/// NegotiatedSubscription template constructors.
using NitrosNodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<
  rclcpp::node_interfaces::NodeBaseInterface,       // get_name(), get_namespace(),
                                                    // create_callback_group()
  rclcpp::node_interfaces::NodeClockInterface,      // get_clock()
  rclcpp::node_interfaces::NodeGraphInterface,      // required by NegotiatedPublisher
  rclcpp::node_interfaces::NodeLoggingInterface,    // get_logger()
  rclcpp::node_interfaces::NodeParametersInterface,  // required by create_publisher/subscription
  rclcpp::node_interfaces::NodeTimersInterface,     // create_wall_timer()
  rclcpp::node_interfaces::NodeTopicsInterface,     // create_publisher / create_subscription
  rclcpp::node_interfaces::NodeWaitablesInterface   // add_waitable()
>;

/// @brief Convenience factory that builds a NitrosNodeInterfaces from any node-like object.
///
/// Accepts rclcpp::Node, rclcpp_lifecycle::LifecycleNode, or any type that
/// implements the standard rclcpp node interface accessors.
template<typename NodeT>
inline NitrosNodeInterfaces MakeNitrosNodeInterfaces(NodeT & node)
{
  return NitrosNodeInterfaces(node);
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS__NITROS_NODE_INTERFACES_HPP_
