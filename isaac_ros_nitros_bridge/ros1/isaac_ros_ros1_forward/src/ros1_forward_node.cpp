// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ros/ros.h"
#include "ros1_forward_node.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace ros1_forward
{

void ROS1ForwardNode::onInit()
{
  ros::NodeHandle & private_nh = getPrivateNodeHandle();

  // Create subscriber topics
  sub_ = private_nh.subscribe("forward_input", 10, &ROS1ForwardNode::ForwardCallback, this);
}

void ROS1ForwardNode::ForwardCallback(const topic_tools::ShapeShifter::ConstPtr & input_msg)
{
  ROS_DEBUG("Received message in ForwardCallback.");
  if (input_msg->getMD5Sum() != input_md5_sum_) {
    ros::NodeHandle & private_nh = getPrivateNodeHandle();
    // Create publisher topics
    ros::AdvertiseOptions opts("forward_output", 10, input_msg->getMD5Sum(),
      input_msg->getDataType(), input_msg->getMessageDefinition());
    pub_ = private_nh.advertise(opts);
  }

  pub_.publish(input_msg);
}

}  // namespace ros1_forward
}  // namespace isaac_ros
}  // namespace nvidia

PLUGINLIB_EXPORT_CLASS(nvidia::isaac_ros::ros1_forward::ROS1ForwardNode, nodelet::Nodelet);
