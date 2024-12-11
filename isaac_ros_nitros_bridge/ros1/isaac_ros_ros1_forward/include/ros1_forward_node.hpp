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

#ifndef ISAAC_ROS_ROS1_Forward__ROS1_FORWARD_NODE_HPP_
#define ISAAC_ROS_ROS1_Forward__ROS1_FORWARD_NODE_HPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ros/ros.h"
#include "topic_tools/shape_shifter.h"


namespace nvidia
{
namespace isaac_ros
{
namespace ros1_forward
{

class ROS1ForwardNode : public nodelet::Nodelet
{
public:
  ROS1ForwardNode() {}

private:
  virtual void onInit() override;

  // Convert bridge message into ros message and copy back to CPU
  void ForwardCallback(const topic_tools::ShapeShifter::ConstPtr & msg);
  std::string input_md5_sum_;

  ros::Publisher pub_;
  ros::Subscriber sub_;
};

}  // namespace ros1_forward
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_ROS1_Forward__ROS1_FORWARD_NODE_HPP_
