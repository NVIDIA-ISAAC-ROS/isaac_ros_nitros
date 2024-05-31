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

#ifndef ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_MESSAGE_FILTER_TRAITS_HPP_
#define ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_MESSAGE_FILTER_TRAITS_HPP_

#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"
#include "isaac_ros_nitros/types/type_utility.hpp"
#include "std_msgs/msg/header.hpp"

#include "message_filters/message_traits.h"
#include "message_filters/subscriber.h"

// Nitros type support for message_filters package
// https://github.com/ros2/message_filters.git

namespace message_filters
{
namespace message_traits
{

template<typename M, typename = void>
struct IsNitrosType : public std::false_type {};

template<typename M>
struct IsNitrosType<M,
  typename std::enable_if<std::is_base_of<nvidia::isaac_ros::nitros::NitrosTypeBase,
  M>::value>::type>: public std::true_type {};

template<typename M>
struct TimeStamp<M, typename std::enable_if<IsNitrosType<M>::value>::type>
{
  static rclcpp::Time value(const M & m)
  {
    std_msgs::msg::Header ros_header;
    if (nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getEntityTimestamp(
        m.handle, ros_header) != GXF_SUCCESS)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("[NITROS message_filter_traits]"),
        "[message_filter] getEntityTimestamp Error");
    }
    return rclcpp::Time(ros_header.stamp, RCL_ROS_TIME);
  }
};

template<typename M>
struct FrameId<M, typename std::enable_if<IsNitrosType<M>::value>::type>
{
  static std::string * pointer(M & m) {return &m.frame_id;}
  static std::string const * pointer(const M & m) {return &m.frame_id;}
  static std::string value(const M & m) {return m.frame_id;}
};

}  // namespace message_traits

}  // namespace message_filters

#endif  // ISAAC_ROS_NITROS__TYPES__NITROS_TYPE_MESSAGE_FILTER_TRAITS_HPP_
