// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros_topic_tools/isaac_ros_nitros_topic_tools_common.hpp"
#include "isaac_ros_nitros_topic_tools/isaac_ros_nitros_camera_drop_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include <isaac_ros_common/qos.hpp>

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

namespace
{
constexpr const char kDefaultQoS[] = "DEFAULT";
}

NitrosCameraDropNode::NitrosCameraDropNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("drop", options)
{
  mode_ = declare_parameter<std::string>("mode", modeToStringMap.at(CameraDropMode::Mono));
  depth_format_string_ =
    declare_parameter<std::string>("depth_format_string", "nitros_image_32FC1");
  sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
  input_queue_size_ = declare_parameter<int>("input_queue_size", 10);
  output_queue_size_ = declare_parameter<int>("output_queue_size", 10);
  const rclcpp::QoS input_qos = ::isaac_ros::common::AddQosParameter(
    *this, kDefaultQoS, "input_qos").keep_last(input_queue_size_);
  const rclcpp::QoS output_qos = ::isaac_ros::common::AddQosParameter(
    *this, kDefaultQoS, "output_qos").keep_last(output_queue_size_);
  const rmw_qos_profile_t input_qos_profile = input_qos.get_rmw_qos_profile();

  // Drop X out of Y messages
  x_ = declare_parameter<int>("X", 15);
  y_ = declare_parameter<int>("Y", 30);
  if (x_ > y_) {
    RCLCPP_ERROR(get_logger(), "X cannot be greater than Y");
    return;
  }
  count_ = 0;

  // Create an array that represents the dropping pattern
  // The array contains even spread of 0s.
  dropping_order_arr_ = create_evenly_spread_array(y_, x_);

  // Initialize common subscribers and publishers for all modes
  image_sub_1_.subscribe(this, "image_1", input_qos_profile);
  camera_info_sub_1_.subscribe(this, "camera_info_1", input_qos_profile);
  image_pub_1_ = std::make_shared<
    nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>(
    this, "image_1_drop",
    nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(), output_qos);
  camera_info_pub_1_ = this->create_publisher<
    sensor_msgs::msg::CameraInfo>("camera_info_1_drop", output_qos);

  if (mode_ == modeToStringMap.at(CameraDropMode::Mono)) {
    // Mode 0: Camera + CameraInfo (mono)
    // Initialize sync policy
    exact_sync_mode_0_ = std::make_shared<ExactSyncMode0>(
      ExactPolicyMode0(sync_queue_size_), image_sub_1_,
      camera_info_sub_1_);  // Use GetDefaultCompatibleFormat
    using namespace std::placeholders;
    exact_sync_mode_0_->registerCallback(
      std::bind(&NitrosCameraDropNode::sync_callback_mode_0, this, _1, _2));
  } else if (mode_ == modeToStringMap.at(CameraDropMode::Stereo)) {
    // Mode 1: Camera + CameraInfo + Camera + CameraInfo (stereo)
    // Initialize second mono subscribers and publishers
    image_sub_2_.subscribe(this, "image_2", input_qos_profile);
    camera_info_sub_2_.subscribe(this, "camera_info_2", input_qos_profile);
    image_pub_2_ = std::make_shared<
      nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>(
      this, "image_2_drop",
      nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
      nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(), output_qos);
    camera_info_pub_2_ = this->create_publisher<
      sensor_msgs::msg::CameraInfo>("camera_info_2_drop", output_qos);
    // Initialize sync policy
    exact_sync_mode_1_ = std::make_shared<ExactSyncMode1>(
      ExactPolicyMode1(sync_queue_size_), image_sub_1_, camera_info_sub_1_, image_sub_2_,
      camera_info_sub_2_);
    using namespace std::placeholders;
    exact_sync_mode_1_->registerCallback(
      std::bind(&NitrosCameraDropNode::sync_callback_mode_1, this, _1, _2, _3, _4));
  } else if (mode_ == modeToStringMap.at(CameraDropMode::MonoDepth)) {
    // Mode 2: Camera + CameraInfo + Depth (mono+depth)
    // Initialize depth subscriber and publisher
    depth_sub_.subscribe(
      this, "depth_1", input_qos_profile, rclcpp::SubscriptionOptions(),
      depth_format_string_);
    depth_pub_ = std::make_shared<
      nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>(
      this, "depth_1_drop", depth_format_string_);
    // Initialize sync policy
    exact_sync_mode_2_ = std::make_shared<ExactSyncMode2>(
      ExactPolicyMode2(sync_queue_size_), image_sub_1_, camera_info_sub_1_, depth_sub_);
    using namespace std::placeholders;
    exact_sync_mode_2_->registerCallback(
      std::bind(&NitrosCameraDropNode::sync_callback_mode_2, this, _1, _2, _3));
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid mode: %s", mode_.c_str());
  }
}

std::vector<bool> NitrosCameraDropNode::create_evenly_spread_array(int array_size, int num_zeroes)
{
  std::vector<bool> dropping_order_arr(array_size, true);  // Initialize an array with all ones
  if (num_zeroes <= 0) {return dropping_order_arr;}
  double spacing = static_cast<double>(array_size) / num_zeroes;  // Calculate spacing
  // Start by dropping the first message, ie, i=0
  for (int i = 0; i < array_size; i++) {
    int index = static_cast<int>(i * spacing);
    if (index < array_size) {  // Ensure index does not exceed the array size
      dropping_order_arr[index] = false;  // Place a 0 at the calculated index
    }
  }
  return dropping_order_arr;
}

bool NitrosCameraDropNode::tick_dropper()
{
  bool publish = dropping_order_arr_[count_];
  count_++;
  if (count_ >= y_) {
    count_ = 0;
  }
  return publish;
}

void NitrosCameraDropNode::sync_callback_mode_0(
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & image_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_ptr)
{
  if (tick_dropper()) {
    image_pub_1_->publish(*image_ptr);
    camera_info_pub_1_->publish(*camera_info_ptr);
  }
}

void NitrosCameraDropNode::sync_callback_mode_1(
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & image_1_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_1_ptr,
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & image_2_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_2_ptr)
{
  if (tick_dropper()) {
    image_pub_1_->publish(*image_1_ptr);
    camera_info_pub_1_->publish(*camera_info_1_ptr);
    image_pub_2_->publish(*image_2_ptr);
    camera_info_pub_2_->publish(*camera_info_2_ptr);
  }
}

void NitrosCameraDropNode::sync_callback_mode_2(
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & image_ptr,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_ptr,
  const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & depth_ptr)
{
  if (tick_dropper()) {
    image_pub_1_->publish(*image_ptr);
    camera_info_pub_1_->publish(*camera_info_ptr);
    depth_pub_->publish(*depth_ptr);
  }
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros::NitrosCameraDropNode)
