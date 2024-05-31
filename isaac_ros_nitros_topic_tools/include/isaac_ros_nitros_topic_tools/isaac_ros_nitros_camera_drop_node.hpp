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

#ifndef ISAAC_ROS_NITROS_TOPIC_TOOLS__ISAAC_ROS_NITROS_CAMERA_DROP_NODE_HPP_
#define ISAAC_ROS_NITROS_TOPIC_TOOLS__ISAAC_ROS_NITROS_CAMERA_DROP_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "isaac_ros_managed_nitros/managed_nitros_message_filters_subscriber.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_managed_nitros/managed_nitros_subscriber.hpp"
#include "isaac_ros_nitros_camera_info_type/nitros_camera_info.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_view.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{
/**
 * @brief NitrosCameraDropNode class implements a node that
 *        synchronizes and drops X out of Y incoming messages.
 *        The node supports three modes:
 *        - Mode 0(mono): Camera + CameraInfo
 *        - Mode 1(stereo): Camera + CameraInfo + Camera + CameraInfo
 *        - Mode 2(mono+depth): Camera + CameraInfo + Depth
 *        The node is based on the ros-tooling/topic_tools::DropNode OSS class.
 *        But the node is modified to drop messages in an evenly spread out fashion.
 *        The node does this by creating a binary array that represents the drop pattern.
 *        The node then uses the binary array to determine if the current message should be
 *        dropped or not.
 */
class NitrosCameraDropNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for NitrosCameraDropNode class.
   * @param options The node options.
   */
  explicit NitrosCameraDropNode(const rclcpp::NodeOptions & options);

private:
  // Subscribers
  nvidia::isaac_ros::nitros::message_filters::Subscriber<nvidia::isaac_ros::nitros::NitrosImageView>
  image_sub_1_;
  ::message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_1_;
  nvidia::isaac_ros::nitros::message_filters::Subscriber<nvidia::isaac_ros::nitros::NitrosImageView>
  image_sub_2_;
  ::message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_2_;
  nvidia::isaac_ros::nitros::message_filters::Subscriber<nvidia::isaac_ros::nitros::NitrosImageView>
  depth_sub_;

  std::shared_ptr<
    nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>
  image_pub_1_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_1_;
  std::shared_ptr<
    nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>
  image_pub_2_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_2_;
  std::shared_ptr<
    nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>
  depth_pub_;
  // Exact message sync policy
  using ExactPolicyMode0 = ::message_filters::sync_policies::ExactTime<
    nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::CameraInfo>;
  using ExactSyncMode0 = ::message_filters::Synchronizer<ExactPolicyMode0>;
  std::shared_ptr<ExactSyncMode0> exact_sync_mode_0_;  // Exact sync mode 0

  using ExactPolicyMode1 = ::message_filters::sync_policies::ExactTime<
    nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::CameraInfo,
    nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::CameraInfo>;
  using ExactSyncMode1 = ::message_filters::Synchronizer<ExactPolicyMode1>;
  std::shared_ptr<ExactSyncMode1> exact_sync_mode_1_;  // Exact sync mode 1

  using ExactPolicyMode2 = ::message_filters::sync_policies::ExactTime<
    nvidia::isaac_ros::nitros::NitrosImage, sensor_msgs::msg::CameraInfo,
    nvidia::isaac_ros::nitros::NitrosImage>;
  using ExactSyncMode2 = ::message_filters::Synchronizer<ExactPolicyMode2>;
  std::shared_ptr<ExactSyncMode2> exact_sync_mode_2_;  // Exact sync mode 2

  /**
   * @brief Callback function for sync mode 0.
   * @param image_ptr Pointer to the image message.
   * @param camera_info_ptr Pointer to the camera info message.
   */
  void sync_callback_mode_0(
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & image_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_ptr);

  /**
   * @brief Callback function for sync mode 1.
   * @param image_1_ptr Pointer to the first image message.
   * @param camera_info_1_ptr Pointer to the first camera info message.
   * @param image_2_ptr Pointer to the second image message.
   * @param camera_info_2_ptr Pointer to the second camera info message.
   */
  void sync_callback_mode_1(
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & image_1_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_1_ptr,
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & image_2_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_2_ptr);

  /**
   * @brief Callback function for sync mode 2.
   * @param image_ptr Pointer to the image message.
   * @param camera_info_ptr Pointer to the camera info message.
   * @param depth_ptr Pointer to the depth message.
   */
  void sync_callback_mode_2(
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & image_ptr,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_ptr,
    const nvidia::isaac_ros::nitros::NitrosImage::ConstSharedPtr & depth_ptr);

  /**
   * @brief Ticks the dropper to determine if the current message should be
   * published.
   *
   * This method iterates through the dropping order array and returns true if the current message
   * should be published.
   *
   * @return True if the message should be published; false otherwise.
   */
  bool tick_dropper();

  /**
   * @brief Create an evenly spread array of size array_size with num_ones ones.
   * @param array_size The size of the array.
   * @param num_ones The number of ones in the array.
   * @return The evenly spread array.
   */
  std::vector<bool> create_evenly_spread_array(int array_size, int num_ones);

  std::string mode_;                      // Input mode
  std::string depth_format_string_;       // Depth format string
  int input_queue_size_;                  // Input queue size
  int output_queue_size_;                 // Output queue size
  int sync_queue_size_;                   // Sync queue size
  int x_;                                 // Drop X out of Y messages
  int y_;                                 // Drop X out of Y messages
  int count_;                             // Count of messages
  std::vector<bool> dropping_order_arr_;  // Dropping order array
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_TOPIC_TOOLS__ISAAC_ROS_NITROS_CAMERA_DROP_NODE_HPP_
