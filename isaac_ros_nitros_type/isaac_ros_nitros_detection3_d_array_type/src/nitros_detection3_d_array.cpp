// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <string>
#include <unordered_map>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"
#include "gxf/std/allocator.hpp"
#include "detection3_d_array_message/detection3_d_array_message.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_detection3_d_array_type/nitros_detection3_d_array.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

constexpr char kEntityName[] = "memory_pool";
constexpr char kComponentName[] = "unbounded_allocator";
constexpr char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";

namespace
{

::nvidia::isaac::Pose3d DetectionToPose3d(const float translation[3], const float rotation_wxyz[4])
{
  return ::nvidia::isaac::Pose3d{
    ::nvidia::isaac::SO3d::FromQuaternion(
      ::nvidia::isaac::Quaterniond{rotation_wxyz[0], rotation_wxyz[1], rotation_wxyz[2],
        rotation_wxyz[3]}),
    ::nvidia::isaac::Vector3d(translation[0], translation[1], translation[2])
  };
}
}  // namespace

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosDetection3DArray,
  vision_msgs::msg::Detection3DArray>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosDetection3DArray::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDetection3DArray"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);
  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto maybe_msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);
  if (!maybe_msg_entity) {
    throw std::runtime_error("Unable to get entity!");
  }

  auto maybe_detection3d_list = nvidia::isaac::GetDetection3DListMessage(maybe_msg_entity.value());
  if (!maybe_detection3d_list) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] failed to get detection3d data from message entity: " <<
      GxfResultStr(maybe_detection3d_list.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDetection3DArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto detection3_d_parts = maybe_detection3d_list.value();

  // Set timestamp for ros message from gxf message
  nvidia::gxf::Timestamp detection3_d_timestamp = *(detection3_d_parts.timestamp);
  destination.header.stamp.sec = static_cast<int32_t>(
    detection3_d_timestamp.acqtime / static_cast<uint64_t>(1e9));
  destination.header.stamp.nanosec = static_cast<uint32_t>(
    detection3_d_timestamp.acqtime % static_cast<uint64_t>(1e9));
  destination.header.frame_id = source.frame_id;

  size_t n_detections = detection3_d_parts.poses.size();

  for (size_t i = 0; i < n_detections; ++i) {
    vision_msgs::msg::Detection3D detection;
    detection.header.frame_id = source.frame_id;
    detection.header.stamp = destination.header.stamp;
    vision_msgs::msg::BoundingBox3D bbox;

    bbox.center.position.x = detection3_d_parts.poses.at(i).value()->translation.x();
    bbox.center.position.y = detection3_d_parts.poses.at(i).value()->translation.y();
    bbox.center.position.z = detection3_d_parts.poses.at(i).value()->translation.z();

    bbox.center.orientation.x = detection3_d_parts.poses.at(i).value()->rotation.quaternion().x();
    bbox.center.orientation.y = detection3_d_parts.poses.at(i).value()->rotation.quaternion().y();
    bbox.center.orientation.z = detection3_d_parts.poses.at(i).value()->rotation.quaternion().z();
    bbox.center.orientation.w = detection3_d_parts.poses.at(i).value()->rotation.quaternion().w();

    bbox.size.x = detection3_d_parts.bbox_sizes.at(i).value()->x();
    bbox.size.y = detection3_d_parts.bbox_sizes.at(i).value()->y();
    bbox.size.z = detection3_d_parts.bbox_sizes.at(i).value()->z();
    detection.bbox = bbox;

    for (size_t j = 0; j < detection3_d_parts.hypothesis.at(i).value()->scores.size(); ++j) {
      vision_msgs::msg::ObjectHypothesisWithPose object_hypothesis;
      object_hypothesis.hypothesis.class_id =
        detection3_d_parts.hypothesis.at(i).value()->class_ids[j];
      object_hypothesis.hypothesis.score = detection3_d_parts.hypothesis.at(i).value()->scores[j];
      detection.results.push_back(object_hypothesis);
    }

    destination.detections.push_back(detection);
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDetection3DArray"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosDetection3DArray,
  vision_msgs::msg::Detection3DArray>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosDetection3DArray::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDetection3DArray"),
    "[convert_to_custom] Conversion started");

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  // Get pointer to allocator component
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kEntityName, kComponentName, kComponentTypeName, cid);

  auto maybe_allocator_handle =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context, cid);
  if (!maybe_allocator_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get allocator's handle: " <<
      GxfResultStr(maybe_allocator_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDetection3DArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  // Extract timestamp from ros message and convert to gxf timestamp format
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;

  // count number of tags first to know how many times to run the below for loop
  size_t num_detections = source.detections.size();

  nvidia::gxf::Entity message;
  auto detection3_d_result = nvidia::isaac::CreateDetection3DListMessage(context, num_detections)
    .map(
    [&](nvidia::isaac::Detection3DListMessageParts message_parts) {
      for (size_t i = 0; i < num_detections; ++i) {
        vision_msgs::msg::Detection3D detection = source.detections[i];
        message_parts.bbox_sizes[i].value()->x() = detection.bbox.size.x;
        message_parts.bbox_sizes[i].value()->y() = detection.bbox.size.y;
        message_parts.bbox_sizes[i].value()->z() = detection.bbox.size.z;

        const float translation[] = {
          static_cast<float>(detection.bbox.center.position.x),
          static_cast<float>(detection.bbox.center.position.y),
          static_cast<float>(detection.bbox.center.position.z)
        };

        // Follow Eigen convention
        const float rotation[] = {
          static_cast<float>(detection.bbox.center.orientation.w),
          static_cast<float>(detection.bbox.center.orientation.x),
          static_cast<float>(detection.bbox.center.orientation.y),
          static_cast<float>(detection.bbox.center.orientation.z)
        };

        *message_parts.poses[i].value() = DetectionToPose3d(
          translation,
          rotation);

        for (const auto & object_hypothesis : detection.results) {
          message_parts.hypothesis[i].value()->class_ids.push_back(
            object_hypothesis.hypothesis.
            class_id);
          message_parts.hypothesis[i].value()->scores.push_back(
            object_hypothesis.hypothesis.score
          );
        }
      }
      message_parts.timestamp->acqtime = input_timestamp;
      message = message_parts.entity;
    });

  if (!detection3_d_result) {
    std::stringstream error_msg;
    error_msg << "[convert_to_custom] Failed to transfer message to custom: " <<
      GxfResultStr(detection3_d_result.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDetection3DArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Set frame ID
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = message.eid();
  GxfEntityRefCountInc(context, message.eid());
  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDetection3DArray"),
    "[convert_to_custom] Conversion completed");
}
