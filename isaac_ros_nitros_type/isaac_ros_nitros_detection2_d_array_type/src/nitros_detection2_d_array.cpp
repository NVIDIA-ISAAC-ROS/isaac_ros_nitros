/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <string>
#include <unordered_map>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "detectnet/detection2_d_array_message.hpp"
#include "gxf/std/allocator.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_detection2_d_array_type/nitros_detection2_d_array.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

// This functions extracts data from a nvidia::isaac_ros::Detection2D object
// and returns a vision_msgs::msg::Detection2D object
vision_msgs::msg::Detection2D GetNewDetection2DMsg(
  nvidia::isaac_ros::Detection2D detection2_d,
  nvidia::gxf::Timestamp detection2_d_timestamp)
{
  vision_msgs::msg::Detection2D detection_msg;
  detection_msg.header.stamp.sec = static_cast<int32_t>(
    detection2_d_timestamp.acqtime / static_cast<uint64_t>(1e9));
  detection_msg.header.stamp.nanosec = static_cast<uint32_t>(
    detection2_d_timestamp.acqtime % static_cast<uint64_t>(1e9));

  detection_msg.bbox.center.position.x = detection2_d.center_x;
  detection_msg.bbox.center.position.y = detection2_d.center_y;
  detection_msg.bbox.center.theta = 0;
  detection_msg.bbox.size_x = detection2_d.size_x;
  detection_msg.bbox.size_y = detection2_d.size_y;
  std::vector<vision_msgs::msg::ObjectHypothesisWithPose> hypothesis_list;
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  for (size_t i = 0; i < detection2_d.results.size(); i++) {
    hypothesis.hypothesis.class_id = detection2_d.results[i].class_id;
    hypothesis.hypothesis.score = detection2_d.results[i].score;
    hypothesis_list.push_back(hypothesis);
  }
  detection_msg.results = hypothesis_list;
  return detection_msg;
}


void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosDetection2DArray,
  vision_msgs::msg::Detection2DArray>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosDetection2DArray::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);
  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDetection2DArray"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);
  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  // Extract gxf message data to a struct type defined in detection2_d_array_message.hpp
  auto detection2_d_parts_expected = nvidia::isaac_ros::GetDetection2DList(
    msg_entity.value());
  if (!detection2_d_parts_expected) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get detection2_d_array data from message entity: " <<
      GxfResultStr(detection2_d_parts_expected.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDetection2DArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto detection2_d_parts = detection2_d_parts_expected.value();

  // Extract detection2_d array to a struct type defined in detection2_d.hpp
  std::vector<nvidia::isaac_ros::Detection2D> detection2_d_array =
    *(detection2_d_parts.detection2_d_array);

  // Set timestamp for ros message from gxf message
  nvidia::gxf::Timestamp detection2_d_timestamp = *(detection2_d_parts.timestamp);
  destination.header.stamp.sec = static_cast<int32_t>(
    detection2_d_timestamp.acqtime / static_cast<uint64_t>(1e9));
  destination.header.stamp.nanosec = static_cast<uint32_t>(
    detection2_d_timestamp.acqtime % static_cast<uint64_t>(1e9));

  // Extract number of detections
  size_t num_bboxes = detection2_d_array.size();
  // resize output array to the number of tags detected before populating
  destination.detections.resize(num_bboxes);

  // Finally populate the ros message with detections
  std::vector<vision_msgs::msg::Detection2D> detections_list;
  for (size_t i = 0; i < num_bboxes; i++) {
    vision_msgs::msg::Detection2D msg_detection = GetNewDetection2DMsg(
      detection2_d_array[i],
      detection2_d_timestamp);

    detections_list.push_back(msg_detection);
  }
  destination.detections = detections_list;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosDetection2DArray"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosDetection2DArray,
  vision_msgs::msg::Detection2DArray>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosDetection2DArray::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();

  // Get pointer to allocator component
  const std::string kEntityName = "memory_pool";
  const std::string kComponentName = "unbounded_allocator";
  const std::string kComponentTypeName = "nvidia::gxf::UnboundedAllocator";

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
      rclcpp::get_logger("NitrosDetection2DArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  auto message = nvidia::gxf::Entity::New(context);
  if (!message) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Error initializing new message entity: " <<
      GxfResultStr(message.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDetection2DArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Extract timestamp from ros message and convert to gxf timestamp format
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;

  // count number of tags first to know how many times to run the below for loop
  size_t num_bboxes = source.detections.size();
  // Extract detections from the ros message into std::vector<nvidia::isaac_ros::Detection2D>
  std::vector<nvidia::isaac_ros::Detection2D> detection_info_vector;
  for (size_t i = 0; i < num_bboxes; i++) {
    nvidia::isaac_ros::Detection2D temp_detection_message;
    vision_msgs::msg::Detection2D detection2_d = source.detections[i];
    temp_detection_message.center_x = detection2_d.bbox.center.position.x;
    temp_detection_message.center_y = detection2_d.bbox.center.position.y;
    temp_detection_message.size_x = detection2_d.bbox.size_x;
    temp_detection_message.size_y = detection2_d.bbox.size_y;
    for (size_t j = 0; j < detection2_d.results.size(); j++) {
      nvidia::isaac_ros::Hypothesis result;
      result.class_id = detection2_d.results[j].hypothesis.class_id;
      result.score = detection2_d.results[j].hypothesis.score;
      temp_detection_message.results.push_back(result);
    }
    detection_info_vector.push_back(temp_detection_message);
  }

  // Finally populate the gxf message with detections and the timestamp
  auto create_detection2_d_list_message_result =
    nvidia::isaac_ros::CreateDetection2DList(context)
    .map(
    [&](nvidia::isaac_ros::Detection2DParts message_parts) -> nvidia::gxf::Expected<void> {
      nvidia::gxf::Expected<void> result;
      for (uint32_t i = 0; i < num_bboxes; i++) {
        (message_parts.detection2_d_array)->push_back(detection_info_vector[i]);
      }
      message_parts.timestamp->acqtime = input_timestamp;
      message = message_parts.message;
      return result;
    });
  if (!create_detection2_d_list_message_result) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to create the detection2_d list message: " <<
      GxfResultStr(create_detection2_d_list_message_result.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosDetection2DArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Set Entity Id
  destination.handle = message->eid();
  GxfEntityRefCountInc(context, message->eid());
  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
