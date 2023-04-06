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
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/multimedia/camera.hpp"
#include "gxf/std/timestamp.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"
#include "extensions/fiducials/gems/fiducial_info.hpp"
#include "extensions/fiducials/messages/fiducial_message.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_april_tag_detection_array_type/nitros_april_tag_detection_array.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"


// Conversion from
::isaac::Pose3d DetectionToPose3d(const float translation[3], const float rotation[4])
{
  return ::isaac::Pose3d{
    ::isaac::SO3d::FromQuaternion(
      ::isaac::Quaterniond{rotation[0], rotation[1], rotation[2],
        rotation[3]}),
    ::isaac::Vector3d(translation[0], translation[1], translation[2])
  };
}

// Gets corner points as a tensor
nvidia::gxf::Expected<nvidia::gxf::Tensor> CornersToTensor(
  float corners[8],
  nvidia::gxf::Handle<nvidia::gxf::Allocator> allocator)
{
  nvidia::gxf::Tensor tensor;
  return tensor.reshape<double>(
    nvidia::gxf::Shape{4, 2}, nvidia::gxf::MemoryStorageType::kHost,
    allocator)
         .and_then([&]() {return tensor.data<double>();})
         .map(
    [&](double * points) {
      const int stride = tensor.shape().dimension(0);
      for (int i = 0; i < stride; i++) {
        points[i] = corners[2 * i];  // y
        points[i + stride] = corners[2 * i + 1];
      }
      return std::move(tensor);
    });
}

void ExtractFamilyId(std::string family_id, std::string & family_, int & id_)
{
  size_t i = 0;
  bool family_read_done = false;
  std::string family;
  std::string id;

  // For AprilTag, the id is of the format <TagFamily_ID>
  // Ex. If the decoded tag ID is 14 and belongs to TagFamily tag36h11, the id is tag36h11_14
  // This struct in defined in fiducial_info.hpp
  while (i < family_id.size()) {
    if (family_id[i] == '_') {family_read_done = true;} else if (family_read_done) {
      id.push_back(family_id[i]);
    } else {family.push_back(family_id[i]);}
    i++;
  }
  family_ = family;
  id_ = stoi(id);  // convert to int
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosAprilTagDetectionArray,
  isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosAprilTagDetectionArray::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosAprilTagDetectionArray"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  // Populate timestamp information back into ROS header
  auto input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>("timestamp");
  if (!input_timestamp) {    // Fallback to label 'timestamp'
    input_timestamp = msg_entity->get<nvidia::gxf::Timestamp>();
  }
  if (input_timestamp) {
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp.value()->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp.value()->acqtime % static_cast<uint64_t>(1e9));
  }

  //  Extract apriltags array to a struct type defined in fiducial_message.hpp
  auto apriltags_detections_array_expected = nvidia::isaac::GetFiducialListMessage(
    msg_entity.value());
  if (!apriltags_detections_array_expected) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get apriltags detections data from message entity: " <<
      GxfResultStr(apriltags_detections_array_expected.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosAprilTagDetectionArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto apriltags_detections_array = apriltags_detections_array_expected.value();

  // Extract number of tags detected
  size_t tags_count = apriltags_detections_array.count;

  // resize output array to the number of tags detected before populating
  destination.detections.resize(tags_count);

  for (size_t i = 0; i < tags_count; i++) {
    // struct is defined in fiducial_message.hpp
    auto info = apriltags_detections_array.info.at(i).value();
    auto pose = apriltags_detections_array.pose.at(i).value();
    auto keypoints = apriltags_detections_array.keypoints.at(i).value();

    isaac_ros_apriltag_interfaces::msg::AprilTagDetection msg_detection;

    std::string tag_family_id = info->id;
    // extracting family and id from tag info
    ExtractFamilyId(tag_family_id, msg_detection.family, msg_detection.id);

    // poulate tag pose data
    msg_detection.pose.pose.pose.position.x = pose->translation.x();
    msg_detection.pose.pose.pose.position.y = pose->translation.y();
    msg_detection.pose.pose.pose.position.z = pose->translation.z();
    msg_detection.pose.pose.pose.orientation.x = pose->rotation.quaternion().x();
    msg_detection.pose.pose.pose.orientation.y = pose->rotation.quaternion().y();
    msg_detection.pose.pose.pose.orientation.z = pose->rotation.quaternion().z();
    msg_detection.pose.pose.pose.orientation.w = pose->rotation.quaternion().w();

    // populate center and corners
    nvidia::gxf::Handle<nvidia::gxf::Tensor> corners_tensor_handle = keypoints;
    auto corners_tensor_data = corners_tensor_handle->data<double>().value();
    const int stride = corners_tensor_handle->shape().dimension(0);

    for (int j = 0; j < stride; j++) {
      msg_detection.corners[j].y = corners_tensor_data[j];
      msg_detection.corners[j].x = corners_tensor_data[j + stride];
    }
    msg_detection.center.x = (msg_detection.corners[0].x + msg_detection.corners[1].x) / 2;
    msg_detection.center.y = (msg_detection.corners[1].y + msg_detection.corners[2].y) / 2;
    destination.detections[i] = msg_detection;
  }

  // Set frame ID
  destination.header.frame_id = source.frame_id;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosAprilTagDetectionArray"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosAprilTagDetectionArray,
  isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosAprilTagDetectionArray::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosAprilTagDetectionArray"),
    "[convert_to_custom] Conversion started");

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
      rclcpp::get_logger("NitrosAprilTagDetectionArray"), error_msg.str().c_str());
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
      rclcpp::get_logger("NitrosAprilTagDetectionArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  auto output_timestamp = message->add<nvidia::gxf::Timestamp>("timestamp");
  if (!output_timestamp) {
    std::stringstream error_msg;
    error_msg << "[convert_to_custom] Failed to add a timestamp component to message: " <<
      GxfResultStr(output_timestamp.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosAprilTagDetectionArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  output_timestamp.value()->acqtime = input_timestamp;

  // count number of tags first to know how many times to run the below for loop
  size_t tags_count = static_cast<size_t>(source.detections.size());
  auto create_fiducial_list_message_result =
    nvidia::isaac::CreateFiducialListMessage(context, tags_count)
    .map(
    [&](nvidia::isaac::FiducialListMessageParts apriltags_list) -> nvidia::gxf::Expected<void> {
      nvidia::gxf::Expected<void> result;
      for (size_t i = 0; i < tags_count; i++) {
        apriltags_list.info[i].value()->type = nvidia::isaac::FiducialInfo::Type::kAprilTag;
        apriltags_list.info[i].value()->id = source.detections[i].family + "_" +
        std::to_string(source.detections[i].id);

        // Create float instead of directly from ros msg to not change the original CornersToTensor
        float corners_[8];
        for (int j = 0; j < 4; j++) {
          corners_[2 * j] = source.detections[i].corners[j].y;
          corners_[2 * j + 1] = source.detections[i].corners[j].x;
        }
        // add corners from ros msg here
        auto keypoints = CornersToTensor(corners_, allocator_handle);
        if (!keypoints) {
          return nvidia::gxf::ForwardError(keypoints);
        }
        *apriltags_list.keypoints[i].value() = std::move(keypoints.value());

        const float translation[3] = {
          static_cast<float>(source.detections[i].pose.pose.pose.position.x),
          static_cast<float>(source.detections[i].pose.pose.pose.position.y),
          static_cast<float>(source.detections[i].pose.pose.pose.position.z)
        };
        const float rotation[4] = {
          static_cast<float>(source.detections[i].pose.pose.pose.orientation.w),
          static_cast<float>(source.detections[i].pose.pose.pose.orientation.x),
          static_cast<float>(source.detections[i].pose.pose.pose.orientation.y),
          static_cast<float>(source.detections[i].pose.pose.pose.orientation.z)
        };
        // add pose from ros msg here
        *apriltags_list.pose[i].value() = DetectionToPose3d(translation, rotation);
      }
      message = apriltags_list.entity;
      return result;
    });
  if (!create_fiducial_list_message_result) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to complete the fiducial list message: " <<
      GxfResultStr(create_fiducial_list_message_result.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosAprilTagDetectionArray"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Set frame ID
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = message->eid();
  GxfEntityRefCountInc(context, message->eid());

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
