/**
 * Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#include <cuda_runtime.h>

#include <string>
#include <unordered_map>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "engine/core/tensor/tensor.hpp"
#include "pose3d_cov_message.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_pose_cov_stamped_type/nitros_pose_cov_stamped.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr char kEntityName[] = "memory_pool";
constexpr char kComponentName[] = "unbounded_allocator";
constexpr char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";
constexpr int kCovarianceMatrixSize = 6;
constexpr int kCovarianceArrSize = 36;
}  // namespace

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosPoseCovStamped,
  geometry_msgs::msg::PoseWithCovarianceStamped>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosPoseCovStamped::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPoseCovStamped"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto maybe_pose_cov_stamped_parts = nvidia::isaac::GetPose3dCovMessage(
    msg_entity.value());
  if (!maybe_pose_cov_stamped_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get pose_3d_cov message"
      " from message entity: " <<
      GxfResultStr(maybe_pose_cov_stamped_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPoseCovStamped"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_cov_stamped_parts = maybe_pose_cov_stamped_parts.value();
  destination.pose.pose.position.x = pose_cov_stamped_parts.pose->translation.x();
  destination.pose.pose.position.y = pose_cov_stamped_parts.pose->translation.y();
  destination.pose.pose.position.z = pose_cov_stamped_parts.pose->translation.z();
  destination.pose.pose.orientation.x = pose_cov_stamped_parts.pose->rotation.quaternion().x();
  destination.pose.pose.orientation.y = pose_cov_stamped_parts.pose->rotation.quaternion().y();
  destination.pose.pose.orientation.z = pose_cov_stamped_parts.pose->rotation.quaternion().z();
  destination.pose.pose.orientation.w = pose_cov_stamped_parts.pose->rotation.quaternion().w();

  auto pose3d_cov_tensor_view = pose_cov_stamped_parts.covariance;
  auto pose3d_cov_dimensions = pose3d_cov_tensor_view.dimensions();
  if (pose3d_cov_dimensions[0] != kCovarianceMatrixSize ||
    pose3d_cov_dimensions[1] != kCovarianceMatrixSize)
  {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Tensor dimensions is not equal to " << kCovarianceMatrixSize;
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPoseCovStamped"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  for (int i = 0; i < pose3d_cov_dimensions[0]; i++) {
    for (int j = 0; j < pose3d_cov_dimensions[1]; j++) {
      destination.pose.covariance[i + kCovarianceMatrixSize * j] =
        pose3d_cov_tensor_view(i, j);
    }
  }

  // Populate timestamp information back into ROS header
  auto input_timestamp = pose_cov_stamped_parts.timestamp;
  if (input_timestamp) {
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp->acqtime % static_cast<uint64_t>(1e9));
  }

  // Set frame ID
  destination.header.frame_id = source.frame_id;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPoseCovStamped"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosPoseCovStamped,
  geometry_msgs::msg::PoseWithCovarianceStamped>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosPoseCovStamped::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPoseCovStamped"),
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
      rclcpp::get_logger("NitrosPoseCovStamped"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  // create Pose3dCovMessage
  auto maybe_pose_cov_stamped_parts = nvidia::isaac::CreatePose3dCovMessage(
    context, allocator_handle);
  if (!maybe_pose_cov_stamped_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_gxf_message] Failed to create Pose3dCovMessage " << GxfResultStr(
      maybe_pose_cov_stamped_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPoseCovStamped"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_cov_stamped_parts = maybe_pose_cov_stamped_parts.value();
  // populate pose data
  *(pose_cov_stamped_parts.pose) = ::isaac::Pose3d{
    ::isaac::SO3d::FromQuaternion(
      ::isaac::Quaterniond{
      source.pose.pose.orientation.w,
      source.pose.pose.orientation.x,
      source.pose.pose.orientation.y,
      source.pose.pose.orientation.z}),
    ::isaac::Vector3d(
      source.pose.pose.position.x,
      source.pose.pose.position.y,
      source.pose.pose.position.z)};

  // populate pose covariance data
  auto pose3d_cov_tensor_view = pose_cov_stamped_parts.covariance;
  auto pose3d_cov_dimensions = pose3d_cov_tensor_view.dimensions();
  if (pose3d_cov_dimensions[0] != kCovarianceMatrixSize ||
    pose3d_cov_dimensions[1] != kCovarianceMatrixSize)
  {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_gxf_message] Pose3dCovMessage 2D array expected shape is" <<
      kCovarianceMatrixSize << "x" << kCovarianceMatrixSize <<
      ", but actual shape is" << pose3d_cov_dimensions[0] << "x" << pose3d_cov_dimensions[1];
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPoseCovStamped"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  if (source.pose.covariance.size() != kCovarianceArrSize) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_gxf_message] Expected PoseWithCovarianceStamped covariance array length is " <<
      kCovarianceArrSize << ", but actual length is" << source.pose.covariance.size();
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPoseCovStamped"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  for (int i = 0; i < pose3d_cov_dimensions[0]; i++) {
    for (int j = 0; j < pose3d_cov_dimensions[1]; j++) {
      pose3d_cov_tensor_view(i, j) =
        static_cast<double>(source.pose.covariance[i + kCovarianceMatrixSize * j]);
    }
  }

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  pose_cov_stamped_parts.timestamp->acqtime = input_timestamp;

  // Set frame ID
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = pose_cov_stamped_parts.entity.eid();
  GxfEntityRefCountInc(context, pose_cov_stamped_parts.entity.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPoseCovStamped"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
