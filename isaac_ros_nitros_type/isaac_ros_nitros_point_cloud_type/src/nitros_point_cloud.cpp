/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
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
#include "point_cloud_message.hpp"
#include "gems/pose_tree/pose_tree.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"

#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr char kEntityName[] = "memory_pool";
constexpr char kComponentName[] = "unbounded_allocator";
constexpr char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";
constexpr char kPoseTreeEntityName[] = "global_pose_tree";
constexpr char kPoseTreeComponentName[] = "pose_tree";
constexpr char kPoseTreeComponentTypeName[] = "nvidia::isaac::PoseTree";
}  // namespace

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosPointCloud,
  sensor_msgs::msg::PointCloud2>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosPointCloud::convert_to_ros_message",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_ros_message] Conversion started for handle=%ld", source.handle);

  auto context = nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getContext();
  auto msg_entity = nvidia::gxf::Entity::Shared(context, source.handle);

  auto maybe_point_cloud_parts = nvidia::isaac_ros::messages::GetPointCloudMessage(
    msg_entity.value());
  if (!maybe_point_cloud_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to get pointcloud message from message entity: " <<
      GxfResultStr(maybe_point_cloud_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto point_cloud_parts = maybe_point_cloud_parts.value();
  const int32_t n_points = point_cloud_parts.points->shape().dimension(0);

  destination.height = 1;
  destination.width = n_points;
  destination.is_bigendian = point_cloud_parts.info->is_bigendian;
  destination.is_dense = false;

  sensor_msgs::PointCloud2Modifier pc2_modifier(destination);
  if (point_cloud_parts.info->use_color) {
    // Data format: x,y,z,rgb; 16 bytes per point
    pc2_modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  } else {
    // Data format: x,y,z; 12 bytes per point
    pc2_modifier.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  }
  const cudaError_t cuda_error = cudaMemcpy(
    destination.data.data(),
    point_cloud_parts.points->data<float>().value(), destination.row_step * destination.height,
    cudaMemcpyDeviceToHost);

  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] cudaMemcpy failed for conversion from "
      "NitrosPointCloud to sensor_msgs::msg::PointCloud2: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_ros_message] "
    "row_step: %d, point_step: %d, x_offset: %d, y_offset: %d, y_offset: %d",
    destination.row_step,
    destination.point_step,
    destination.fields[0].offset,
    destination.fields[1].offset,
    destination.fields[2].offset);

  // Populate timestamp information back into ROS header
  auto input_timestamp = point_cloud_parts.timestamp;
  if (input_timestamp) {
    destination.header.stamp.sec = static_cast<int32_t>(
      input_timestamp->acqtime / static_cast<uint64_t>(1e9));
    destination.header.stamp.nanosec = static_cast<uint32_t>(
      input_timestamp->acqtime % static_cast<uint64_t>(1e9));
  }

  // Get pointer to posetree component
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kPoseTreeEntityName, kPoseTreeComponentName, kPoseTreeComponentTypeName, cid);
  auto maybe_pose_tree_handle =
    nvidia::gxf::Handle<nvidia::isaac::PoseTree>::Create(context, cid);
  if (!maybe_pose_tree_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get pose tree's handle: " <<
      GxfResultStr(maybe_pose_tree_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_tree_handle = maybe_pose_tree_handle.value();
  auto frame_name = pose_tree_handle->getFrameName(point_cloud_parts.pose_frame_uid->uid);
  if (frame_name) {
    destination.header.frame_id = frame_name.value();
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("NitrosPointCloud"), "Setting frame if from NITROS msg");
    // Set NITROS frame id as fallback method of populating frame_id
    // Set frame ID
    destination.header.frame_id = source.frame_id;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_ros_message] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosPointCloud,
  sensor_msgs::msg::PointCloud2>::convert_to_custom(
  const ros_message_type & source,
  custom_type & destination)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "NitrosPointCloud::convert_to_custom",
    nvidia::isaac_ros::nitros::CLR_PURPLE);

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
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
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator_handle = maybe_allocator_handle.value();

  const int32_t height = source.height;
  const int32_t width = source.width;
  const int32_t point_float_count = source.point_step / sizeof(float);
  const bool use_color = (point_float_count == 4);  // {x, y, z, RGB}
  const int32_t n_points = source.height * source.width;

  auto maybe_point_cloud_message_parts = nvidia::isaac_ros::messages::CreatePointCloudMessage(
    context, allocator_handle, n_points, use_color);
  if (!maybe_point_cloud_message_parts) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_ros_message] Failed to create CreatePointCloudMessage " << GxfResultStr(
      maybe_point_cloud_message_parts.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto point_cloud_message_parts = maybe_point_cloud_message_parts.value();

  point_cloud_message_parts.info->use_color = use_color;
  point_cloud_message_parts.info->is_bigendian = source.is_bigendian;

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_custom] height: %d, width: %d, point_float_count: %d, use_color: %d",
    height, width, point_float_count, use_color);

  // Copy data from point cloud msg to gxf tensor.
  const cudaError_t cuda_error = cudaMemcpy(
    point_cloud_message_parts.points->data<float>().value(),
    source.data.data(), source.row_step * source.height, cudaMemcpyHostToDevice);
  if (cuda_error != cudaSuccess) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] cudaMemcpy failed for copying data from "
      "sensor_msgs::msg::PointCloud2 to NitrosPointCloud: " <<
      cudaGetErrorName(cuda_error) <<
      " (" << cudaGetErrorString(cuda_error) << ")";
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  // Add timestamp to the message
  uint64_t input_timestamp =
    source.header.stamp.sec * static_cast<uint64_t>(1e9) +
    source.header.stamp.nanosec;
  point_cloud_message_parts.timestamp->acqtime = input_timestamp;

  // Get pointer to posetree component
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kPoseTreeEntityName, kPoseTreeComponentName, kPoseTreeComponentTypeName, cid);
  auto maybe_pose_tree_handle =
    nvidia::gxf::Handle<nvidia::isaac::PoseTree>::Create(context, cid);
  if (!maybe_pose_tree_handle) {
    std::stringstream error_msg;
    error_msg <<
      "[convert_to_custom] Failed to get pose tree's handle: " <<
      GxfResultStr(maybe_pose_tree_handle.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto pose_tree_handle = maybe_pose_tree_handle.value();
  auto maybe_pointcloud_frame_uid = pose_tree_handle->findOrCreateFrame(
    source.header.frame_id.c_str());

  if (maybe_pointcloud_frame_uid) {
    point_cloud_message_parts.pose_frame_uid->uid = maybe_pointcloud_frame_uid.value();
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("NitrosPointCloud"), "Could not create Pose Tree Frame");
  }

  // Set NITROS frame id as fallback method of populating frame_id
  destination.frame_id = source.header.frame_id;

  // Set Entity Id
  destination.handle = point_cloud_message_parts.message.eid();
  GxfEntityRefCountInc(context, point_cloud_message_parts.message.eid());

  RCLCPP_DEBUG(
    rclcpp::get_logger("NitrosPointCloud"),
    "[convert_to_custom] Conversion completed");

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}
