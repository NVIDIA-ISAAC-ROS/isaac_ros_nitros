// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifdef NITROS_GXF_COMPAT_MODE

#include <cuda_runtime.h>

#include <sstream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/allocator.hpp"
#include "gxf/std/timestamp.hpp"
#include "messages/point_cloud_message.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud.hpp"
#include "isaac_ros_nitros_point_cloud_type/nitros_point_cloud_gxf_compat.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

namespace
{
const char kEntityName[] = "memory_pool";
const char kComponentName[] = "unbounded_allocator";
const char kComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";
}  // namespace

// NitrosPointCloud (GXF entity) -> NitrosPointCloud (value with handle)
NitrosPointCloud NitrosGxfCompatTraits<NitrosPointCloud>::CreateFromGxfEntity(
  gxf_context_t context,
  int64_t eid)
{
  auto entity = nvidia::gxf::Entity::Shared(context, eid);
  if (!entity) {
    throw std::runtime_error("NitrosPointCloud GXF compat: Failed to get GXF entity");
  }

  auto maybe_parts = nvidia::isaac_ros::messages::GetPointCloudMessage(entity.value());
  if (!maybe_parts) {
    std::stringstream error_msg;
    error_msg << "[CreateFromGxfEntity] Failed to get point cloud message: "
              << GxfResultStr(maybe_parts.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto parts = maybe_parts.value();

  NitrosPointCloud msg;
  msg.handle = eid;
  GxfEntityRefCountInc(context, eid);

  msg.width = static_cast<uint32_t>(parts.points->shape().dimension(0));
  msg.height = 1;
  msg.use_color = parts.info->use_color;
  msg.is_bigendian = parts.info->is_bigendian;
  msg.point_step = msg.use_color ? 16u : 12u;
  msg.row_step = msg.point_step * msg.width;

  if (parts.timestamp) {
    uint64_t acqtime = parts.timestamp->acqtime;
    msg.timestamp_sec = static_cast<uint32_t>(acqtime / 1000000000UL);
    msg.timestamp_nsec = static_cast<uint32_t>(acqtime % 1000000000UL);
  }
  msg.frame_id = "";  // GXF point cloud message has no frame_id component

  return msg;
}

// NitrosPointCloud (buffer or handle) -> GXF entity
int64_t NitrosGxfCompatTraits<NitrosPointCloud>::CreateGxfEntity(
  gxf_context_t context,
  const NitrosPointCloud & msg)
{
  if (msg.handle >= 0) {
    GxfEntityRefCountInc(context, static_cast<gxf_uid_t>(msg.handle));
    return msg.handle;
  }

  // Buffer-based: create GXF entity and copy buffer into it
  gxf_uid_t cid;
  GetTypeAdapterNitrosContext().getCid(
    kEntityName, kComponentName, kComponentTypeName, cid);
  auto maybe_allocator =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context, cid);
  if (!maybe_allocator) {
    std::stringstream error_msg;
    error_msg << "[CreateGxfEntity] Failed to get allocator: "
              << GxfResultStr(maybe_allocator.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto allocator = maybe_allocator.value();

  const int32_t num_points = static_cast<int32_t>(msg.width) * static_cast<int32_t>(msg.height);
  auto maybe_parts = nvidia::isaac_ros::messages::CreatePointCloudMessage(
    context, allocator, num_points, msg.use_color);
  if (!maybe_parts) {
    std::stringstream error_msg;
    error_msg << "[CreateGxfEntity] Failed to create point cloud message: "
              << GxfResultStr(maybe_parts.error());
    RCLCPP_ERROR(rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  auto parts = maybe_parts.value();

  parts.info->use_color = msg.use_color;
  parts.info->is_bigendian = msg.is_bigendian;
  parts.timestamp->acqtime =
    static_cast<uint64_t>(msg.timestamp_sec) * 1000000000UL +
    static_cast<uint64_t>(msg.timestamp_nsec);

  cudaStream_t stream = GetTypeAdapterNitrosContext().getCudaStreamFromNitrosGraph();
  auto read_handle = msg.get_read_handle(stream);
  const uint8_t * src = read_handle.get_ptr();
  if (!src) {
    throw std::runtime_error("[CreateGxfEntity] NitrosPointCloud buffer pointer is null");
  }

  const size_t data_size = static_cast<size_t>(num_points) * (msg.use_color ? 4 : 3) *
    sizeof(float);  // NOLINT
  auto maybe_dst = parts.points->data<float>();
  if (!maybe_dst) {
    throw std::runtime_error("[CreateGxfEntity] Tensor returned no data pointer");
  }

  cudaError_t err = cudaMemcpyAsync(
    *maybe_dst, src, data_size, cudaMemcpyDeviceToDevice, stream);
  if (err != cudaSuccess) {
    std::stringstream error_msg;
    error_msg << "[CreateGxfEntity] cudaMemcpyAsync failed: "
              << cudaGetErrorName(err) << " (" << cudaGetErrorString(err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  err = cudaStreamSynchronize(stream);
  if (err != cudaSuccess) {
    std::stringstream error_msg;
    error_msg << "[CreateGxfEntity] cudaStreamSynchronize failed: "
              << cudaGetErrorName(err) << " (" << cudaGetErrorString(err) << ")";
    RCLCPP_ERROR(rclcpp::get_logger("NitrosPointCloud"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }

  int64_t eid = parts.message.eid();
  GxfEntityRefCountInc(context, eid);
  return eid;
}

NITROS_GXF_COMPAT_REGISTER_CONVERTER(NitrosPointCloud)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NITROS_GXF_COMPAT_MODE
