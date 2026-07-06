// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// ============================================================================
// INTERIM: GXF Compatibility Implementation for NitrosFlatScan
// TODO(ayusmans): Remove entire file when all nodes migrated to GXF-free
// ============================================================================

#include "isaac_ros_nitros_flat_scan_type/nitros_flat_scan_gxf_compat.hpp"
#include "isaac_ros_nitros_flat_scan_type/nitros_flat_scan.hpp"

#include <cuda_runtime.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "messages/flat_scan_message.hpp"
#include "gems/pose_tree/pose_tree.hpp"
#include "extensions/atlas/pose_tree_frame.hpp"
#pragma GCC diagnostic pop

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros/types/type_adapter_nitros_context.hpp"
#include "isaac_ros_common/cuda_stream.hpp"

namespace
{
constexpr char kMemoryEntityName[] = "memory_pool";
constexpr char kMemoryComponentName[] = "unbounded_allocator";
constexpr char kMemoryComponentTypeName[] = "nvidia::gxf::UnboundedAllocator";
constexpr char kPoseTreeEntityName[] = "global_pose_tree";
constexpr char kPoseTreeComponentName[] = "pose_tree";
constexpr char kPoseTreeComponentTypeName[] = "nvidia::isaac::PoseTree";
constexpr int kFlatscanAngleIndx = 0;
constexpr int kFlatscanRangeIndx = 2;
constexpr int kNFieldsFlatscanMsg = 5;
}  // namespace

template<typename Deleter>
using unique_p = std::unique_ptr<double[], Deleter>;

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Register format callbacks for NitrosNode compatibility
NITROS_GXF_COMPAT_FORMATS_BEGIN(NitrosFlatScan)
NITROS_GXF_COMPAT_FORMAT_ADD(nitros_flat_scan_t)
NITROS_GXF_COMPAT_FORMATS_END()

// GXF entity -> NitrosFlatScan (with GPU buffer)
NitrosFlatScan NitrosGxfCompatTraits<NitrosFlatScan>::CreateFromGxfEntity(
  gxf_context_t context,
  int64_t eid)
{
  auto entity = nvidia::gxf::Entity::Shared(context, eid);
  if (!entity) {
    throw std::runtime_error("[NitrosFlatScan compat] Failed to get GXF entity");
  }

  auto maybe_flatscan_parts = nvidia::isaac_ros::messages::GetFlatscanMessage(entity.value());
  if (!maybe_flatscan_parts) {
    std::string error_msg =
      "[NitrosFlatScan compat] Failed to get flatscan message: " +
      std::string(GxfResultStr(maybe_flatscan_parts.error()));
    throw std::runtime_error(error_msg);
  }
  auto flatscan_parts = maybe_flatscan_parts.value();

  auto beams_tensor = flatscan_parts.beams;
  const auto & beams_tensor_shape = beams_tensor->shape();

  if (beams_tensor->rank() != 2 || beams_tensor_shape.dimension(1) != kNFieldsFlatscanMsg) {
    throw std::runtime_error("[NitrosFlatScan compat] Unexpected tensor shape");
  }

  const uint32_t num_points = static_cast<uint32_t>(beams_tensor_shape.dimension(0));

  // Extract beam data from GXF tensor to host-side staging vectors
  auto deleter = [](double * ptr) {cudaFreeHost(ptr);};
  auto cuda_alloc = [](size_t sz) {
      void * ptr;
      cudaMallocHost(reinterpret_cast<void **>(&ptr), sz);
      return ptr;
    };
  unique_p<decltype(deleter)> beams_cpu(
    reinterpret_cast<double *>(cuda_alloc(beams_tensor->size())), deleter);

  ::nvidia::isaac::CpuTensorView2d beams_view;
  switch (beams_tensor->storage_type()) {
    case nvidia::gxf::MemoryStorageType::kHost:
      beams_view = ::nvidia::isaac::CreateCpuTensorViewFromData<double, 2>(
        beams_tensor->data<double>().value(), beams_tensor_shape.size(),
        ::nvidia::isaac::Vector2i(
          beams_tensor_shape.dimension(0), beams_tensor_shape.dimension(1)));
      break;
    case nvidia::gxf::MemoryStorageType::kDevice:
      {
        cudaError_t err = cudaMemcpy(
          beams_cpu.get(), beams_tensor->pointer(),
          beams_tensor->size(), cudaMemcpyDeviceToHost);
        if (err != cudaSuccess) {
          throw std::runtime_error(
                  std::string("[NitrosFlatScan compat] cudaMemcpy D2H failed: ") +
                  cudaGetErrorString(err));
        }
        beams_view = ::nvidia::isaac::CreateCpuTensorViewFromData<double, 2>(
          beams_cpu.get(), beams_tensor_shape.size(),
          ::nvidia::isaac::Vector2i(
            beams_tensor_shape.dimension(0), beams_tensor_shape.dimension(1)));
      }
      break;
    default:
      throw std::runtime_error("[NitrosFlatScan compat] Unsupported tensor storage type");
  }

  // Stage beam data in SoA float layout: [angles | ranges]
  std::vector<float> soa_host(num_points * 2);
  float * h_angles = soa_host.data();
  float * h_ranges = soa_host.data() + num_points;
  for (uint32_t i = 0; i < num_points; ++i) {
    h_angles[i] = static_cast<float>(beams_view(i, kFlatscanAngleIndx));
    h_ranges[i] = static_cast<float>(beams_view(i, kFlatscanRangeIndx));
  }

  // Upload SoA data to GPU buffer
  NitrosFlatScan msg;
  msg.num_beams = num_points;
  msg.range_max = static_cast<float>(flatscan_parts.info->out_of_range);
  msg.range_min = 0.0f;

  if (num_points > 0) {
    const size_t total_bytes = num_points * 2 * sizeof(float);
    uint8_t * d_ptr = nullptr;
    cudaError_t err = cudaMalloc(&d_ptr, total_bytes);
    if (err != cudaSuccess) {
      throw std::runtime_error("[NitrosFlatScan compat] cudaMalloc failed");
    }

    // RAII guard so d_ptr is freed if make_shared throws
    auto cuda_deleter = [](uint8_t * p) {if (p) {cudaFree(p);}};
    std::unique_ptr<uint8_t, decltype(cuda_deleter)> guard(d_ptr, cuda_deleter);
    msg.buffer_ = std::make_shared<nvidia::isaac_ros::nitros::NitrosBuffer>(
      guard.release(), total_bytes);

    err = cudaMemcpy(d_ptr, soa_host.data(), total_bytes, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
      throw std::runtime_error("[NitrosFlatScan compat] cudaMemcpy H2D failed");
    }
  }

  // Extract timestamp
  auto input_timestamp = flatscan_parts.timestamp;
  if (input_timestamp) {
    msg.timestamp_sec = static_cast<uint32_t>(
      input_timestamp->acqtime / static_cast<uint64_t>(1e9));
    msg.timestamp_nsec = static_cast<uint32_t>(
      input_timestamp->acqtime % static_cast<uint64_t>(1e9));
  }

  // Resolve frame_id from PoseTree
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kPoseTreeEntityName, kPoseTreeComponentName, kPoseTreeComponentTypeName, cid);
  auto maybe_pose_tree =
    nvidia::gxf::Handle<nvidia::isaac::PoseTree>::Create(context, cid);
  if (maybe_pose_tree) {
    auto frame_name = maybe_pose_tree.value()->getFrameName(
      flatscan_parts.pose_frame_uid->uid);
    if (frame_name) {
      msg.frame_id = frame_name.value();
    }
  }

  msg.handle = -1;
  return msg;
}

// NitrosFlatScan (GPU buffer) -> GXF entity
int64_t NitrosGxfCompatTraits<NitrosFlatScan>::CreateGxfEntity(
  gxf_context_t context,
  const NitrosFlatScan & msg)
{
  // Get allocator
  gxf_uid_t cid;
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kMemoryEntityName, kMemoryComponentName, kMemoryComponentTypeName, cid);
  auto maybe_allocator =
    nvidia::gxf::Handle<nvidia::gxf::Allocator>::Create(context, cid);
  if (!maybe_allocator) {
    throw std::runtime_error("[NitrosFlatScan compat] Failed to get allocator");
  }

  const int n_points = static_cast<int>(msg.num_beams);
  auto maybe_flatscan_parts = nvidia::isaac_ros::messages::CreateFlatscanMessage(
    context, maybe_allocator.value(), n_points, false);
  if (!maybe_flatscan_parts) {
    throw std::runtime_error(
      "[NitrosFlatScan compat] Failed to create flatscan message: " +
      std::string(GxfResultStr(maybe_flatscan_parts.error())));
  }
  auto flatscan_parts = maybe_flatscan_parts.value();

  // Download beam data from GPU buffer to host, then convert SoA float -> AoS double
  if (n_points > 0) {
    auto buffer = nvidia::isaac_ros::nitros::NitrosBufferAccessor<NitrosFlatScan>::get_buffer(msg);
    const size_t plane_bytes = n_points * sizeof(float);
    std::vector<float> h_angles(n_points);
    std::vector<float> h_ranges(n_points);
    const uint8_t * gpu_ptr = buffer->get_data();
    cudaError_t err = cudaMemcpy(
      h_angles.data(), gpu_ptr, plane_bytes, cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
      throw std::runtime_error(
              std::string("[NitrosFlatScan compat] cudaMemcpy angles D2H failed: ") +
              cudaGetErrorString(err));
    }
    err = cudaMemcpy(
      h_ranges.data(), gpu_ptr + plane_bytes, plane_bytes, cudaMemcpyDeviceToHost);
    if (err != cudaSuccess) {
      throw std::runtime_error(
              std::string("[NitrosFlatScan compat] cudaMemcpy ranges D2H failed: ") +
              cudaGetErrorString(err));
    }

    auto beams_tensor = flatscan_parts.beams;
    size_t beams_size_bytes = n_points * sizeof(double) * kNFieldsFlatscanMsg;

    auto dbl_deleter = [](double * ptr) {cudaFreeHost(ptr);};
    unique_p<decltype(dbl_deleter)> beams_staging(new double[n_points], dbl_deleter);
    cudaMallocHost(reinterpret_cast<void **>(&beams_staging), beams_size_bytes);
    ::nvidia::isaac::CpuTensorView2d beams_view =
      ::nvidia::isaac::CreateCpuTensorViewFromData<double, 2>(
      beams_staging.get(), beams_size_bytes,
      ::nvidia::isaac::Vector2i(n_points, kNFieldsFlatscanMsg));

    for (int i = 0; i < n_points; ++i) {
      beams_view(i, kFlatscanAngleIndx) = h_angles[i];
      beams_view(i, kFlatscanRangeIndx) = h_ranges[i];
    }

    auto nitros_cuda_stream =
      nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCudaStreamFromNitrosGraph();

    switch (beams_tensor->storage_type()) {
      case nvidia::gxf::MemoryStorageType::kHost:
        cudaMemcpyAsync(
          beams_tensor->data<double>().value(), beams_staging.get(),
          beams_size_bytes, cudaMemcpyHostToHost, nitros_cuda_stream);
        break;
      case nvidia::gxf::MemoryStorageType::kDevice:
        cudaMemcpyAsync(
          beams_tensor->data<double>().value(), beams_staging.get(),
          beams_size_bytes, cudaMemcpyHostToDevice, nitros_cuda_stream);
        break;
      default:
        throw std::runtime_error("[NitrosFlatScan compat] Unsupported tensor storage type");
    }
  }

  flatscan_parts.info->out_of_range = msg.range_max;

  // Set timestamp
  uint64_t acqtime =
    static_cast<uint64_t>(msg.timestamp_sec) * static_cast<uint64_t>(1e9) +
    msg.timestamp_nsec;
  flatscan_parts.timestamp->acqtime = acqtime;

  // Set PoseTree frame
  nvidia::isaac_ros::nitros::GetTypeAdapterNitrosContext().getCid(
    kPoseTreeEntityName, kPoseTreeComponentName, kPoseTreeComponentTypeName, cid);
  auto maybe_pose_tree =
    nvidia::gxf::Handle<nvidia::isaac::PoseTree>::Create(context, cid);
  if (maybe_pose_tree) {
    auto maybe_uid = maybe_pose_tree.value()->findOrCreateFrame(msg.frame_id.c_str());
    if (maybe_uid) {
      flatscan_parts.pose_frame_uid->uid = maybe_uid.value();
    }
  }

  int64_t eid = flatscan_parts.entity.eid();
  GxfEntityRefCountInc(context, eid);
  return eid;
}

NITROS_GXF_COMPAT_REGISTER_CONVERTER(NitrosFlatScan)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // NITROS_GXF_COMPAT_MODE
