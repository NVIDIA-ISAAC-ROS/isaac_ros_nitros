// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


#include <fcntl.h>
#include <errno.h>
#include <sys/un.h>

#include "image_converter_node.hpp"

#define SYS_pidfd_getfd_nitros_bridge 438


namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

void ImageConverterNode::onInit()
{
  ros::NodeHandle & private_nh = getPrivateNodeHandle();

  // Create publisher topics
  bridge_image_pub_ = private_nh.advertise<isaac_ros_nitros_bridge_msgs::NitrosBridgeImage>(
    "ros1_output_bridge_image", 10);
  image_pub_ = private_nh.advertise<sensor_msgs::Image>("ros1_output_image", 10);

  // Create subscriber topics
  image_sub_ = private_nh.subscribe(
    "ros1_input_image", 10, &ImageConverterNode::ROSToBridgeCallback,
    this);
  bridge_image_sub_ = private_nh.subscribe(
    "ros1_input_bridge_image", 10,
    &ImageConverterNode::BridgeToROSCallback, this);

  private_nh.param("num_blocks", num_blocks_, 40);
  private_nh.param("timeout", timeout_, 100);

  cudaSetDevice(0);
  cuDevicePrimaryCtxRetain(&ctx_, 0);

  cudaEventCreateWithFlags(&event_, cudaEventInterprocess | cudaEventDisableTiming);
  cudaIpcGetEventHandle(reinterpret_cast<cudaIpcEventHandle_t *>(&ipc_event_handle_), event_);
}

void ImageConverterNode::BridgeToROSCallback(
  const isaac_ros_nitros_bridge_msgs::NitrosBridgeImage::ConstPtr & input_msg)
{
  cuCtxSetCurrent(ctx_);

  CUdeviceptr gpu_buffer = 0ULL;
  CUmemGenericAllocationHandle generic_allocation_handle;

  auto pid = input_msg->data[0];
  auto fd = input_msg->data[1];

  auto msg_uid = input_msg->uid;
  cudaEvent_t event;
  cudaIpcEventHandle_t event_handle;

  // Construct CUDA IPC event handle if it exists
  if (input_msg->cuda_event_handle.size() != 0) {
    if (input_msg->cuda_event_handle.size() != sizeof(cudaIpcEventHandle_t)) {
      ROS_ERROR("Invalid event handle size.");
      return;
    }
    memcpy(&event_handle, input_msg->cuda_event_handle.data(), sizeof(cudaIpcEventHandle_t));
    auto err = cudaIpcOpenEventHandle(&event, event_handle);
    if (err != cudaSuccess) {
      ROS_ERROR( "cudaIpcOpenEventHandle failed: %s", cudaGetErrorString(err));
      return;
    }

    // The event may record the completion of the previous operation
    err = cudaEventSynchronize(event);
    if (err != cudaSuccess) {
      ROS_ERROR("CUDA event synchronize failed: %s", cudaGetErrorString(err));
      return;
    }
  }

  // Compare UID if exists
  if (!msg_uid.empty()) {
    std::string shm_name = std::to_string(pid) + std::to_string(fd);
    host_ipc_buffer_ = std::make_shared<HostIPCBuffer>(shm_name, HostIPCBuffer::Mode::OPEN);
    if (!host_ipc_buffer_->refcoun_inc_if_uid_match(msg_uid)) {
      ROS_WARN("Failed to match UID, skip.");
      return;
    }
  }

  if (handle_ptr_map_.find(input_msg->data.data()[1]) != handle_ptr_map_.end()) {
    gpu_buffer = handle_ptr_map_[input_msg->data.data()[1]];
    ROS_DEBUG("Found FD in local map.");
  } else {
    int pidfd = syscall(SYS_pidfd_open, input_msg->data.data()[0], 0);
    if (pidfd <= 0) {
      perror("SYS_pidfd_open failed");
    }
    int fd = syscall(SYS_pidfd_getfd_nitros_bridge, pidfd, input_msg->data.data()[1], 0);
    if (fd <= 0) {
      perror("SYS_pidfd_getfd failed");
    }

    auto cuda_err = cuMemImportFromShareableHandle(
      &generic_allocation_handle,
      reinterpret_cast<void *>((uintptr_t)fd),
      CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR( "Failed to call cuMemImportFromShareableHandle %s", error_str);
      throw std::runtime_error("[NITROS Bridge] cuMemImportFromShareableHandle Error");
    }

    CUmemAllocationProp prop = {};
    prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
    prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    prop.location.id = 0;
    prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
    size_t granularity = 0;

    cuda_err = cuMemGetAllocationGranularity(
      &granularity, &prop, CU_MEM_ALLOC_GRANULARITY_MINIMUM);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR( "Failed to call cuMemGetAllocationGranularity %s", error_str);
      throw std::runtime_error(
              "[NITROS Bridge] cuMemGetAllocationGranularity Error");
    }

    auto alloc_size = input_msg->height * input_msg->step;
    // The alloc size must be the integral multiple of granularity
    alloc_size = alloc_size - (alloc_size % granularity) + granularity;

    cuda_err = cuMemAddressReserve(&gpu_buffer, alloc_size, 0, 0, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR("Failed to call cuMemAddressReserve %s", error_str);
      throw std::runtime_error("[NITROS Bridge] cuMemAddressReserve Error");
    }

    cuda_err = cuMemMap(gpu_buffer, alloc_size, 0, generic_allocation_handle, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR("Failed to call cuMemMap %s", error_str);
      throw std::runtime_error("[NITROS Bridge] cuMemMap Error");
    }

    CUmemAccessDesc accessDesc = {};
    accessDesc.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    accessDesc.location.id = 0;
    accessDesc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
    cuda_err = cuMemSetAccess(gpu_buffer, alloc_size, &accessDesc, 1);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR("Failed to call cuMemSetAccess %s", error_str);
      throw std::runtime_error("[NITROS Bridge] cuMemMap Error");
    }
    handle_ptr_map_[input_msg->data.data()[1]] = gpu_buffer;
  }

  sensor_msgs::Image output_msg;
  output_msg.header = input_msg->header;
  output_msg.height = input_msg->height;
  output_msg.width = input_msg->width;
  output_msg.encoding = input_msg->encoding;
  output_msg.step = input_msg->step;
  output_msg.data.resize(input_msg->step * input_msg->height);

  auto cuda_err = cuMemcpyDtoH(
  output_msg.data.data(), gpu_buffer, input_msg->step * input_msg->height);
  if (CUDA_SUCCESS != cuda_err) {
    const char * error_str = NULL;
    cuGetErrorString(cuda_err, &error_str);
    ROS_ERROR("Failed to call cuMemcpyDtoH %s", error_str);
  }

  image_pub_.publish(output_msg);

  // Update refcount if it exists
  if (!msg_uid.empty()) {
    host_ipc_buffer_->refcount_dec();
  }
  ROS_DEBUG("NITROS Image is Published from NITROS Bridge.");
}

void ImageConverterNode::ROSToBridgeCallback(const sensor_msgs::Image::ConstPtr & input_msg)
{
  cuCtxSetCurrent(ctx_);

  if (first_msg_received_ == false) {
    ipc_buffer_manager_ = std::make_shared<
      IPCBufferManager>(num_blocks_, input_msg->step * input_msg->height, timeout_);
    first_msg_received_ = true;
  }

  auto ipc_buffer = ipc_buffer_manager_->find_next_available_buffer();

  isaac_ros_nitros_bridge_msgs::NitrosBridgeImage msg;
  msg.header = input_msg->header;
  msg.height = input_msg->height;
  msg.width = input_msg->width;
  msg.encoding = input_msg->encoding;
  msg.step = input_msg->step;

  auto cuda_err = cuMemcpyHtoD(
    ipc_buffer->d_ptr,
    input_msg->data.data(),
    msg.step * msg.height);
  if (CUDA_SUCCESS != cuda_err) {
    const char * error_str = NULL;
    cuGetErrorString(cuda_err, &error_str);
    ROS_ERROR("Failed to call cuMemcpyDtoD %s", error_str);
    throw std::runtime_error("[NITROS Bridge] cuMemcpyDtoD Error");
  }

  msg.data.push_back(ipc_buffer->pid);
  msg.data.push_back(ipc_buffer->fd);
  msg.uid = ipc_buffer->uid;
  msg.device_id = 0;

  bridge_image_pub_.publish(msg);
}

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

PLUGINLIB_EXPORT_CLASS(nvidia::isaac_ros::nitros_bridge::ImageConverterNode, nodelet::Nodelet);
