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


#include <cuda_runtime_api.h>
#include <sys/un.h>
#include <string>

#include "isaac_ros_nitros_bridge_ros2/image_converter_node.hpp"
#include "sensor_msgs/image_encodings.hpp"

#define SYS_pidfd_getfd_nitros_bridge 438


namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

ImageConverterNode::ImageConverterNode(const rclcpp::NodeOptions options)
: rclcpp::Node("image_converter_node", options),
  num_blocks_(declare_parameter<int64_t>("num_blocks", 40)),
  // Timeout in microseconds: duration to wait after refcount reaches 0 before recycling the buffer
  timeout_(declare_parameter<int64_t>("timeout", 500)),
  bridge_pub_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "bridge_pub_qos")},
  bridge_sub_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "bridge_sub_qos")},
  nitros_pub_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "nitros_pub_qos")},
  nitros_sub_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "nitros_sub_qos")}
{
  cudaSetDevice(0);
  cuDevicePrimaryCtxRetain(&ctx_, 0);
  auto cuda_err = cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
  if (cuda_err != cudaSuccess) {
    throw std::runtime_error("[NITROS Bridge] cudaStreamCreateWithFlags Error");
  }

  cudaEventCreateWithFlags(&event_, cudaEventInterprocess | cudaEventDisableTiming);
  cudaIpcGetEventHandle(reinterpret_cast<cudaIpcEventHandle_t *>(&ipc_event_handle_), event_);

  rclcpp::PublisherOptions nitros_pub_options;
  nitros_pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  rclcpp::SubscriptionOptions nitros_sub_options;
  nitros_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  bridge_image_pub_ = create_publisher<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>(
    "ros2_output_bridge_image", bridge_pub_qos_);

  bridge_image_sub_ = create_subscription<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>(
    "ros2_input_bridge_image", bridge_sub_qos_,
    std::bind(&ImageConverterNode::BridgeToROSCallback, this, std::placeholders::_1));

  nitros_pub_ = create_publisher<nvidia::isaac_ros::nitros::NitrosImage>(
    "ros2_output_image", nitros_pub_qos_, nitros_pub_options);

  nitros_sub_ = create_subscription<nvidia::isaac_ros::nitros::NitrosImage>(
    "ros2_input_image", nitros_sub_qos_,
    std::bind(&ImageConverterNode::ROSToBridgeCallback, this, std::placeholders::_1),
    nitros_sub_options);
}

ImageConverterNode::~ImageConverterNode()
{
  if (cuda_stream_ != nullptr) {
    cudaStreamDestroy(cuda_stream_);
  }
}

void ImageConverterNode::BridgeToROSCallback(
  const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage::SharedPtr msg)
{
  cuCtxSetCurrent(ctx_);

  CUdeviceptr gpu_buffer = 0ULL;
  CUmemGenericAllocationHandle generic_allocation_handle;

  auto pid = msg->data[0];
  auto fd = msg->data[1];

  auto msg_uid = msg->uid;
  cudaEvent_t event;
  cudaIpcEventHandle_t event_handle;

  // Construct CUDA IPC event handle if it exists
  if (msg->cuda_event_handle.size() != 0) {
    if (msg->cuda_event_handle.size() != sizeof(cudaIpcEventHandle_t)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid event handle size.");
      return;
    }
    memcpy(&event_handle, msg->cuda_event_handle.data(), sizeof(cudaIpcEventHandle_t));
    auto err = cudaIpcOpenEventHandle(&event, event_handle);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(
        this->get_logger(), "cudaIpcOpenEventHandle failed: %s",
        cudaGetErrorString(err));
      return;
    }

    // The event may record the completion of the previous operation
    err = cudaEventSynchronize(event);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(
        this->get_logger(), "CUDA event synchronize failed: %s",
        cudaGetErrorString(err));
      return;
    }
  }

  // Compare UID if exists
  std::shared_ptr<HostIPCBuffer> host_ipc_buffer;
  if (!msg_uid.empty()) {
    std::string shm_name = std::to_string(pid) + std::to_string(fd);
    host_ipc_buffer = std::make_shared<HostIPCBuffer>(shm_name, HostIPCBuffer::Mode::OPEN);
    if (!host_ipc_buffer->refcoun_inc_if_uid_match(msg_uid)) {
      RCLCPP_WARN(this->get_logger(), "Failed to match UID, skip.");
      return;
    }
  }

  if (handle_ptr_map_.find(msg->data.data()[1]) != handle_ptr_map_.end()) {
    gpu_buffer = handle_ptr_map_[msg->data.data()[1]];
    RCLCPP_DEBUG(this->get_logger(), "Found FD in local map.");
  } else {
    int pidfd = syscall(SYS_pidfd_open, msg->data.data()[0], 0);
    if (pidfd <= 0) {
      perror("SYS_pidfd_open failed");
    }
    int fd = syscall(SYS_pidfd_getfd_nitros_bridge, pidfd, msg->data.data()[1], 0);
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
      RCLCPP_ERROR(
        this->get_logger(), "Failed to call cuMemImportFromShareableHandle %s",
        error_str);
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
      RCLCPP_ERROR(
        this->get_logger(), "Failed to call cuMemGetAllocationGranularity %s",
        error_str);
      throw std::runtime_error(
              "[NITROS Bridge] cuMemGetAllocationGranularity Error");
    }

    auto alloc_size = msg->height * msg->step;
    // The alloc size must be the integral multiple of granularity
    alloc_size = alloc_size - (alloc_size % granularity) + granularity;

    cuda_err = cuMemAddressReserve(&gpu_buffer, alloc_size, 0, 0, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(this->get_logger(), "Failed to call cuMemAddressReserve %s", error_str);
      throw std::runtime_error("[NITROS Bridge] cuMemAddressReserve Error");
    }

    cuda_err = cuMemMap(gpu_buffer, alloc_size, 0, generic_allocation_handle, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(this->get_logger(), "Failed to call cuMemMap %s", error_str);
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
      RCLCPP_ERROR(this->get_logger(), "Failed to call cuMemSetAccess %s", error_str);
      throw std::runtime_error("[NITROS Bridge] cuMemMap Error");
    }
    handle_ptr_map_[msg->data.data()[1]] = gpu_buffer;
  }

  auto host_ipc_buffer_ptr = host_ipc_buffer;
  auto deleter = [host_ipc_buffer_ptr](uint8_t * p) {
      (void)p;
      if (host_ipc_buffer_ptr) {
        host_ipc_buffer_ptr->refcount_dec();
      }
    };

  nvidia::isaac_ros::nitros::NitrosImage nitros_image;
  {
    [[maybe_unused]] auto write_handle = nitros_image.from_external(
      reinterpret_cast<void *>(gpu_buffer),
      msg->height * msg->step,
      msg->width, msg->height, msg->step, msg->encoding, cuda_stream_, deleter);
  }
  nitros_image.timestamp_sec = msg->header.stamp.sec;
  nitros_image.timestamp_nsec = msg->header.stamp.nanosec;
  nitros_image.frame_id = msg->header.frame_id;

  nitros_pub_->publish(nitros_image);

  RCLCPP_DEBUG(this->get_logger(), "NITROS Image is Published from NITROS Bridge.");
}

void ImageConverterNode::ROSToBridgeCallback(
  const nvidia::isaac_ros::nitros::NitrosImage::SharedPtr msg)
{
  cuCtxSetCurrent(ctx_);

  const size_t image_size_bytes = msg->get_data_size();

  if (first_msg_received_ == false) {
    ipc_buffer_manager_ = std::make_shared<IPCBufferManager>(
      num_blocks_, image_size_bytes, timeout_);
    first_msg_received_ = true;
  }

  auto ipc_buffer = ipc_buffer_manager_->find_next_available_buffer();

  isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage img_msg;
  img_msg.header.frame_id = msg->get_frame_id();
  img_msg.header.stamp.sec = msg->get_timestamp_sec();
  img_msg.header.stamp.nanosec = msg->get_timestamp_nsec();
  img_msg.height = msg->height;
  img_msg.width = msg->width;
  img_msg.encoding = msg->encoding;
  img_msg.step = msg->step;

  auto read_handle = msg->get_read_handle(cuda_stream_);
  auto cuda_err = cudaMemcpyAsync(
    reinterpret_cast<void *>(ipc_buffer->d_ptr),
    read_handle.get_ptr(),
    image_size_bytes,
    cudaMemcpyDeviceToDevice,
    cuda_stream_);
  if (cudaSuccess != cuda_err) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to call cudaMemcpyAsync %s",
      cudaGetErrorString(cuda_err));
    throw std::runtime_error("[NITROS Bridge] cudaMemcpyAsync Error");
  }

  cuda_err = cudaStreamSynchronize(cuda_stream_);
  if (cudaSuccess != cuda_err) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to synchronize CUDA stream %s",
      cudaGetErrorString(cuda_err));
    throw std::runtime_error("[NITROS Bridge] cudaStreamSynchronize Error");
  }

  img_msg.data.push_back(ipc_buffer->pid);
  img_msg.data.push_back(ipc_buffer->fd);
  img_msg.uid = ipc_buffer->uid;
  img_msg.device_id = 0;

  bridge_image_pub_->publish(img_msg);
}

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros_bridge::ImageConverterNode)
