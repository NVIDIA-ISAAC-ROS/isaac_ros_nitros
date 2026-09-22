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

#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "isaac_ros_nitros_bridge_ros2/image_converter_node.hpp"

#define SYS_pidfd_getfd_nitros_bridge 438

namespace
{

class HostIpcRefGuard
{
public:
  explicit HostIpcRefGuard(
    std::shared_ptr<nvidia::isaac_ros::nitros_bridge::HostIPCBuffer> buffer)
  : buffer_(std::move(buffer)) {}

  HostIpcRefGuard(const HostIpcRefGuard &) = delete;
  HostIpcRefGuard & operator=(const HostIpcRefGuard &) = delete;

  ~HostIpcRefGuard()
  {
    if (buffer_) {
      buffer_->refcount_dec();
    }
  }

private:
  std::shared_ptr<nvidia::isaac_ros::nitros_bridge::HostIPCBuffer> buffer_;
};

class CudaEventGuard
{
public:
  ~CudaEventGuard()
  {
    if (opened_) {
      cudaEventDestroy(event_);
    }
  }

  cudaEvent_t * event_ptr() {return &event_;}

  void set_opened() {opened_ = true;}

private:
  cudaEvent_t event_{};
  bool opened_{false};
};

bool LogCuError(
  const rclcpp::Logger & logger, CUresult cuda_err, const char * what)
{
  if (CUDA_SUCCESS == cuda_err) {
    return true;
  }
  const char * error_str = nullptr;
  cuGetErrorString(cuda_err, &error_str);
  RCLCPP_ERROR(
    logger, "Failed to call %s %s", what,
    error_str != nullptr ? error_str : "unknown");
  return false;
}

}  // namespace

namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

ImageConverterNode::ImageConverterNode(const rclcpp::NodeOptions options)
: rclcpp::Node("image_converter_node", options),
  num_blocks_(declare_parameter<int64_t>("num_blocks", 40)),
      // Timeout in microseconds: duration to wait after refcount reaches 0
      // before recycling the buffer
  timeout_(declare_parameter<int64_t>("timeout", 500)),
  bridge_pub_qos_{::isaac_ros::common::AddQosParameter(
      *this, "DEFAULT", "bridge_pub_qos")},
  bridge_sub_qos_{::isaac_ros::common::AddQosParameter(
      *this, "DEFAULT", "bridge_sub_qos")},
  image_pub_qos_{::isaac_ros::common::AddQosParameter(
      *this, "DEFAULT", "ros_pub_qos")},
  image_sub_qos_{::isaac_ros::common::AddQosParameter(
      *this, "DEFAULT", "ros_sub_qos")}
{
  cudaSetDevice(0);
  cuDevicePrimaryCtxRetain(&ctx_, 0);
  auto cuda_err =
    cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
  if (cuda_err != cudaSuccess) {
    throw std::runtime_error("[NITROS Bridge] cudaStreamCreateWithFlags Error");
  }

  rclcpp::PublisherOptions image_pub_options;
  image_pub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Enable;
  rclcpp::SubscriptionOptions image_sub_options;
  image_sub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Enable;
  image_sub_options.acceptable_buffer_backends = "any";

  bridge_image_pub_ = create_publisher<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>(
      "ros2_output_bridge_image", bridge_pub_qos_);

  bridge_image_sub_ = create_subscription<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>(
      "ros2_input_bridge_image", bridge_sub_qos_,
      std::bind(
        &ImageConverterNode::BridgeToROSCallback, this,
        std::placeholders::_1));

  image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "ros2_output_image", image_pub_qos_, image_pub_options);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "ros2_input_image", image_sub_qos_,
      std::bind(
        &ImageConverterNode::ROSToBridgeCallback, this,
        std::placeholders::_1),
      image_sub_options);
}

ImageConverterNode::~ImageConverterNode()
{
  if (cuda_stream_ != nullptr) {
    cudaStreamSynchronize(cuda_stream_);
    cudaStreamDestroy(cuda_stream_);
    cuda_stream_ = nullptr;
  }
  cuDevicePrimaryCtxRelease(0);
}

void ImageConverterNode::BridgeToROSCallback(
  const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage::SharedPtr
  msg)
{
  cuCtxSetCurrent(ctx_);

  if (msg->data.size() < 2 || msg->height == 0 || msg->step == 0) {
    RCLCPP_ERROR(get_logger(), "Invalid bridge image metadata.");
    return;
  }

  CUdeviceptr gpu_buffer = 0ULL;
  CUmemGenericAllocationHandle generic_allocation_handle;

  const auto pid = msg->data[0];
  const auto fd = msg->data[1];
  const auto msg_uid = msg->uid;

  CudaEventGuard event_guard;
  if (msg->cuda_event_handle.size() != 0) {
    if (msg->cuda_event_handle.size() != sizeof(cudaIpcEventHandle_t)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid event handle size.");
      return;
    }
    cudaIpcEventHandle_t event_handle;
    memcpy(
      &event_handle, msg->cuda_event_handle.data(),
      sizeof(cudaIpcEventHandle_t));
    auto err = cudaIpcOpenEventHandle(event_guard.event_ptr(), event_handle);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(
        this->get_logger(), "cudaIpcOpenEventHandle failed: %s",
        cudaGetErrorString(err));
      return;
    }
    event_guard.set_opened();

    err = cudaEventSynchronize(*event_guard.event_ptr());
    if (err != cudaSuccess) {
      RCLCPP_ERROR(
        this->get_logger(), "CUDA event synchronize failed: %s",
        cudaGetErrorString(err));
      return;
    }
  }

  std::unique_ptr<HostIpcRefGuard> host_ipc_guard;
  if (!msg_uid.empty()) {
    const std::string shm_name = std::to_string(pid) + std::to_string(fd);
    auto host_ipc_buffer =
      std::make_shared<HostIPCBuffer>(shm_name, HostIPCBuffer::Mode::OPEN);
    if (!host_ipc_buffer->refcoun_inc_if_uid_match(msg_uid)) {
      RCLCPP_WARN(this->get_logger(), "Failed to match UID, skip.");
      return;
    }
    host_ipc_guard = std::make_unique<HostIpcRefGuard>(std::move(host_ipc_buffer));
  }

  if (handle_ptr_map_.find(fd) != handle_ptr_map_.end()) {
    gpu_buffer = handle_ptr_map_[fd];
    RCLCPP_DEBUG(this->get_logger(), "Found FD in local map.");
  } else {
    int pidfd = syscall(SYS_pidfd_open, pid, 0);
    if (pidfd <= 0) {
      RCLCPP_ERROR(get_logger(), "SYS_pidfd_open failed.");
      return;
    }
    int imported_fd = syscall(SYS_pidfd_getfd_nitros_bridge, pidfd, fd, 0);
    if (imported_fd <= 0) {
      RCLCPP_ERROR(get_logger(), "SYS_pidfd_getfd failed.");
      return;
    }

    auto cuda_err = cuMemImportFromShareableHandle(
        &generic_allocation_handle,
        reinterpret_cast<void *>(static_cast<uintptr_t>(imported_fd)),
        CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
    if (!LogCuError(
        get_logger(), cuda_err, "cuMemImportFromShareableHandle"))
    {
      return;
    }

    CUmemAllocationProp prop = {};
    prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
    prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    prop.location.id = 0;
    prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
    size_t granularity = 0;

    cuda_err = cuMemGetAllocationGranularity(
        &granularity, &prop, CU_MEM_ALLOC_GRANULARITY_MINIMUM);
    if (!LogCuError(
        get_logger(), cuda_err, "cuMemGetAllocationGranularity"))
    {
      return;
    }

    auto alloc_size = static_cast<size_t>(msg->height) * msg->step;
    alloc_size = alloc_size - (alloc_size % granularity) + granularity;

    cuda_err = cuMemAddressReserve(&gpu_buffer, alloc_size, 0, 0, 0);
    if (!LogCuError(get_logger(), cuda_err, "cuMemAddressReserve")) {
      return;
    }

    cuda_err =
      cuMemMap(gpu_buffer, alloc_size, 0, generic_allocation_handle, 0);
    if (!LogCuError(get_logger(), cuda_err, "cuMemMap")) {
      return;
    }

    CUmemAccessDesc accessDesc = {};
    accessDesc.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    accessDesc.location.id = 0;
    accessDesc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
    cuda_err = cuMemSetAccess(gpu_buffer, alloc_size, &accessDesc, 1);
    if (!LogCuError(get_logger(), cuda_err, "cuMemSetAccess")) {
      return;
    }
    handle_ptr_map_[fd] = gpu_buffer;
  }

  auto image = std::make_unique<sensor_msgs::msg::Image>();
  image->header = msg->header;
  image->height = msg->height;
  image->width = msg->width;
  image->encoding = msg->encoding;
  image->is_bigendian = msg->is_bigendian;
  image->step = msg->step;
  const size_t image_size_bytes = static_cast<size_t>(msg->height) * msg->step;
  try {
    image->data = cuda_buffer_backend::allocate_buffer(image_size_bytes);
    auto write_handle =
      cuda_buffer_backend::from_output_buffer(image->data, cuda_stream_);
    auto cuda_err = cudaMemcpyAsync(
        write_handle.get_ptr(), reinterpret_cast<void *>(gpu_buffer),
        image_size_bytes, cudaMemcpyDeviceToDevice, cuda_stream_);
    if (cuda_err != cudaSuccess) {
      RCLCPP_ERROR(
        get_logger(), "Failed to copy bridge image: %s",
        cudaGetErrorString(cuda_err));
      return;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to allocate or copy bridge image: %s", e.what());
    return;
  }

  auto cuda_err = cudaStreamSynchronize(cuda_stream_);
  if (cuda_err != cudaSuccess) {
    RCLCPP_ERROR(
      get_logger(), "Failed to synchronize bridge image copy: %s",
      cudaGetErrorString(cuda_err));
    return;
  }
  image_pub_->publish(std::move(image));
}

void ImageConverterNode::ROSToBridgeCallback(
  const sensor_msgs::msg::Image::SharedPtr msg)
{
  cuCtxSetCurrent(ctx_);

  const size_t image_size_bytes = static_cast<size_t>(msg->height) * msg->step;
  if (image_size_bytes == 0 || msg->data.size() < image_size_bytes) {
    RCLCPP_ERROR(get_logger(), "Image data is smaller than height * step.");
    return;
  }

  if (first_msg_received_ == false) {
    ipc_buffer_manager_ = std::make_shared<IPCBufferManager>(
        num_blocks_, image_size_bytes, timeout_);
    ipc_buffer_bytes_ = image_size_bytes;
    first_msg_received_ = true;
  } else if (image_size_bytes > ipc_buffer_bytes_) {
    RCLCPP_ERROR(
      get_logger(),
      "Image payload (%zu bytes) exceeds IPC pool size (%zu bytes).",
      image_size_bytes, ipc_buffer_bytes_);
    return;
  }

  auto ipc_buffer = ipc_buffer_manager_->find_next_available_buffer();

  isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage img_msg;
  img_msg.header = msg->header;
  img_msg.height = msg->height;
  img_msg.width = msg->width;
  img_msg.encoding = msg->encoding;
  img_msg.is_bigendian = msg->is_bigendian;
  img_msg.step = msg->step;

  try {
    auto read_handle =
      cuda_buffer_backend::from_input_buffer(msg->data, cuda_stream_);
    auto cuda_err = cudaMemcpyAsync(
        reinterpret_cast<void *>(ipc_buffer->d_ptr), read_handle.get_ptr(),
        image_size_bytes, cudaMemcpyDeviceToDevice, cuda_stream_);
    if (cudaSuccess != cuda_err) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to call cudaMemcpyAsync %s",
        cudaGetErrorString(cuda_err));
      return;
    }

    cuda_err = cudaStreamSynchronize(cuda_stream_);
    if (cudaSuccess != cuda_err) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to synchronize CUDA stream %s",
        cudaGetErrorString(cuda_err));
      return;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to copy image into bridge memory: %s", e.what());
    return;
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
RCLCPP_COMPONENTS_REGISTER_NODE(
    nvidia::isaac_ros::nitros_bridge::ImageConverterNode)
