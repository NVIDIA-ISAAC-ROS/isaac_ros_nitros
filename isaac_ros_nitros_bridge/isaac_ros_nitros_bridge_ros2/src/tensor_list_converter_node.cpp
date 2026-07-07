// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <atomic>
#include <utility>

#include "isaac_ros_nitros_bridge_ros2/tensor_list_converter_node.hpp"


#define SYS_pidfd_getfd_nitros_bridge 438


namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

TensorListConverterNode::TensorListConverterNode(const rclcpp::NodeOptions options)
: rclcpp::Node("tensor_list_converter_node", options),
  num_blocks_(declare_parameter<int64_t>("num_blocks", 40)),
  // Timeout in microseconds: duration to wait after refcount reaches 0 before recycling the buffer
  timeout_(declare_parameter<int64_t>("timeout", 500)),
  nitros_pub_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "nitros_pub_qos")},
  nitros_sub_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "nitros_sub_qos")},
  bridge_pub_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "bridge_pub_qos")},
  bridge_sub_qos_{::isaac_ros::common::AddQosParameter(*this, "DEFAULT", "bridge_sub_qos")}
{
  cudaSetDevice(0);
  cuDevicePrimaryCtxRetain(&ctx_, 0);
  auto cuda_runtime_err = cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
  if (cuda_runtime_err != cudaSuccess) {
    throw std::runtime_error("[NITROS Bridge] cudaStreamCreateWithFlags Error");
  }

  cudaEventCreateWithFlags(&event_, cudaEventInterprocess | cudaEventDisableTiming);
  cudaIpcGetEventHandle(reinterpret_cast<cudaIpcEventHandle_t *>(&ipc_event_handle_), event_);

  rclcpp::PublisherOptions nitros_pub_options;
  nitros_pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  rclcpp::SubscriptionOptions nitros_sub_options;
  nitros_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  nitros_pub_ = create_publisher<nvidia::isaac_ros::nitros::NitrosTensorList>(
    "ros2_output_tensor_list", nitros_pub_qos_, nitros_pub_options);

  nitros_sub_ = create_subscription<nvidia::isaac_ros::nitros::NitrosTensorList>(
    "ros2_input_tensor_list", nitros_sub_qos_,
    std::bind(&TensorListConverterNode::ROSToBridgeCallback, this, std::placeholders::_1),
    nitros_sub_options);

  nitros_bridge_pub_ = create_publisher<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>(
    "ros2_output_bridge_tensor_list", bridge_pub_qos_);
  nitros_bridge_sub_ = create_subscription<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>(
    "ros2_input_bridge_tensor_list", bridge_sub_qos_,
    std::bind(&TensorListConverterNode::BridgeToROSCallback, this, std::placeholders::_1));
}

TensorListConverterNode::~TensorListConverterNode()
{
  if (cuda_stream_ != nullptr) {
    cudaStreamDestroy(cuda_stream_);
  }
}

void TensorListConverterNode::BridgeToROSCallback(
  const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList::SharedPtr msg)
{
  cuCtxSetCurrent(ctx_);

  CUdeviceptr gpu_buffer = 0ULL;
  CUmemGenericAllocationHandle generic_allocation_handle;

  auto msg_pid = msg->pid;
  auto msg_fd = msg->fd;
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
      fprintf(stderr, "cudaIpcOpenEventHandle failed: %s\n", cudaGetErrorString(err));
      return;
    }

    // The event may record the completion of the previous operation
    err = cudaEventSynchronize(event);
    if (err != cudaSuccess) {
      fprintf(stderr, "CUDA event synchronize failed: %s\n", cudaGetErrorString(err));
      return;
    }
  }

  // Get total size of all tensors
  size_t total_size = 0;
  for (size_t i = 0; i < msg->tensors.size(); i++) {
    auto tensor = msg->tensors[i];
    if (tensor.shape.rank == 0) {
      RCLCPP_INFO(this->get_logger(), "Received tensor with rank 0, skip.");
      continue;
    }
    if (tensor.shape.dims.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid tensor shape.");
      return;
    }
    if (tensor.strides.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid tensor strides.");
      return;
    }
    total_size += tensor.strides[0] * tensor.shape.dims[0];
  }

  // Compare UID if exists
  std::shared_ptr<HostIPCBuffer> host_ipc_buffer;
  if (!msg_uid.empty()) {
    std::string shm_name = std::to_string(msg_pid) + std::to_string(msg_fd);
    host_ipc_buffer = std::make_shared<HostIPCBuffer>(shm_name, HostIPCBuffer::Mode::OPEN);
    if (!host_ipc_buffer->refcoun_inc_if_uid_match(msg_uid)) {
      RCLCPP_WARN(this->get_logger(), "Failed to match UID, skip.");
      return;
    }
  }

  if (handle_ptr_map_.find(msg_fd) != handle_ptr_map_.end()) {
    gpu_buffer = handle_ptr_map_[msg_fd];
    RCLCPP_DEBUG(this->get_logger(), "Found FD in local map.");
  } else {
    int pidfd = syscall(SYS_pidfd_open, msg_pid, 0);
    if (pidfd <= 0) {
      perror("SYS_pidfd_open failed");
    }
    int fd = syscall(SYS_pidfd_getfd_nitros_bridge, pidfd, msg_fd, 0);
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
      throw std::runtime_error(
              "[NITROS Bridge] cuMemImportFromShareableHandle Error");
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

    // The alloc size must be the integral multiple of granularity
    auto alloc_size = total_size - (total_size % granularity) + granularity;

    cuda_err = cuMemAddressReserve(&gpu_buffer, alloc_size, 0, 0, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(this->get_logger(), "Failed to call cuMemAddressReserve %s", error_str);
      throw std::runtime_error(
              "[NITROS Bridge] cuMemAddressReserve Error");
    }

    cuda_err = cuMemMap(gpu_buffer, alloc_size, 0, generic_allocation_handle, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(this->get_logger(), "Failed to call cuMemMap %s", error_str);
      throw std::runtime_error(
              "[NITROS Bridge] cuMemMap Error");
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
      throw std::runtime_error(
              "[NITROS Bridge] cuMemSetAccess Error");
    }
    handle_ptr_map_[msg_fd] = gpu_buffer;
  }

  nvidia::isaac_ros::nitros::NitrosTensorList nitros_tensor_list;
  nitros_tensor_list.set_timestamp_sec(msg->header.stamp.sec);
  nitros_tensor_list.set_timestamp_nsec(msg->header.stamp.nanosec);
  nitros_tensor_list.set_frame_id(msg->header.frame_id);

  // Create a shared counter for all tensors from this message
  auto tensor_list_counter = std::make_shared<std::atomic<int>>(msg->tensors.size());

  size_t offset = 0;
  auto host_ipc_buffer_ptr = host_ipc_buffer;
  for (size_t i = 0; i < msg->tensors.size(); i++) {
    auto ros_tensor = msg->tensors[i];
    const size_t tensor_size = ros_tensor.strides[0] * ros_tensor.shape.dims[0];
    auto tensor_shape = std::vector<int32_t>{
      ros_tensor.shape.dims.begin(), ros_tensor.shape.dims.end()};

    auto deleter = [host_ipc_buffer_ptr, tensor_list_counter](uint8_t * p) {
        (void)p;
        if (host_ipc_buffer_ptr) {
          if (tensor_list_counter->fetch_sub(1) == 1) {
            host_ipc_buffer_ptr->refcount_dec();
          }
        }
      };
    nvidia::isaac_ros::nitros::NitrosTensor cur_tensor;
    const auto data_type = nvidia::isaac_ros::nitros::convert_to_nitros_data_type(
      static_cast<int32_t>(ros_tensor.data_type));
    if (data_type == nvidia::isaac_ros::nitros::NitrosDataType::kUnknown) {
      throw std::invalid_argument("[NITROS Bridge] Unknown tensor data type: " +
        std::to_string(ros_tensor.data_type));
    }
    {
      [[maybe_unused]] auto write_handle = cur_tensor.from_external(
        ros_tensor.name,
        reinterpret_cast<void *>(gpu_buffer + offset),
        tensor_size,
        nvidia::isaac_ros::nitros::NitrosTensorShape(tensor_shape),
        data_type,
        cuda_stream_,
        deleter);
    }
    nitros_tensor_list.add_tensor(std::move(cur_tensor));
    offset += tensor_size;
  }

  nitros_pub_->publish(nitros_tensor_list);

  RCLCPP_DEBUG(this->get_logger(), "NITROS Tensor List is Published from NITROS Bridge.");
}

void TensorListConverterNode::ROSToBridgeCallback(
  const nvidia::isaac_ros::nitros::NitrosTensorList::SharedPtr msg)
{
  cuCtxSetCurrent(ctx_);

  auto tensor_count = msg->num_tensors();
  // Get total size first
  if (tensor_count == 0) {
    RCLCPP_INFO(this->get_logger(), "No tensor found in the list.");
    return;
  }
  size_t total_size = 0;

  // Get total size of all tensors
  for (size_t i = 0; i < tensor_count; i++) {
    total_size += msg->get_tensor(i).tensor_size();
  }

  // Create IPC buffer manager
  if (first_msg_received_ == false) {
    ipc_buffer_manager_ = std::make_shared<IPCBufferManager>(
      num_blocks_, total_size, timeout_);
    first_msg_received_ = true;
  }

  auto ipc_buffer = ipc_buffer_manager_->find_next_available_buffer();
  isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList tensor_list_msg;
  tensor_list_msg.header.frame_id = msg->get_frame_id();
  tensor_list_msg.header.stamp.sec = static_cast<int32_t>(msg->get_timestamp_sec());
  tensor_list_msg.header.stamp.nanosec = msg->get_timestamp_nsec();

  size_t offset = 0;
  for (size_t i = 0; i < tensor_count; i++) {
    const auto & tensor = msg->get_tensor(i);
    const auto tensor_shape = tensor.shape();
    const auto tensor_rank = tensor_shape.rank();
    const auto tensor_element_type = tensor.data_type();
    const auto tensor_name = tensor.get_name();
    const auto tensor_size = tensor.tensor_size();
    std::vector<uint32_t> tensor_dims;
    const auto tensor_shape_dims = tensor_shape.dims();
    for (uint32_t d = 0; d < tensor_rank; ++d) {
      tensor_dims.push_back(static_cast<uint32_t>(tensor_shape_dims[d]));
    }

    isaac_ros_tensor_list_interfaces::msg::Tensor ros2_tensor;
    ros2_tensor.name = tensor_name;
    ros2_tensor.shape.dims = tensor_dims;
    ros2_tensor.data_type = static_cast<int32_t>(tensor_element_type);
    ros2_tensor.shape.rank = tensor_rank;
    ros2_tensor.strides = tensor.strides();

    auto read_handle = tensor.get_read_handle(cuda_stream_);
    auto cuda_runtime_err = cudaStreamSynchronize(cuda_stream_);
    if (cuda_runtime_err != cudaSuccess) {
      RCLCPP_ERROR(
        this->get_logger(), "cudaStreamSynchronize failed: %s",
        cudaGetErrorString(cuda_runtime_err));
      throw std::runtime_error("[NITROS Bridge] cudaStreamSynchronize Error");
    }

    auto cuda_err = cuMemcpyDtoD(
      ipc_buffer->d_ptr + offset,
      reinterpret_cast<CUdeviceptr>(read_handle.get_ptr()),
      tensor_size);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(
        this->get_logger(), "Failed to call cuMemcpyDtoD %s",
        error_str);
      throw std::runtime_error("[NITROS Bridge] cuMemcpyDtoD Error");
    }
    offset += tensor_size;

    tensor_list_msg.tensors.push_back(ros2_tensor);
  }

  // cuMemcpyDtoD is an aysnchronize call, wait until it complete.
  cuCtxSynchronize();

  tensor_list_msg.pid = ipc_buffer->pid;
  tensor_list_msg.fd = ipc_buffer->fd;
  tensor_list_msg.uid = ipc_buffer->uid;
  tensor_list_msg.device_id = 0;

  nitros_bridge_pub_->publish(tensor_list_msg);
}

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros_bridge::TensorListConverterNode)
