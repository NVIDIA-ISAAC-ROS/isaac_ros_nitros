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
  nitros_pub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
        nvidia::isaac_ros::nitros::NitrosTensorList>>(
      this, "ros2_output_tensor_list",
      nvidia::isaac_ros::nitros::nitros_tensor_list_nhwc_t::supported_type_name)},
  nitros_bridge_pub_{
    create_publisher<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>(
      "ros2_output_bridge_tensor_list", 10)},
  nitros_sub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosTensorListView>>(
      this, "ros2_input_tensor_list",
      nvidia::isaac_ros::nitros::nitros_tensor_list_nhwc_t::supported_type_name,
      std::bind(&TensorListConverterNode::ROSToBridgeCallback, this,
      std::placeholders::_1))},
  nitros_bridge_sub_{
    create_subscription<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>(
      "ros2_input_bridge_tensor_list", 10,
      std::bind(&TensorListConverterNode::BridgeToROSCallback, this,
      std::placeholders::_1))},
  num_blocks_(declare_parameter<int64_t>("num_blocks", 40)),
  timeout_(declare_parameter<int64_t>("timeout", 500))
{
  cudaSetDevice(0);
  cuDevicePrimaryCtxRetain(&ctx_, 0);

  cudaEventCreateWithFlags(&event_, cudaEventInterprocess | cudaEventDisableTiming);
  cudaIpcGetEventHandle(reinterpret_cast<cudaIpcEventHandle_t *>(&ipc_event_handle_), event_);
}

TensorListConverterNode::~TensorListConverterNode() = default;


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

  // Compare UID if it exists
  // Compare UID if exists
  if (!msg_uid.empty()) {
    std::string shm_name = std::to_string(msg_pid) + std::to_string(msg_fd);
    host_ipc_buffer_ = std::make_shared<HostIPCBuffer>(shm_name, HostIPCBuffer::Mode::OPEN);
    if (!host_ipc_buffer_->refcoun_inc_if_uid_match(msg_uid)) {
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

  auto nitros_tensor_list_builder =
    nvidia::isaac_ros::nitros::NitrosTensorListBuilder()
    .WithHeader(msg->header);

  int offset = 0;
  for (size_t i = 0; i < msg->tensors.size(); i++) {
    auto ros_tensor = msg->tensors[i];
    auto tensor_shape = std::vector<int32_t>{
      ros_tensor.shape.dims.begin(), ros_tensor.shape.dims.end()};

    auto cur_tensor = nvidia::isaac_ros::nitros::NitrosTensorBuilder()
      .WithShape(nvidia::isaac_ros::nitros::NitrosTensorShape(tensor_shape))
      .WithDataType(nvidia::isaac_ros::nitros::NitrosDataType::kFloat32)
      .WithData(reinterpret_cast<void *>(gpu_buffer + offset))
      .Build();
    nitros_tensor_list_builder.AddTensor(ros_tensor.name.c_str(), cur_tensor);
    offset += ros_tensor.data.size();
  }

  auto nitros_tensor_list = nitros_tensor_list_builder.Build();

  nitros_pub_->publish(nitros_tensor_list);

  // Update refcount if it exists
  if (!msg_uid.empty()) {
    host_ipc_buffer_->refcount_dec();
  }

  RCLCPP_DEBUG(this->get_logger(), "NITROS Tensor List is Published from NITROS Bridge.");
}

void TensorListConverterNode::ROSToBridgeCallback(
  const nvidia::isaac_ros::nitros::NitrosTensorListView view)
{
  cuCtxSetCurrent(ctx_);

  auto tensor_count = view.GetTensorCount();
  // Get total size first
  if (tensor_count == 0) {
    RCLCPP_INFO(this->get_logger(), "No tensor found in the list.");
    return;
  }
  auto tensor_list = view.GetAllTensor();
  size_t total_size = 0;

  // Get total size of all tensors
  for (size_t i = 0; i < tensor_count; i++) {
    auto tensor = tensor_list[i];
    auto tensor_bytes_per_element = tensor.GetBytesPerElement();
    auto tensor_element_count = tensor.GetElementCount();
    total_size += tensor_element_count * tensor_bytes_per_element;
  }

  // Create IPC buffer manager
  if (first_msg_received_ == false) {
    ipc_buffer_manager_ = std::make_shared<IPCBufferManager>(
      num_blocks_, total_size, timeout_);
    first_msg_received_ = true;
  }

  auto ipc_buffer = ipc_buffer_manager_->find_next_available_buffer();
  isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList tensor_list_msg;
  tensor_list_msg.header.frame_id = view.GetFrameId();
  tensor_list_msg.header.stamp.sec = view.GetTimestampSeconds();
  tensor_list_msg.header.stamp.nanosec = view.GetTimestampNanoseconds();

  size_t offset = 0;
  for (size_t i = 0; i < tensor_count; i++) {
    auto tensor = tensor_list[i];
    auto tensor_shape = tensor.GetShape().shape();
    auto tensor_rank = tensor.GetRank();
    auto tensor_element_type = tensor.GetElementType();
    auto tensor_name = tensor.GetName();
    auto tensor_bytes_per_element = tensor.GetBytesPerElement();
    auto tensor_element_count = tensor.GetElementCount();
    std::vector<uint32_t> tensor_dims;
    std::vector<uint64_t> tensor_strides;
    for (uint32_t i = 0; i < tensor_rank; ++i) {
      tensor_dims.push_back(tensor_shape.dimension(i));
    }

    isaac_ros_tensor_list_interfaces::msg::Tensor ros2_tensor;
    ros2_tensor.name = tensor_name;
    ros2_tensor.shape.dims = tensor_dims;
    ros2_tensor.data_type = static_cast<uint8_t>(tensor_element_type);
    ros2_tensor.shape.rank = tensor_rank;
    ros2_tensor.strides = tensor.GetStrides();

    auto cuda_err = cuMemcpyDtoD(
      ipc_buffer->d_ptr + offset,
      (CUdeviceptr)(tensor.GetBuffer()),
      tensor_element_count * tensor_bytes_per_element);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(
        rclcpp::get_logger(""), "Failed to call cuMemcpyDtoD %s",
        error_str);
      throw std::runtime_error("[NITROS Bridge] cuMemcpyDtoD Error");
    }
    offset += tensor_element_count * tensor_bytes_per_element;

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
