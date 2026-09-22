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

#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "isaac_ros_nitros_bridge_ros2/tensor_list_converter_node.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor.hpp"
#include "isaac_ros_tensor_msgs/tensor_utils.hpp"

#define SYS_pidfd_getfd_nitros_bridge 438

namespace
{

struct DType
{
  uint8_t code;
  uint8_t bits;
  uint16_t lanes;
};

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

DType FromBridgeDataType(int32_t data_type)
{
  switch (data_type) {
    case 1:
      return {0, 8, 1};
    case 2:
      return {1, 8, 1};
    case 3:
      return {0, 16, 1};
    case 4:
      return {1, 16, 1};
    case 5:
      return {0, 32, 1};
    case 6:
      return {1, 32, 1};
    case 7:
      return {0, 64, 1};
    case 8:
      return {1, 64, 1};
    case 9:
      return {2, 32, 1};
    case 10:
      return {2, 64, 1};
    default:
      throw std::invalid_argument("Unsupported bridge tensor data type");
  }
}

int32_t ToBridgeDataType(const tensor_msgs::msg::ExperimentalTensor & tensor)
{
  if (tensor.dtype_lanes != 1) {
    throw std::invalid_argument("Bridge tensors do not support vector lanes");
  }
  if (tensor.dtype_code == 0) {
    switch (tensor.dtype_bits) {
      case 8:
        return 1;
      case 16:
        return 3;
      case 32:
        return 5;
      case 64:
        return 7;
      default:
        break;
    }
  } else if (tensor.dtype_code == 1) {
    switch (tensor.dtype_bits) {
      case 8:
        return 2;
      case 16:
        return 4;
      case 32:
        return 6;
      case 64:
        return 8;
      default:
        break;
    }
  } else if (tensor.dtype_code == 2) {
    switch (tensor.dtype_bits) {
      case 32:
        return 9;
      case 64:
        return 10;
      default:
        break;
    }
  }
  throw std::invalid_argument("Unsupported DLPack tensor data type");
}

size_t BytesPerElement(const tensor_msgs::msg::ExperimentalTensor & tensor)
{
  if (tensor.dtype_bits == 0 || tensor.dtype_bits % 8 != 0 ||
    tensor.dtype_lanes == 0)
  {
    throw std::invalid_argument(
        "Tensor data type does not have a whole-byte element size");
  }
  return static_cast<size_t>(tensor.dtype_bits / 8) * tensor.dtype_lanes;
}

size_t BridgeStorageBytes(const tensor_msgs::msg::ExperimentalTensor & tensor)
{
  const size_t element_size = BytesPerElement(tensor);
  const size_t storage_count =
    isaac_ros_tensor_msgs::RequiredStorageElements(tensor);
  if (storage_count > std::numeric_limits<size_t>::max() / element_size) {
    throw std::overflow_error("Tensor storage size overflow");
  }
  return storage_count * element_size;
}

}  // namespace

namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

TensorListConverterNode::TensorListConverterNode(
  const rclcpp::NodeOptions options)
: rclcpp::Node("tensor_list_converter_node", options),
  num_blocks_(declare_parameter<int64_t>("num_blocks", 40)),
      // Timeout in microseconds: duration to wait after refcount reaches 0
      // before recycling the buffer
  timeout_(declare_parameter<int64_t>("timeout", 500)),
  tensor_list_pub_qos_{::isaac_ros::common::AddQosParameter(
      *this, "DEFAULT", "ros_pub_qos")},
  tensor_list_sub_qos_{::isaac_ros::common::AddQosParameter(
      *this, "DEFAULT", "ros_sub_qos")},
  bridge_pub_qos_{::isaac_ros::common::AddQosParameter(
      *this, "DEFAULT", "bridge_pub_qos")},
  bridge_sub_qos_{::isaac_ros::common::AddQosParameter(
      *this, "DEFAULT", "bridge_sub_qos")}
{
  cudaSetDevice(0);
  cuDevicePrimaryCtxRetain(&ctx_, 0);
  auto cuda_runtime_err =
    cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
  if (cuda_runtime_err != cudaSuccess) {
    throw std::runtime_error("[NITROS Bridge] cudaStreamCreateWithFlags Error");
  }

  rclcpp::PublisherOptions tensor_list_pub_options;
  tensor_list_pub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Enable;
  rclcpp::SubscriptionOptions tensor_list_sub_options;
  tensor_list_sub_options.use_intra_process_comm =
    rclcpp::IntraProcessSetting::Enable;
  tensor_list_sub_options.acceptable_buffer_backends = "any";

  tensor_list_pub_ = create_publisher<isaac_ros_tensor_msgs::msg::TensorList>(
      "ros2_output_tensor_list", tensor_list_pub_qos_, tensor_list_pub_options);

  tensor_list_sub_ = create_subscription<isaac_ros_tensor_msgs::msg::TensorList>(
      "ros2_input_tensor_list", tensor_list_sub_qos_,
      std::bind(
        &TensorListConverterNode::ROSToBridgeCallback, this,
        std::placeholders::_1),
      tensor_list_sub_options);

  nitros_bridge_pub_ = create_publisher<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>(
      "ros2_output_bridge_tensor_list", bridge_pub_qos_);
  nitros_bridge_sub_ = create_subscription<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>(
      "ros2_input_bridge_tensor_list", bridge_sub_qos_,
      std::bind(
        &TensorListConverterNode::BridgeToROSCallback, this,
        std::placeholders::_1));
}

TensorListConverterNode::~TensorListConverterNode()
{
  if (cuda_stream_ != nullptr) {
    cudaStreamSynchronize(cuda_stream_);
    cudaStreamDestroy(cuda_stream_);
    cuda_stream_ = nullptr;
  }
  cuDevicePrimaryCtxRelease(0);
}

void TensorListConverterNode::BridgeToROSCallback(
  const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList::
  SharedPtr msg)
{
  cuCtxSetCurrent(ctx_);

  if (msg->tensors.empty()) {
    RCLCPP_ERROR(get_logger(), "Bridge tensor list is empty.");
    return;
  }

  CUdeviceptr gpu_buffer = 0ULL;
  CUmemGenericAllocationHandle generic_allocation_handle;

  const auto msg_pid = msg->pid;
  const auto msg_fd = msg->fd;
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

  size_t total_size = 0;
  for (size_t i = 0; i < msg->tensors.size(); i++) {
    const auto & tensor = msg->tensors[i];
    if (tensor.shape.rank == 0 ||
      tensor.shape.dims.size() != tensor.shape.rank)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid tensor shape.");
      return;
    }
    if (tensor.strides.size() != tensor.shape.rank) {
      RCLCPP_ERROR(this->get_logger(), "Invalid tensor strides.");
      return;
    }
    total_size += tensor.strides[0] * tensor.shape.dims[0];
  }

  std::unique_ptr<HostIpcRefGuard> host_ipc_guard;
  if (!msg_uid.empty()) {
    const std::string shm_name =
      std::to_string(msg_pid) + std::to_string(msg_fd);
    auto host_ipc_buffer =
      std::make_shared<HostIPCBuffer>(shm_name, HostIPCBuffer::Mode::OPEN);
    if (!host_ipc_buffer->refcoun_inc_if_uid_match(msg_uid)) {
      RCLCPP_WARN(this->get_logger(), "Failed to match UID, skip.");
      return;
    }
    host_ipc_guard = std::make_unique<HostIpcRefGuard>(std::move(host_ipc_buffer));
  }

  if (handle_ptr_map_.find(msg_fd) != handle_ptr_map_.end()) {
    gpu_buffer = handle_ptr_map_[msg_fd];
    RCLCPP_DEBUG(this->get_logger(), "Found FD in local map.");
  } else {
    int pidfd = syscall(SYS_pidfd_open, msg_pid, 0);
    if (pidfd <= 0) {
      RCLCPP_ERROR(get_logger(), "SYS_pidfd_open failed.");
      return;
    }
    int imported_fd = syscall(SYS_pidfd_getfd_nitros_bridge, pidfd, msg_fd, 0);
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

    auto alloc_size = total_size - (total_size % granularity) + granularity;

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
    handle_ptr_map_[msg_fd] = gpu_buffer;
  }

  auto tensor_list = std::make_unique<isaac_ros_tensor_msgs::msg::TensorList>();
  tensor_list->header = msg->header;
  size_t offset = 0;
  try {
    for (const auto & bridge_tensor : msg->tensors) {
      const DType dtype = FromBridgeDataType(bridge_tensor.data_type);
      const size_t bytes_per_element =
        static_cast<size_t>(dtype.bits / 8) * dtype.lanes;
      const size_t tensor_size =
        bridge_tensor.strides[0] * bridge_tensor.shape.dims[0];

      tensor_msgs::msg::ExperimentalTensor tensor;
      tensor.dtype_code = dtype.code;
      tensor.dtype_bits = dtype.bits;
      tensor.dtype_lanes = dtype.lanes;
      tensor.shape.assign(
          bridge_tensor.shape.dims.begin(), bridge_tensor.shape.dims.end());
      tensor.strides.reserve(bridge_tensor.strides.size());
      for (const uint64_t byte_stride : bridge_tensor.strides) {
        if (byte_stride % bytes_per_element != 0) {
          RCLCPP_ERROR(
            get_logger(),
            "Tensor byte stride is not divisible by element size.");
          return;
        }
        tensor.strides.push_back(
            static_cast<int64_t>(byte_stride / bytes_per_element));
      }
      tensor.byte_offset = 0;
      tensor.data = cuda_buffer_backend::allocate_buffer(tensor_size);
      {
        auto write_handle =
          cuda_buffer_backend::from_output_buffer(tensor.data, cuda_stream_);
        auto cuda_err = cudaMemcpyAsync(
            write_handle.get_ptr(),
            reinterpret_cast<void *>(gpu_buffer + offset), tensor_size,
            cudaMemcpyDeviceToDevice, cuda_stream_);
        if (cuda_err != cudaSuccess) {
          RCLCPP_ERROR(
            get_logger(), "Failed to copy bridge tensor: %s",
            cudaGetErrorString(cuda_err));
          return;
        }
      }
      tensor_list->names.push_back(bridge_tensor.name);
      tensor_list->tensors.push_back(std::move(tensor));
      offset += tensor_size;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to convert bridge tensors: %s", e.what());
    return;
  }

  auto cuda_err = cudaStreamSynchronize(cuda_stream_);
  if (cuda_err != cudaSuccess) {
    RCLCPP_ERROR(
      get_logger(), "Failed to synchronize bridge tensor copies: %s",
      cudaGetErrorString(cuda_err));
    return;
  }
  tensor_list_pub_->publish(std::move(tensor_list));
}

void TensorListConverterNode::ROSToBridgeCallback(
  const isaac_ros_tensor_msgs::msg::TensorList::SharedPtr msg)
{
  cuCtxSetCurrent(ctx_);

  const size_t tensor_count = msg->tensors.size();
  if (tensor_count == 0) {
    RCLCPP_INFO(this->get_logger(), "No tensor found in the list.");
    return;
  }
  if (msg->names.size() != tensor_count) {
    RCLCPP_ERROR(
      get_logger(),
      "Tensor names and tensors must have matching sizes.");
    return;
  }

  size_t total_size = 0;
  std::vector<size_t> tensor_sizes;
  tensor_sizes.reserve(tensor_count);
  try {
    for (const auto & tensor : msg->tensors) {
      const size_t tensor_size = BridgeStorageBytes(tensor);
      if (tensor.byte_offset > tensor.data.size() ||
        tensor_size > tensor.data.size() - tensor.byte_offset)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Tensor data buffer is smaller than its described view.");
        return;
      }
      if (total_size > std::numeric_limits<size_t>::max() - tensor_size) {
        RCLCPP_ERROR(get_logger(), "Tensor list size overflow.");
        return;
      }
      tensor_sizes.push_back(tensor_size);
      total_size += tensor_size;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid tensor list for bridge: %s", e.what());
    return;
  }

  if (first_msg_received_ == false) {
    ipc_buffer_manager_ =
      std::make_shared<IPCBufferManager>(num_blocks_, total_size, timeout_);
    ipc_buffer_bytes_ = total_size;
    first_msg_received_ = true;
  } else if (total_size > ipc_buffer_bytes_) {
    RCLCPP_ERROR(
      get_logger(),
      "Tensor list payload (%zu bytes) exceeds IPC pool size (%zu bytes).",
      total_size, ipc_buffer_bytes_);
    return;
  }

  auto ipc_buffer = ipc_buffer_manager_->find_next_available_buffer();
  isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList
    tensor_list_msg;
  tensor_list_msg.header = msg->header;

  size_t offset = 0;
  try {
    for (size_t i = 0; i < tensor_count; ++i) {
      const auto & tensor = msg->tensors[i];
      const size_t tensor_size = tensor_sizes[i];
      const size_t bytes_per_element = BytesPerElement(tensor);
      isaac_ros_tensor_list_interfaces::msg::Tensor ros2_tensor;
      ros2_tensor.name = msg->names[i];
      ros2_tensor.shape.rank = tensor.shape.size();
      ros2_tensor.shape.dims.reserve(tensor.shape.size());
      for (const int64_t dimension : tensor.shape) {
        if (dimension <= 0 || static_cast<uint64_t>(dimension) >
          std::numeric_limits<uint32_t>::max())
        {
          RCLCPP_ERROR(
            get_logger(),
            "Tensor dimension is outside the bridge message range.");
          return;
        }
        ros2_tensor.shape.dims.push_back(static_cast<uint32_t>(dimension));
      }
      ros2_tensor.data_type = ToBridgeDataType(tensor);
      ros2_tensor.strides.reserve(tensor.shape.size());
      for (size_t dimension = 0; dimension < tensor.shape.size(); ++dimension) {
        const size_t element_stride =
          isaac_ros_tensor_msgs::StrideInElements(tensor, dimension);
        ros2_tensor.strides.push_back(element_stride * bytes_per_element);
      }

      auto read_handle =
        cuda_buffer_backend::from_input_buffer(tensor.data, cuda_stream_);
      auto cuda_err = cudaMemcpyAsync(
          reinterpret_cast<void *>(ipc_buffer->d_ptr + offset),
          read_handle.get_ptr() + tensor.byte_offset, tensor_size,
          cudaMemcpyDeviceToDevice, cuda_stream_);
      if (cuda_err != cudaSuccess) {
        RCLCPP_ERROR(
          get_logger(), "Failed to copy tensor into bridge memory: %s",
          cudaGetErrorString(cuda_err));
        return;
      }
      offset += tensor_size;

      tensor_list_msg.tensors.push_back(ros2_tensor);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to convert tensors to bridge: %s", e.what());
    return;
  }

  auto cuda_err = cudaStreamSynchronize(cuda_stream_);
  if (cuda_err != cudaSuccess) {
    RCLCPP_ERROR(
      get_logger(), "Failed to synchronize tensor copies: %s",
      cudaGetErrorString(cuda_err));
    return;
  }

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
RCLCPP_COMPONENTS_REGISTER_NODE(
    nvidia::isaac_ros::nitros_bridge::TensorListConverterNode)
