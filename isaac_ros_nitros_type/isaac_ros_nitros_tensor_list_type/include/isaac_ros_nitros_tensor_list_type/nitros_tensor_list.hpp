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

#ifndef ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_HPP_
#define ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_HPP_

#ifndef NITROS_GXF_COMPAT_MODE
#define NITROS_GXF_COMPAT_MODE
#endif

#include <cuda_runtime.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <map>
#include <utility>
#include "isaac_ros_nitros/types/nitros_format_agent.hpp"
#include "isaac_ros_nitros/types/nitros_type_base.hpp"
#include "isaac_ros_tensor_list_interfaces/msg/tensor_list.hpp"
#include "isaac_ros_nitros/types/nitros_buffer.hpp"
#include "isaac_ros_nitros/types/cuda_memory_pool.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_data_type.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_shape.hpp"
#include "rclcpp/type_adapter.hpp"

#ifdef NITROS_GXF_COMPAT_MODE
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_gxf_compat.hpp"
#endif

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

// Type forward declaration
struct NitrosTensorList;

NitrosDataType convert_to_nitros_data_type(int32_t data_type);
int32_t convert_to_ros_data_type(NitrosDataType nitros_data_type);

// Format descriptors for NitrosTypeManager and compatible data format negotiation
struct nitros_tensor_list_nchw_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nchw";
};

struct nitros_tensor_list_nhwc_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nhwc";
};

struct nitros_tensor_list_nchw_rgb_f32_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nchw_rgb_f32";
};

struct nitros_tensor_list_nhwc_rgb_f32_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nhwc_rgb_f32";
};

struct nitros_tensor_list_nchw_bgr_f32_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nchw_bgr_f32";
};

struct nitros_tensor_list_nhwc_bgr_f32_t
{
  using MsgT = NitrosTensorList;
  static const inline std::string supported_type_name = "nitros_tensor_list_nhwc_bgr_f32";
};

class NitrosTensorList : public NitrosTypeBase
{
public:
#ifdef NITROS_GXF_COMPAT_MODE
  static std::map<std::string, NitrosFormatCallbacks> GetFormatCallbacks();
  static std::vector<std::pair<std::string, std::string>> GetExtensions() {return {};}
#else
  static std::map<std::string, NitrosFormatCallbacks> GetFormatCallbacks() {return {};}
  static std::vector<std::pair<std::string, std::string>> GetExtensions() {return {};}
#endif
  static std::string GetDefaultCompatibleFormat()
  {
    return nitros_tensor_list_nchw_rgb_f32_t::supported_type_name;
  }

  NitrosTensorList()
  : NitrosTypeBase()
  {
#ifndef NITROS_GXF_COMPAT_MODE
    handle = -1;
#endif
  }
  explicit NitrosTensorList(const NitrosTypeBase & base)
  : NitrosTypeBase(base) {}

  explicit NitrosTensorList(cudaStream_t stream)
  : stream_(stream)
  {
#ifndef NITROS_GXF_COMPAT_MODE
    handle = -1;
#endif
  }

  // Standard ROS2 message pointer type aliases (required by message_filters)
  using SharedPtr = std::shared_ptr<NitrosTensorList>;
  using ConstSharedPtr = std::shared_ptr<const NitrosTensorList>;
  using UniquePtr = std::unique_ptr<NitrosTensorList>;
  using ConstUniquePtr = std::unique_ptr<const NitrosTensorList>;
  using WeakPtr = std::weak_ptr<NitrosTensorList>;
  using ConstWeakPtr = std::weak_ptr<const NitrosTensorList>;
  using ConstPtr = const NitrosTensorList *;

  // Message metadata accessors
  cudaStream_t get_stream() const {return stream_;}
  void set_stream(cudaStream_t stream) {stream_ = stream;}
  std::shared_ptr<CUDAMemoryPool> get_pool() {return pool_;}
  const std::vector<NitrosTensor> & get_tensors() const {return tensors_;}
  void add_tensor(const NitrosTensor & tensor) {tensors_.push_back(tensor);}
  void add_tensor(NitrosTensor && tensor) {tensors_.push_back(std::move(tensor));}
  size_t num_tensors() const {return tensors_.size();}
  const NitrosTensor & get_tensor(size_t i) const {return tensors_.at(i);}

  // Get read handle for a specific tensor (default: first tensor)
  ReadHandle get_read_handle(
    cudaStream_t stream, size_t tensor_index = 0) const
  {
    return tensors_.at(tensor_index).get_read_handle(stream);
  }
  std::shared_ptr<NitrosTensor> get_tensor_by_name(const std::string & name) const
  {
    for (auto & tensor : tensors_) {
      if (tensor.get_name() == name) {
        return std::make_shared<NitrosTensor>(tensor);
      }
    }
    return nullptr;
  }

  cudaMemoryType get_storage_type() const {return storage_type_;}
  void set_storage_type(cudaMemoryType storage_type) {storage_type_ = storage_type;}
  std_msgs::msg::Header get_header() const
  {
    std_msgs::msg::Header header;
    header.stamp.sec = static_cast<int32_t>(timestamp_sec_);
    header.stamp.nanosec = static_cast<uint32_t>(timestamp_nsec_);
    header.frame_id = frame_id_;
    return header;
  }
  uint32_t get_timestamp_sec() const override {return timestamp_sec_;}
  uint32_t get_timestamp_nsec() const override {return timestamp_nsec_;}
  void set_timestamp_sec(uint32_t sec) override {timestamp_sec_ = sec;}
  void set_timestamp_nsec(uint32_t nsec) override {timestamp_nsec_ = nsec;}
  const std::string & get_frame_id() const {return frame_id_;}
  void set_frame_id(const std::string & frame_id) {frame_id_ = frame_id;}

private:
  friend class NitrosBufferAccessor<NitrosTensorList>;
  friend struct rclcpp::TypeAdapter<NitrosTensorList,
    isaac_ros_tensor_list_interfaces::msg::TensorList>;

  std::vector<NitrosTensor> tensors_{};
  std::shared_ptr<CUDAMemoryPool> pool_;
  cudaStream_t stream_{nullptr};
  uint32_t timestamp_sec_{0};
  uint32_t timestamp_nsec_{0};
  cudaMemoryType storage_type_{cudaMemoryTypeDevice};
  std::string frame_id_;
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

template<>
struct rclcpp::TypeAdapter<
  nvidia::isaac_ros::nitros::NitrosTensorList,
  isaac_ros_tensor_list_interfaces::msg::TensorList>
{
  using is_specialized = std::true_type;
  using custom_type = nvidia::isaac_ros::nitros::NitrosTensorList;
  using ros_message_type = isaac_ros_tensor_list_interfaces::msg::TensorList;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination);

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  nvidia::isaac_ros::nitros::NitrosTensorList,
  isaac_ros_tensor_list_interfaces::msg::TensorList);

#endif  // ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_HPP_
