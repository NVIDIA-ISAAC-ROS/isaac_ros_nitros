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

#ifndef ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_VIEW_HPP_
#define ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_VIEW_HPP_

#include <string>
#include <vector>

#include "isaac_ros_nitros/types/nitros_type_view_factory.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_shape.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{
NITROS_TYPE_VIEW_FACTORY_BEGIN_NO_GXF(NitrosTensorList)

MARK_PUBLIC_SECTION()
inline size_t GetTensorCount() const {return msg_.get_tensors().size();}
inline const NitrosTensor & GetTensorByName(const std::string & name) const
{
  auto tensor = msg_.get_tensor_by_name(name);
  if (!tensor) {
    throw std::runtime_error("Tensor with name '" + name + "' not found");
  }
  return *tensor;
}
inline const NitrosTensor & GetNamedTensor(const std::string & name) const
{
  return GetTensorByName(name);
}
inline const unsigned char * GetBufferByName(const std::string & name) const
{
  return GetTensorByName(name).get_read_handle(msg_.get_stream()).get_ptr();
}
inline const NitrosTensor & get_tensor(size_t index = 0) const
{
  if (index >= msg_.get_tensors().size()) {
    throw std::out_of_range("Tensor index " + std::to_string(index) + " out of range");
  }
  return msg_.get_tensors().at(index);
}
inline const unsigned char * GetBuffer(size_t index = 0) const
{
  return get_tensor(index).get_read_handle(msg_.get_stream()).get_ptr();
}
inline const std::string GetName(size_t index = 0) const
{
  return get_tensor(index).get_name();
}

inline uint32_t GetRank(size_t index = 0) const
{
  return get_tensor(index).shape().rank();
}
inline uint64_t GetBytesPerElement(size_t index = 0) const
{
  return get_tensor(index).bytes_per_element();
}
inline uint64_t GetElementCount(size_t index = 0) const
{
  return get_tensor(index).element_count();
}
inline size_t GetDimension(size_t index = 0, size_t dimension = 0) const
{
  const auto & tensor = get_tensor(index);
  if (dimension >= tensor.shape().rank()) {
    throw std::out_of_range("Dimension index " + std::to_string(dimension) + " out of range");
  }
  return tensor.shape().dims()[dimension];
}
inline size_t GetTensorSize(size_t index = 0) const
{
  return get_tensor(index).element_count() * get_tensor(index).bytes_per_element();
}
inline NitrosTensorShape GetShape(size_t index = 0) const
{
  return get_tensor(index).shape();
}

MARK_PUBLIC_SECTION()
NITROS_TYPE_VIEW_FACTORY_END_NO_GXF(NitrosTensorList)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_VIEW_HPP_
