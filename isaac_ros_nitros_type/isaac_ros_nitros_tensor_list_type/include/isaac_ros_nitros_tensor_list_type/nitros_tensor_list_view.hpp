// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/core/gxf.h"
#include "gxf/std/tensor.hpp"
#pragma GCC diagnostic pop

#include "isaac_ros_nitros/types/nitros_type_view_factory.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

NITROS_TYPE_VIEW_FACTORY_BEGIN(NitrosTensorList)
MARK_PRIVATE_SECTION()
class NitrosTensorView
{
public:
  explicit NitrosTensorView(const gxf::Tensor & tensor)
  : tensor_{tensor} {}
  inline const unsigned char * GetBuffer() const {return tensor_.pointer();}
  inline uint32_t GetRank() const {return tensor_.rank();}
  inline uint64_t GetBytesPerElement() const {return tensor_.bytes_per_element();}
  inline uint64_t GetElementCount() const {return tensor_.element_count();}
  inline size_t GetTensorSize() const {return tensor_.size();}

private:
  const gxf::Tensor & tensor_{};
};

MARK_PUBLIC_SECTION()
// Public methods
size_t GetTensorCount() const;
const NitrosTensorView GetAnyNamedTensor(std::string tensor_name) const;
const NitrosTensorView GetNamedTensor(std::string tensor_name) const;

MARK_PRIVATE_SECTION()
void GetAllTensorEntity();
FixedVector<gxf::Handle<gxf::Tensor>, kMaxComponents> tensor_list_;
NITROS_TYPE_VIEW_FACTORY_END(NitrosTensorList)

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_VIEW_HPP_
