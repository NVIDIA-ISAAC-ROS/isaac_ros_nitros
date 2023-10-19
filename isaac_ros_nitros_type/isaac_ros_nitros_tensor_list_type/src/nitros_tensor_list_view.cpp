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

#include <string>
#include <vector>

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list_view.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

void NitrosTensorListView::InitView()
{
  GetAllTensorEntity();
}

void NitrosTensorListView::GetAllTensorEntity()
{
  auto gxf_tensors = msg_entity_->findAll<gxf::Tensor>();
  if (!gxf_tensors) {
    std::stringstream error_msg;
    error_msg <<
      "[GetTensorCount] failed to get all GXF tensors: " <<
      GxfResultStr(gxf_tensors.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTensorListView"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  tensor_list_ = gxf_tensors.value();
}

size_t NitrosTensorListView::GetTensorCount() const
{
  return tensor_list_.size();
}

const NitrosTensorListView::NitrosTensorView NitrosTensorListView::GetAnyNamedTensor(
  std::string tensor_name) const
{
  auto gxf_tensor = msg_entity_->get<gxf::Tensor>(tensor_name.c_str());
  if (!gxf_tensor) {
    std::stringstream error_msg;
    error_msg <<
      "[GetAnyNamedTensor] failed to get GXF tensor of name : [" << tensor_name << "] : " <<
      GxfResultStr(gxf_tensor.error());
    RCLCPP_ERROR(
      rclcpp::get_logger("NitrosTensorListView"), error_msg.str().c_str());
    throw std::runtime_error(error_msg.str().c_str());
  }
  return NitrosTensorListView::NitrosTensorView(*(gxf_tensor.value()));
}

const NitrosTensorListView::NitrosTensorView NitrosTensorListView::GetNamedTensor(
  std::string tensor_name) const
{
  size_t tensor_count = 0;
  // Throw error in case of more than 1 tensor with the same name
  for (auto tensor : tensor_list_) {
    std::string name = std::string(tensor.value().name());
    if (name != tensor_name) {continue;}
    if (name == tensor_name) {++tensor_count;}
    if (tensor_count > 1) {
      std::stringstream error_msg;
      error_msg << "[GetNamedTensor] failed to get unique GXF tensor of name : [" <<
        tensor_name << \
        "]. More than 1 exists.";
      RCLCPP_ERROR(
        rclcpp::get_logger("NitrosTensorListView"), error_msg.str().c_str());
      throw std::runtime_error(error_msg.str().c_str());
    }
  }
  return GetAnyNamedTensor(tensor_name);
}

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
