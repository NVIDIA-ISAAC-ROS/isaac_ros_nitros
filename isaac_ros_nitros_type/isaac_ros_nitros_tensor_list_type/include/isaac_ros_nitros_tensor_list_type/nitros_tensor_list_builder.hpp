// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_BUILDER_HPP_
#define ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_BUILDER_HPP_

#include <string>

#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_list.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_tensor_shape.hpp"
#include "isaac_ros_nitros_tensor_list_type/nitros_data_type.hpp"

#include "std_msgs/msg/header.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

class NitrosTensorListBuilder
{
public:
  NitrosTensorListBuilder();

  NitrosTensorListBuilder & WithHeader(std_msgs::msg::Header header);

  NitrosTensorListBuilder & AddTensor(std::string name, NitrosTensor tensor);

  NitrosTensorList Build();

private:
  NitrosTensorList nitros_tensor_list_{};
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_LIST_BUILDER_HPP_
