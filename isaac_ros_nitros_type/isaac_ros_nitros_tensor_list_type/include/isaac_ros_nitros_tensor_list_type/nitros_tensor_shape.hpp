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

#ifndef ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_SHAPE_HPP_
#define ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_SHAPE_HPP_

#include <vector>

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{
constexpr uint32_t kMaxRank = 4;
class NitrosTensorShape
{
public:
  NitrosTensorShape() = default;
  explicit NitrosTensorShape(int32_t rank, const int32_t * dims)
  : rank_(rank)
  {
    for (int32_t i = 0; i < rank; i++) {
      dims_.push_back(dims[i]);
    }
  }

  explicit NitrosTensorShape(const std::initializer_list<int32_t> & dimensions)
  : rank_(dimensions.size())
  {
    for (int32_t dim : dimensions) {
      dims_.push_back(dim);
    }
  }

  explicit NitrosTensorShape(const std::vector<int32_t> & dimensions)
  : rank_(static_cast<int32_t>(dimensions.size())), dims_(dimensions) {}

  explicit NitrosTensorShape(const std::vector<uint32_t> & dimensions)
  : rank_(dimensions.size())
  {
    for (uint32_t dim : dimensions) {
      dims_.push_back(static_cast<int32_t>(dim));
    }
  }

  uint32_t rank() const {return rank_;}
  const std::vector<int32_t> & dims() const
  {
    return dims_;
  }

private:
  int32_t rank_{0};
  std::vector<int32_t> dims_{};
};

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_TENSOR_LIST_TYPE__NITROS_TENSOR_SHAPE_HPP_
