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
#ifndef DETECTION3_D_ARRAY_MESSAGE__DETECTION3_D_ARRAY_MESSAGE_HPP_
#define DETECTION3_D_ARRAY_MESSAGE__DETECTION3_D_ARRAY_MESSAGE_HPP_

#include <string>
#include <vector>

#include "engine/core/math/pose3.hpp"
#include "gxf/core/entity.hpp"
#include "gxf/core/expected.hpp"
#include "gxf/std/tensor.hpp"
#include "gxf/std/timestamp.hpp"

namespace nvidia
{
namespace isaac
{

struct ObjectHypothesis
{
  std::vector<float> scores;
  std::vector<std::string> class_ids;
};

struct Detection3DListMessageParts
{
  gxf::Entity entity;
  FixedVector<gxf::Handle<::nvidia::isaac::Pose3d>, kMaxComponents> poses;
  FixedVector<gxf::Handle<::nvidia::isaac::Vector3f>, kMaxComponents> bbox_sizes;
  FixedVector<gxf::Handle<ObjectHypothesis>, kMaxComponents> hypothesis;
  gxf::Handle<gxf::Timestamp> timestamp;
  size_t count;
};

gxf::Expected<Detection3DListMessageParts> CreateDetection3DListMessage(
  gxf_context_t context, size_t detections);
gxf::Expected<Detection3DListMessageParts> GetDetection3DListMessage(gxf::Entity entity);

}  // namespace isaac
}  // namespace nvidia

#endif  // DETECTION3_D_ARRAY_MESSAGE__DETECTION3_D_ARRAY_MESSAGE_HPP_
