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
#pragma once

#include <vector>

#include "gems/core/math/types.hpp"
#include "gems/core/tensor/sample_cloud.hpp"
#include "gems/sight/sop_serializer.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Sight show operation for point clouds.
class SopPointCloud : public SopSerializer {
 public:
  SopPointCloud() = default;
  SopPointCloud(SopPointCloud&&) = default;
  ~SopPointCloud() override = default;

  // Creates a SopPointCloud with 2d points
  SopPointCloud(::nvidia::isaac::SampleCloudConstView2f points, int downsample_stride = 1);
  SopPointCloud(::nvidia::isaac::SampleCloudConstView2f points,
                ::nvidia::isaac::SampleCloudConstView3ub colors,
                int downsample_stride = 1);

  // Creates a SopPointCloud with 3d points
  SopPointCloud(::nvidia::isaac::SampleCloudConstView3f points, int downsample_stride = 1);
  SopPointCloud(::nvidia::isaac::SampleCloudConstView3f points,
                ::nvidia::isaac::SampleCloudConstView3ub colors,
                int downsample_stride = 1);

  // Construct a sop by calling fromBinary
  SopPointCloud(BufferSerialization& buffer);

  bool toBinary(BufferSerialization& buffer) const override;
  bool fromBinary(BufferSerialization& buffer) override;

 private:
  int dim_ = 0;
  std::vector<float> pts_;
  std::vector<uint8_t> colors_;
};

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
