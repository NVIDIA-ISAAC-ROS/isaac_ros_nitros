// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2019-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <utility>

#include "engine/core/tensor/sample_cloud.hpp"
#include "engine/gems/serialization/json.hpp"

namespace nvidia {
namespace isaac {
namespace sight {

// Sight shop operation for point clouds.
class SopPointCloud {
 public:
  // Delete copy constructor
  SopPointCloud(const SopPointCloud&) = delete;
  SopPointCloud(SopPointCloud&&) = default;

  // Creates a show operation for point clouds
  // This function assumes that the data is 2 dimensionsal and consists of floats.
  // down_sample_stride can be used to render less data than provided, but
  // will force an aditional copy at this time.
  static SopPointCloud Create(
      SampleCloudConstView2f points,
      SampleCloudConstView3f colors = SampleCloudConstView3f(),
      int downsample_stride = 1);

  // Creates a show operation for point clouds
  // This function assumes that the data is 3 dimensionsal and consists of floats.
  // down_sample_stride can be used to render less data than provided, but
  // will force an aditional copy at this time.
  static SopPointCloud Create(
      SampleCloudConstView3f points,
      SampleCloudConstView3f colors = SampleCloudConstView3f(),
      int downsample_stride = 1);

 private:
  friend const Json& ToJson(const SopPointCloud&);
  friend Json ToJson(SopPointCloud&&);

  // Private to allow construction from the static function
  SopPointCloud() = default;

  Json json_;
};

// Returns the json of a SopPointCloud
inline const Json& ToJson(const SopPointCloud& sop) {
  return sop.json_;
}

inline Json ToJson(SopPointCloud&& sop) {
  return std::move(sop.json_);
}

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
