// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <array>
#include <string>

#include "gxf/core/component.hpp"
#include "gxf/std/allocator.hpp"
#include "gxf/std/tensor.hpp"

namespace nvidia {
namespace isaac {

// This component loads data from an image file into a 2 or 3-dimensional tensor. Supports both
// grayscale and color PNGs and color JPEGs. On initialize, codelet allocates the
// appropriate amount of memory for the tensor based on the provided dimensions.
class ImageLoader : public gxf::Component {
 public:
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override;

  gxf_result_t registerInterface(gxf::Registrar* registrar) override;

 private:
  gxf::Parameter<std::string> filename_;
  gxf::Parameter<gxf::Handle<gxf::Allocator>> allocator_;
  gxf::Parameter<gxf::Handle<gxf::Tensor>> tensor_;
  gxf::Parameter<bool> color_;
};

}  // namespace isaac
}  // namespace nvidia
