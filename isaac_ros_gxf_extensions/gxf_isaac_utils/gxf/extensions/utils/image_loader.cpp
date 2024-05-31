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
#include "extensions/utils/image_loader.hpp"

#include "gems/core/image/image.hpp"
#include "gems/core/tensor/tensor.hpp"
#include "gems/gxf_helpers/image_view.hpp"
#include "gems/image/io.hpp"

namespace nvidia {
namespace isaac {

gxf_result_t ImageLoader::registerInterface(gxf::Registrar* registrar) {
  gxf::Expected<void> result;
  result &= registrar->parameter(
      filename_, "filename", "Filename", "Path to PNG or JPG/JPEG image file to load",
      gxf::Registrar::NoDefaultParameter(), GXF_PARAMETER_FLAGS_OPTIONAL);
  result &= registrar->parameter(
      allocator_, "allocator", "Allocator",
      "Allocator for tensor");
  result &= registrar->parameter(
      tensor_, "tensor", "Tensor",
      "Tensor to contain the loaded image data");
  result &= registrar->parameter(
      color_, "color", "Color",
      "Whether the loaded image is loaded with color or not. False by default.", false);
  return gxf::ToResultCode(result);
}

gxf_result_t ImageLoader::initialize() {
  if (!filename_.try_get()) {
    GXF_LOG_WARNING("Not passing filename, so not using image loader");
    *(tensor_.get()) = gxf::Tensor();
    return GXF_SUCCESS;
  }

  const auto& filename = filename_.try_get().value();

  // Get image dimensions
  ::nvidia::isaac::Vector3i dimensions;
  if (!::nvidia::isaac::LoadImageShape(filename, dimensions)) {
    GXF_LOG_ERROR("Failed to load image shape.");
    return GXF_FAILURE;
  }
  gxf::Shape shape;
  if (color_.get()) {
    shape = {dimensions[0], dimensions[1], 3};
  } else {
    shape = {dimensions[0], dimensions[1]};
  }

  // Reshape tensor
  auto tensor_reshape_result = tensor_->reshape<uint8_t>(
       shape, gxf::MemoryStorageType::kHost, allocator_);
  if (!tensor_reshape_result) {
    GXF_LOG_ERROR("Tensor reshape failed");
    return gxf::ToResultCode(tensor_reshape_result);
  }

  // Convert to Isaac TensorView
  if (color_.get()) {
    auto maybe_view = ToIsaacImageView<uint8_t, 3>(*tensor_.get());
    if (!maybe_view) {
      GXF_LOG_ERROR("Could not create view");
      return gxf::ToResultCode(maybe_view);
    }
    // Load image from file
    if (!::nvidia::isaac::LoadImage(filename, maybe_view.value())) {
      GXF_LOG_ERROR("Failed to load image with color.");
      return GXF_FAILURE;
    }
  } else {
    auto maybe_view = ToIsaacImageView<uint8_t, 1>(*tensor_.get());
    if (!maybe_view) {
      GXF_LOG_ERROR("Could not create view");
      return gxf::ToResultCode(maybe_view);
    }
    // Load image from file
    if (!::nvidia::isaac::LoadImage(filename, maybe_view.value())) {
      GXF_LOG_ERROR("Failed to load image without color.");
      return GXF_FAILURE;
    }
  }
  return GXF_SUCCESS;
}

gxf_result_t ImageLoader::deinitialize() {
  // Clears loaded Tensor to avoid racing destruction with Allocator instance
  *(tensor_.get()) = gxf::Tensor();
  return GXF_SUCCESS;
}

}  // namespace isaac
}  // namespace nvidia
