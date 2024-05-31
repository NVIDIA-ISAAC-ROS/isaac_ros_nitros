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

#include "extensions/utils/disparity_to_depth.hpp"

#include <numeric>

#include "extensions/messages/camera_message.hpp"
#include "extensions/utils/disparity_to_depth.cu.hpp"
#include "gems/gxf_helpers/expected_macro_gxf.hpp"

namespace nvidia {
namespace isaac {

gxf_result_t DisparityToDepth::registerInterface(gxf::Registrar* registrar) {
  gxf::Expected<void> result;

  result &= registrar->parameter(
      disparity_input_, "disparity_input", "Input disparity",
      "Incoming CameraMessage frames with disparity");
  result &= registrar->parameter(
      depth_output_, "depth_output", "Output depth",
      "Computed depth maps as CameraMessage");
  result &= registrar->parameter(
      allocator_, "allocator", "Allocator",
      "Allocator to allocate output messages");

  return gxf::ToResultCode(result);
}

gxf_result_t DisparityToDepth::start() {
  return GXF_SUCCESS;
}

gxf_result_t DisparityToDepth::stop() {
  return GXF_SUCCESS;
}

gxf_result_t DisparityToDepth::tick() {
  gxf::Entity message = UNWRAP_OR_RETURN(disparity_input_->receive());

  CameraMessageParts disparity_message =
    UNWRAP_OR_RETURN(GetCameraMessage(message));

  gxf::VideoBufferInfo disparity_info = disparity_message.frame->video_frame_info();

  // validate input message
  if (disparity_message.frame->storage_type() != gxf::MemoryStorageType::kDevice) {
    GXF_LOG_ERROR("Input disparity image must be stored in "
                  "gxf::MemoryStorageType::kDevice");
    return GXF_INVALID_DATA_FORMAT;
  }

  if (disparity_info.color_format != gxf::VideoFormat::GXF_VIDEO_FORMAT_D32F) {
    GXF_LOG_ERROR("Input disparity must be of type "
                  "gxf::VideoFormat::GXF_VIDEO_FORMAT_D32F");
    return GXF_INVALID_DATA_FORMAT;
  }

  CameraMessageParts depth_message =
    UNWRAP_OR_RETURN(CreateCameraMessage<gxf::VideoFormat::GXF_VIDEO_FORMAT_D32F>(
        context(),
        disparity_info.width,
        disparity_info.height,
        disparity_info.surface_layout,
        gxf::MemoryStorageType::kDevice,
        allocator_,
        false));  // TODO(kpatzwaldt): change to true and adjust kernel accordingly

  // compute parameters
  float focal_length = disparity_message.intrinsics->focal_length.x;

  std::array<float, 3> translation = disparity_message.extrinsics->translation;
  float baseline = std::sqrt(std::inner_product(translation.begin(),
                                                translation.end(),
                                                translation.begin(),
                                                0.0L));

  // call CUDA kernel and wait for completion
  disparity_to_depth_cuda(
      reinterpret_cast<const float *>(disparity_message.frame->pointer()),
      reinterpret_cast<float *>(depth_message.frame->pointer()),
      baseline, focal_length, disparity_info.height, disparity_info.width);

  // forward other components as is
  *depth_message.intrinsics = *disparity_message.intrinsics;
  *depth_message.extrinsics = *disparity_message.extrinsics;
  *depth_message.sequence_number = *disparity_message.sequence_number;
  *depth_message.timestamp = *disparity_message.timestamp;

  RETURN_IF_ERROR(depth_output_->publish(depth_message.entity));

  return GXF_SUCCESS;
}

}  // namespace isaac
}  // namespace nvidia
