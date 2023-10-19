// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_nitros/utils/vpi_utilities.hpp"

#include <string>
#include <unordered_map>

namespace nvidia
{
namespace isaac_ros
{
namespace nitros
{

namespace
{

// Map from human-friendly backend string to VPI backend as uint32_t
const std::unordered_map<std::string, uint32_t> g_str_to_vpi_backend({
          {"CPU", VPI_BACKEND_CPU},
          {"CUDA", VPI_BACKEND_CUDA},
          {"PVA", VPI_BACKEND_PVA},
          {"VIC", VPI_BACKEND_VIC},
          {"NVENC", VPI_BACKEND_NVENC},
          {"TEGRA", VPI_BACKEND_TEGRA},
          {"ALL", VPI_BACKEND_ALL},
        });

}  // namespace

uint32_t DeclareVPIBackendParameter(rclcpp::Node * node, uint32_t default_backends) noexcept
{
  const std::string DEFAULT_BACKENDS_STRING{""};
  const std::string backends_string{node->declare_parameter("backends", DEFAULT_BACKENDS_STRING)};

  // If the ROS 2 parameter is still at the default value, then return the default backend
  if (backends_string == DEFAULT_BACKENDS_STRING) {
    return default_backends;
  }

  // Final backend to be returned after combining all requested backends
  uint32_t backends{};

  std::stringstream stream{backends_string};
  while (stream.good()) {
    // Extract next backend delimited by commas
    std::string backend_str;
    std::getline(stream, backend_str, ',');

    // Search the map for the backend
    auto backend_it{g_str_to_vpi_backend.find(backend_str)};
    if (backend_it != g_str_to_vpi_backend.end()) {
      // If found, bitwise-or the backend to add it into the allowable backends
      backends |= backend_it->second;
    } else {
      // Otherwise, log an error with all allowable backends
      std::ostringstream os{};
      os << "Backend '" << backend_str << "' from requested backends '" << backends_string <<
        "' not recognized. Backend must be one of:" <<
        std::endl;
      std::for_each(
        g_str_to_vpi_backend.begin(), g_str_to_vpi_backend.end(), [&os](auto & entry) {
          os << entry.first << std::endl;
        });

      RCLCPP_ERROR(node->get_logger(), os.str().c_str());

      // Return default backends due to error
      return default_backends;
    }
  }

  // Once all backends have been bitwise-or'ed together, return the result
  return backends;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
VPIFormat ToVpiFormat(VideoFormat value)
{
  switch (value) {
    case VideoFormat::GXF_VIDEO_FORMAT_NV12:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_NV12,
        .pixel_type = {VPI_PIXEL_TYPE_U8, VPI_PIXEL_TYPE_2U8}};
    case VideoFormat::GXF_VIDEO_FORMAT_NV12_ER:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_NV12_ER,
        .pixel_type = {VPI_PIXEL_TYPE_U8, VPI_PIXEL_TYPE_2U8}};
    case VideoFormat::GXF_VIDEO_FORMAT_RGBA:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_RGBA8, .pixel_type = {VPI_PIXEL_TYPE_4U8}};
    case VideoFormat::GXF_VIDEO_FORMAT_BGRA:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_BGRA8, .pixel_type = {VPI_PIXEL_TYPE_4U8}};
    case VideoFormat::GXF_VIDEO_FORMAT_RGB:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_RGB8, .pixel_type = {VPI_PIXEL_TYPE_3U8}};
    case VideoFormat::GXF_VIDEO_FORMAT_BGR:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_BGR8, .pixel_type = {VPI_PIXEL_TYPE_3U8}};
    case VideoFormat::GXF_VIDEO_FORMAT_GRAY:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_U8, .pixel_type = {VPI_PIXEL_TYPE_U8}};
    case VideoFormat::GXF_VIDEO_FORMAT_GRAY16:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_U16, .pixel_type = {VPI_PIXEL_TYPE_U16}};
    case VideoFormat::GXF_VIDEO_FORMAT_GRAY32:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_U32, .pixel_type = {VPI_PIXEL_TYPE_U32}};
    case VideoFormat::GXF_VIDEO_FORMAT_NV24:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_NV24,
        .pixel_type = {VPI_PIXEL_TYPE_U8, VPI_PIXEL_TYPE_2U8}};
    case VideoFormat::GXF_VIDEO_FORMAT_NV24_ER:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_NV24_ER,
        .pixel_type = {VPI_PIXEL_TYPE_U8, VPI_PIXEL_TYPE_2U8}};
    default:
      return VPIFormat{.image_format = VPI_IMAGE_FORMAT_RGB8, .pixel_type = {VPI_PIXEL_TYPE_3U8}};
  }
}
#pragma GCC diagnostic pop

}  // namespace nitros
}  // namespace isaac_ros
}  // namespace nvidia
