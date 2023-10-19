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

#ifndef ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_DETAILS_HPP_
#define ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_DETAILS_HPP_

#include <string>
#include <unordered_map>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/multimedia/video.hpp"
#pragma GCC diagnostic pop

#include "sensor_msgs/image_encodings.hpp"
#include "vpi/Image.h"

namespace
{
using VideoFormat = nvidia::gxf::VideoFormat;
namespace img_encodings = sensor_msgs::image_encodings;
// Map to store the ROS format encoding to Nitros format encoding
const std::unordered_map<std::string, VideoFormat> g_ros_to_gxf_video_format({
    {img_encodings::RGB8, VideoFormat::GXF_VIDEO_FORMAT_RGB},
    {img_encodings::RGBA8, VideoFormat::GXF_VIDEO_FORMAT_RGBA},
    {img_encodings::RGB16, VideoFormat::GXF_VIDEO_FORMAT_RGB16},
    {img_encodings::BGR8, VideoFormat::GXF_VIDEO_FORMAT_BGR},
    {img_encodings::BGRA8, VideoFormat::GXF_VIDEO_FORMAT_BGRA},
    {img_encodings::BGR16, VideoFormat::GXF_VIDEO_FORMAT_BGR16},
    {img_encodings::MONO8, VideoFormat::GXF_VIDEO_FORMAT_GRAY},
    {img_encodings::MONO16, VideoFormat::GXF_VIDEO_FORMAT_GRAY16},
    {img_encodings::TYPE_32FC1, VideoFormat::GXF_VIDEO_FORMAT_GRAY32},
    {img_encodings::NV24, VideoFormat::GXF_VIDEO_FORMAT_NV24_ER},
    {"nv12", VideoFormat::GXF_VIDEO_FORMAT_NV12_ER},
  });

// Map to store the Nitros format encoding to ROS format encoding
const std::unordered_map<VideoFormat, std::string> g_gxf_to_ros_video_format({
    {VideoFormat::GXF_VIDEO_FORMAT_RGB, img_encodings::RGB8},
    {VideoFormat::GXF_VIDEO_FORMAT_RGBA, img_encodings::RGBA8},
    {VideoFormat::GXF_VIDEO_FORMAT_RGB16, img_encodings::RGB16},
    {VideoFormat::GXF_VIDEO_FORMAT_BGR, img_encodings::BGR8},
    {VideoFormat::GXF_VIDEO_FORMAT_BGRA, img_encodings::BGRA8},
    {VideoFormat::GXF_VIDEO_FORMAT_BGR16, img_encodings::BGR16},
    {VideoFormat::GXF_VIDEO_FORMAT_GRAY, img_encodings::MONO8},
    {VideoFormat::GXF_VIDEO_FORMAT_GRAY16, img_encodings::MONO16},
    {VideoFormat::GXF_VIDEO_FORMAT_GRAY32, img_encodings::TYPE_32FC1},
    {VideoFormat::GXF_VIDEO_FORMAT_D32F, img_encodings::TYPE_32FC1},
    {VideoFormat::GXF_VIDEO_FORMAT_NV24_ER, img_encodings::NV24},
    {VideoFormat::GXF_VIDEO_FORMAT_NV12_ER, "nv12"},
  });

template<VideoFormat T>
struct NoPaddingColorPlanes {};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_RGB>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("RGB", 3, width * 3)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_BGR>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("BGR", 3, width * 3)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_RGBA>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("RGBA", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_BGRA>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("BGRA", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_RGB16>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("RGB16", 6, width * 6)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_BGR16>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("BGR16", 6, width * 6)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_GRAY>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("GRAY", 1, width)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_GRAY16>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("GRAY", 2, width * 2)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_GRAY32>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("GRAY", 4, width * 4)}) {}
  std::array<nvidia::gxf::ColorPlane, 1> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_NV12>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("Y", 1, width),
        nvidia::gxf::ColorPlane("UV", 2, width * 2)}) {}
  std::array<nvidia::gxf::ColorPlane, 2> planes;
};

template<>
struct NoPaddingColorPlanes<VideoFormat::GXF_VIDEO_FORMAT_NV24>
{
  NoPaddingColorPlanes(size_t width)
  : planes({nvidia::gxf::ColorPlane("Y", 1, width),
        nvidia::gxf::ColorPlane("UV", 2, width * 2)}) {}
  std::array<nvidia::gxf::ColorPlane, 2> planes;
};

}  // namespace

#endif  // ISAAC_ROS_NITROS_IMAGE_TYPE__NITROS_IMAGE_DETAILS_HPP_
