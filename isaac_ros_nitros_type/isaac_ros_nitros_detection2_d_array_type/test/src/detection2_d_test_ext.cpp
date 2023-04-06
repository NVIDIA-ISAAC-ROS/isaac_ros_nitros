/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <string>

#include "gxf/core/gxf.h"
#include "gxf/std/extension_factory_helper.hpp"
#include "detectnet/detection2_d.hpp"

extern "C" {

GXF_EXT_FACTORY_BEGIN()

GXF_EXT_FACTORY_SET_INFO(
  0x94485739160245e4, 0x8ef19134f30ad931, "Detection2DTestMessageExtension",
  "Detection2D Message GXF extension",
  "NVIDIA", "1.0.0", "LICENSE");

GXF_EXT_FACTORY_ADD_0(
  0xa4c9101525594104, 0xaf12d9f22a134906,
  std::vector<nvidia::isaac_ros::Detection2D>,
  "Array of decoded 2D object detections in an image");

GXF_EXT_FACTORY_END()

}  // extern "C"
