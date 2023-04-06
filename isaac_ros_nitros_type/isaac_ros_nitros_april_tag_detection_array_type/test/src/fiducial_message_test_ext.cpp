/*
Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "extensions/fiducials/gems/fiducial_info.hpp"
#include "gxf/std/extension_factory_helper.hpp"

GXF_EXT_FACTORY_BEGIN()

GXF_EXT_FACTORY_SET_INFO(
  0xd8d7816ec0485ad4, 0xff795a414bd445ca, "FiducialsTestMessageExtension",
  "Test extension for fiducials messages",
  "NVIDIA", "1.0.0", "LICENSE");

GXF_EXT_FACTORY_ADD_0(
  0xe91d3fa6a42b85ff, 0x966f4e80c607ca9e,
  nvidia::isaac::FiducialInfo,
  "Holds fiducial meta information.");

GXF_EXT_FACTORY_END()
