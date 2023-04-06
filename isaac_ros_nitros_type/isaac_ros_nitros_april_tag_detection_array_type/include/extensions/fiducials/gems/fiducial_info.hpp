/*
Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

namespace nvidia
{
namespace isaac
{

// Data structure holding meta information about fiducial messages.
struct FiducialInfo
{
  // A fiducial can be of type (April Tag, QRCode, Barcode or ARTag)
  enum class Type
  {
    kAprilTag,
    kQrCode,
    kBarcode,
    kArTag,
  };
  // Enum to identify the type of fiducial represented by the message
  Type type;
  // Text field that identifies the ID of the fiducial
  // For AprilTag, the id is of the format <TagFamily_ID>
  // Ex. If the decoded tag ID is 14 and belongs to TagFamily tag36h11, the id is tag36h11_14
  std::string id;
};

}  // namespace isaac
}  // namespace nvidia
