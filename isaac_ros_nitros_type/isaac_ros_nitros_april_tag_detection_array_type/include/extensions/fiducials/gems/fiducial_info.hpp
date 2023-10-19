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
